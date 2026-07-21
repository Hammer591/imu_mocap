#!/usr/bin/env python3
"""Jetson USB CDC client for STM32 mocap protocol V3.

Payload layout: 60 little-endian float32 values.

Typical complete acquisition:

    python3 jetson_mocap_v3.py --port /dev/ttyACM0 run \
        --csv mocap.csv --print-every 20

The program opens the CDC device, waits for a heartbeat, sends START_STREAM,
receives data, and sends STOP_STREAM before closing.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None

    class SerialException(Exception):
        pass


SOF = b"\xAA\x55"
PROTOCOL_VERSION = 3

MSG_DATA = 0x01
MSG_HEARTBEAT = 0x02
MSG_COMMAND = 0x10
MSG_RESPONSE = 0x11
MSG_EVENT = 0x20

CMD_START_STREAM = 0x0001
CMD_STOP_STREAM = 0x0002
CMD_GET_STATUS = 0x0003
CMD_PING = 0x0004

RSP_OK = 0x0000

STATE_BOOT = 0
STATE_IDLE = 1
STATE_STREAMING = 2
STATE_ERROR = 3

FLAG_STREAM = 1 << 0
FLAG_TIMESTAMP = 1 << 1
FLAG_WARNING = 1 << 2
FLAG_ACK_REQUIRED = 1 << 3
FLAG_RESPONSE = 1 << 4
FLAG_ERROR = 1 << 5

VALID_LEFT_THIGH = 1 << 0
VALID_LEFT_SHANK = 1 << 1
VALID_RIGHT_THIGH = 1 << 2
VALID_RIGHT_SHANK = 1 << 3
VALID_LEFT_FOOT = 1 << 4
VALID_RIGHT_FOOT = 1 << 5
VALID_ALL = 0x003F

HEADER_FORMAT = "<2sBBBBHIQHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_FORMAT = "<I"
CRC_SIZE = struct.calcsize(CRC_FORMAT)

DATA_FORMAT = "<60f"
DATA_PAYLOAD_SIZE = struct.calcsize(DATA_FORMAT)
HEARTBEAT_FORMAT = "<IIIII"
HEARTBEAT_PAYLOAD_SIZE = struct.calcsize(HEARTBEAT_FORMAT)
RESPONSE_FORMAT = "<HH"
RESPONSE_PAYLOAD_SIZE = struct.calcsize(RESPONSE_FORMAT)
MAX_PAYLOAD_SIZE = DATA_PAYLOAD_SIZE

STATE_NAMES = {
    STATE_BOOT: "BOOT",
    STATE_IDLE: "IDLE",
    STATE_STREAMING: "STREAMING",
    STATE_ERROR: "ERROR",
}

COMMAND_NAMES = {
    CMD_START_STREAM: "START_STREAM",
    CMD_STOP_STREAM: "STOP_STREAM",
    CMD_GET_STATUS: "GET_STATUS",
    CMD_PING: "PING",
}

LEG_FIELDS = (
    "roll", "pitch", "yaw",
    "gyrox", "gyroy", "gyroz",
    "ax", "ay", "az",
)
FOOT_FIELDS = (
    "copx", "copy", "grf",
    "roll", "pitch", "yaw",
    "gyrox", "gyroy", "gyroz",
    "ax", "ay", "az",
)

CSV_COLUMNS = [
    *[f"left_thigh_{field}" for field in LEG_FIELDS],
    *[f"left_shank_{field}" for field in LEG_FIELDS],
    *[f"right_thigh_{field}" for field in LEG_FIELDS],
    *[f"right_shank_{field}" for field in LEG_FIELDS],
    *[f"left_foot_{field}" for field in FOOT_FIELDS],
    *[f"right_foot_{field}" for field in FOOT_FIELDS],
]


class PacketError(ValueError):
    """Raised when a protocol packet is malformed."""


@dataclass(frozen=True)
class PacketHeader:
    version: int
    message_type: int
    flags: int
    unit_count: int
    payload_length: int
    sequence: int
    timestamp_us: int
    meta0: int
    meta1: int


@dataclass(frozen=True)
class GenericPacket:
    header: PacketHeader
    payload: bytes
    crc32: int


@dataclass(frozen=True)
class LegImuSample:
    roll: float
    pitch: float
    yaw: float
    gyrox: float
    gyroy: float
    gyroz: float
    ax: float
    ay: float
    az: float


@dataclass(frozen=True)
class FootSample:
    copx: float
    copy: float
    grf: float
    roll: float
    pitch: float
    yaw: float
    gyrox: float
    gyroy: float
    gyroz: float
    ax: float
    ay: float
    az: float


@dataclass(frozen=True)
class MocapDataPacket:
    header: PacketHeader
    data: tuple[float, ...]

    @property
    def valid_mask(self) -> int:
        return self.header.meta0

    def unit_is_valid(self, mask: int) -> bool:
        return bool(self.valid_mask & mask)

    @property
    def left_thigh(self) -> LegImuSample:
        return LegImuSample(*self.data[0:9])

    @property
    def left_shank(self) -> LegImuSample:
        return LegImuSample(*self.data[9:18])

    @property
    def right_thigh(self) -> LegImuSample:
        return LegImuSample(*self.data[18:27])

    @property
    def right_shank(self) -> LegImuSample:
        return LegImuSample(*self.data[27:36])

    @property
    def left_foot(self) -> FootSample:
        return FootSample(*self.data[36:48])

    @property
    def right_foot(self) -> FootSample:
        return FootSample(*self.data[48:60])


@dataclass(frozen=True)
class HeartbeatPacket:
    header: PacketHeader
    uptime_ms: int
    stream_packets_sent: int
    tx_drop_count: int
    rx_crc_error_count: int
    stream_tick_overrun_count: int

    @property
    def system_state(self) -> int:
        return self.header.meta0

    @property
    def valid_mask(self) -> int:
        return self.header.meta1


@dataclass(frozen=True)
class ResponsePacket:
    header: PacketHeader
    system_state: int
    valid_mask: int

    @property
    def transaction_id(self) -> int:
        return self.header.sequence

    @property
    def command(self) -> int:
        return self.header.meta0

    @property
    def response_code(self) -> int:
        return self.header.meta1


@dataclass
class ParserStatistics:
    packets_ok: int = 0
    crc_errors: int = 0
    header_errors: int = 0
    discarded_bytes: int = 0
    sequence_gaps: int = 0
    estimated_lost_packets: int = 0


def crc32_iso_hdlc(data: bytes | bytearray | memoryview) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def packet_size(payload_length: int) -> int:
    return HEADER_SIZE + payload_length + CRC_SIZE


def encode_packet(
    *,
    message_type: int,
    flags: int,
    unit_count: int,
    sequence: int,
    timestamp_us: int,
    meta0: int,
    meta1: int,
    payload: bytes = b"",
) -> bytes:
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError("payload too large")

    header = struct.pack(
        HEADER_FORMAT,
        SOF,
        PROTOCOL_VERSION,
        message_type,
        flags,
        unit_count,
        len(payload),
        sequence & 0xFFFFFFFF,
        timestamp_us & 0xFFFFFFFFFFFFFFFF,
        meta0 & 0xFFFF,
        meta1 & 0xFFFF,
    )
    body = header + payload
    return body + struct.pack(CRC_FORMAT, crc32_iso_hdlc(body[2:]))


def encode_command(transaction_id: int, command: int, payload: bytes = b"") -> bytes:
    return encode_packet(
        message_type=MSG_COMMAND,
        flags=FLAG_ACK_REQUIRED | FLAG_TIMESTAMP,
        unit_count=0,
        sequence=transaction_id,
        timestamp_us=time.monotonic_ns() // 1000,
        meta0=command,
        meta1=0,
        payload=payload,
    )


def parse_packet(packet: bytes) -> GenericPacket:
    if len(packet) < HEADER_SIZE + CRC_SIZE:
        raise PacketError("packet too short")

    (
        sof, version, message_type, flags, unit_count, payload_length,
        sequence, timestamp_us, meta0, meta1,
    ) = struct.unpack_from(HEADER_FORMAT, packet, 0)

    if sof != SOF:
        raise PacketError("bad SOF")
    if version != PROTOCOL_VERSION:
        raise PacketError(f"bad protocol version: {version}")
    if payload_length > MAX_PAYLOAD_SIZE:
        raise PacketError(f"payload too large: {payload_length}")

    expected = packet_size(payload_length)
    if len(packet) != expected:
        raise PacketError(f"bad packet length: {len(packet)} != {expected}")

    crc_offset = HEADER_SIZE + payload_length
    received_crc = struct.unpack_from(CRC_FORMAT, packet, crc_offset)[0]
    calculated_crc = crc32_iso_hdlc(packet[2:crc_offset])
    if received_crc != calculated_crc:
        raise PacketError(
            f"bad CRC: received=0x{received_crc:08X}, "
            f"calculated=0x{calculated_crc:08X}"
        )

    return GenericPacket(
        header=PacketHeader(
            version=version,
            message_type=message_type,
            flags=flags,
            unit_count=unit_count,
            payload_length=payload_length,
            sequence=sequence,
            timestamp_us=timestamp_us,
            meta0=meta0,
            meta1=meta1,
        ),
        payload=packet[HEADER_SIZE:crc_offset],
        crc32=received_crc,
    )


def decode_typed(
    packet: GenericPacket,
) -> MocapDataPacket | HeartbeatPacket | ResponsePacket | GenericPacket:
    h = packet.header

    if h.message_type == MSG_DATA:
        if h.unit_count != 6:
            raise PacketError(f"bad unit count: {h.unit_count}")
        if h.payload_length != DATA_PAYLOAD_SIZE:
            raise PacketError(f"bad data payload length: {h.payload_length}")
        return MocapDataPacket(h, struct.unpack(DATA_FORMAT, packet.payload))

    if h.message_type == MSG_HEARTBEAT:
        if h.payload_length != HEARTBEAT_PAYLOAD_SIZE:
            raise PacketError(
                f"bad heartbeat payload length: {h.payload_length}"
            )
        values = struct.unpack(HEARTBEAT_FORMAT, packet.payload)
        return HeartbeatPacket(h, *values)

    if h.message_type == MSG_RESPONSE:
        if h.payload_length != RESPONSE_PAYLOAD_SIZE:
            raise PacketError(f"bad response payload length: {h.payload_length}")
        system_state, valid_mask = struct.unpack(RESPONSE_FORMAT, packet.payload)
        return ResponsePacket(h, system_state, valid_mask)

    return packet


class MocapStreamParser:
    """Incremental parser that handles half packets, concatenation and resync."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats = ParserStatistics()
        self._last_data_sequence: Optional[int] = None

    def feed(
        self, data: bytes
    ) -> list[MocapDataPacket | HeartbeatPacket | ResponsePacket | GenericPacket]:
        if data:
            self.buffer.extend(data)

        result = []
        while True:
            sof_index = self.buffer.find(SOF)
            if sof_index < 0:
                keep = 1 if self.buffer.endswith(SOF[:1]) else 0
                discard = len(self.buffer) - keep
                if discard > 0:
                    del self.buffer[:discard]
                    self.stats.discarded_bytes += discard
                break

            if sof_index > 0:
                del self.buffer[:sof_index]
                self.stats.discarded_bytes += sof_index

            if len(self.buffer) < HEADER_SIZE:
                break

            payload_length = struct.unpack_from("<H", self.buffer, 6)[0]
            if payload_length > MAX_PAYLOAD_SIZE:
                self.stats.header_errors += 1
                del self.buffer[0]
                self.stats.discarded_bytes += 1
                continue

            total = packet_size(payload_length)
            if len(self.buffer) < total:
                break

            candidate = bytes(self.buffer[:total])
            try:
                typed = decode_typed(parse_packet(candidate))
            except PacketError as exc:
                if "CRC" in str(exc):
                    self.stats.crc_errors += 1
                else:
                    self.stats.header_errors += 1
                del self.buffer[0]
                self.stats.discarded_bytes += 1
                continue

            del self.buffer[:total]
            self.stats.packets_ok += 1
            if isinstance(typed, MocapDataPacket):
                self._update_sequence(typed.header.sequence)
            result.append(typed)

        return result

    def _update_sequence(self, sequence: int) -> None:
        if self._last_data_sequence is not None:
            expected = (self._last_data_sequence + 1) & 0xFFFFFFFF
            if sequence != expected:
                lost = (sequence - expected) & 0xFFFFFFFF
                self.stats.sequence_gaps += 1
                if lost < 0x80000000:
                    self.stats.estimated_lost_packets += lost
        self._last_data_sequence = sequence


class MocapClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        timeout: float = 0.05,
    ) -> None:
        if serial is None:
            raise RuntimeError(
                "pyserial is not installed. Install it with: "
                "sudo apt install python3-serial"
            )

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.parser = MocapStreamParser()

        self._stop_reader = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._heartbeat_event = threading.Event()
        self._latest_heartbeat: Optional[HeartbeatPacket] = None

        self._response_lock = threading.Lock()
        self._response_events: dict[int, threading.Event] = {}
        self._responses: dict[int, ResponsePacket] = {}
        self._transaction_id = 1
        self._write_lock = threading.Lock()

        self.on_data: Optional[Callable[[MocapDataPacket], None]] = None
        self.on_heartbeat: Optional[Callable[[HeartbeatPacket], None]] = None

    def open(self) -> None:
        if self.ser is not None:
            return
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )
        self.ser.dtr = True
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self._stop_reader.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="mocap-usb-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_reader.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        if self.ser is not None:
            try:
                self.ser.dtr = False
            except (SerialException, OSError):
                pass
            self.ser.close()
            self.ser = None

    def wait_for_heartbeat(self, timeout: float = 3.0) -> HeartbeatPacket:
        if not self._heartbeat_event.wait(timeout):
            raise TimeoutError(
                f"No heartbeat from {self.port} within {timeout:.1f} s"
            )
        assert self._latest_heartbeat is not None
        return self._latest_heartbeat

    def send_command(
        self,
        command: int,
        *,
        payload: bytes = b"",
        timeout: float = 0.5,
        retries: int = 3,
    ) -> ResponsePacket:
        if self.ser is None:
            raise RuntimeError("serial port is not open")

        transaction_id = self._next_transaction_id()
        packet = encode_command(transaction_id, command, payload)
        event = threading.Event()

        with self._response_lock:
            self._response_events[transaction_id] = event
            self._responses.pop(transaction_id, None)

        try:
            for attempt in range(1, retries + 1):
                with self._write_lock:
                    assert self.ser is not None
                    self.ser.write(packet)
                    self.ser.flush()

                if event.wait(timeout):
                    with self._response_lock:
                        response = self._responses.pop(transaction_id)
                        self._response_events.pop(transaction_id, None)
                    if response.command != command:
                        raise RuntimeError("response command mismatch")
                    return response

                if attempt < retries:
                    print(
                        f"{COMMAND_NAMES.get(command, hex(command))} timeout, "
                        f"retry {attempt + 1}/{retries}",
                        file=sys.stderr,
                    )

            raise TimeoutError(
                f"No response to {COMMAND_NAMES.get(command, hex(command))}"
            )
        finally:
            with self._response_lock:
                self._response_events.pop(transaction_id, None)
                self._responses.pop(transaction_id, None)

    def start_stream(self) -> ResponsePacket:
        response = self.send_command(CMD_START_STREAM)
        if response.response_code != RSP_OK:
            raise RuntimeError(f"START_STREAM failed: {response.response_code}")
        return response

    def stop_stream(self) -> ResponsePacket:
        response = self.send_command(CMD_STOP_STREAM)
        if response.response_code != RSP_OK:
            raise RuntimeError(f"STOP_STREAM failed: {response.response_code}")
        return response

    def _next_transaction_id(self) -> int:
        value = self._transaction_id
        self._transaction_id = (self._transaction_id + 1) & 0xFFFFFFFF
        if self._transaction_id == 0:
            self._transaction_id = 1
        return value

    def _reader_loop(self) -> None:
        assert self.ser is not None
        try:
            while not self._stop_reader.is_set():
                chunk = self.ser.read(max(self.ser.in_waiting, 1))
                if not chunk:
                    continue
                for packet in self.parser.feed(chunk):
                    self._dispatch(packet)
        except (SerialException, OSError) as exc:
            if not self._stop_reader.is_set():
                print(f"Serial reader stopped: {exc}", file=sys.stderr)

    def _dispatch(
        self,
        packet: MocapDataPacket | HeartbeatPacket | ResponsePacket | GenericPacket,
    ) -> None:
        if isinstance(packet, MocapDataPacket):
            if self.on_data is not None:
                self.on_data(packet)
            return
        if isinstance(packet, HeartbeatPacket):
            self._latest_heartbeat = packet
            self._heartbeat_event.set()
            if self.on_heartbeat is not None:
                self.on_heartbeat(packet)
            return
        if isinstance(packet, ResponsePacket):
            with self._response_lock:
                self._responses[packet.transaction_id] = packet
                event = self._response_events.get(packet.transaction_id)
            if event is not None:
                event.set()


class CsvRecorder:
    def __init__(self, path: Optional[Path], print_every: int) -> None:
        self.path = path
        self.print_every = print_every
        self.handle = None
        self.writer = None
        self.count = 0
        self.pending_flush = 0

    def open(self) -> None:
        if self.path is None:
            return
        self.handle = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.handle)
        self.writer.writerow([
            "host_monotonic_ns", "sequence", "stm32_timestamp_us",
            "flags", "valid_mask", *CSV_COLUMNS,
        ])
        self.handle.flush()

    def close(self) -> None:
        if self.handle is not None:
            self.handle.flush()
            self.handle.close()
            self.handle = None
            self.writer = None

    def on_packet(self, packet: MocapDataPacket) -> None:
        self.count += 1
        if self.print_every > 0 and self.count % self.print_every == 0:
            print_data_packet(packet)
        if self.writer is not None:
            self.writer.writerow([
                time.monotonic_ns(),
                packet.header.sequence,
                packet.header.timestamp_us,
                packet.header.flags,
                packet.valid_mask,
                *packet.data,
            ])
            self.pending_flush += 1
            if self.pending_flush >= 100:
                assert self.handle is not None
                self.handle.flush()
                self.pending_flush = 0


def print_leg(name: str, sample: LegImuSample, valid: bool) -> None:
    print(
        f"  {name:<12} valid={valid} "
        f"rpy=({sample.roll:+.3f},{sample.pitch:+.3f},{sample.yaw:+.3f}) "
        f"gyro=({sample.gyrox:+.3f},{sample.gyroy:+.3f},{sample.gyroz:+.3f}) "
        f"acc=({sample.ax:+.3f},{sample.ay:+.3f},{sample.az:+.3f})"
    )


def print_foot(name: str, sample: FootSample, valid: bool) -> None:
    print(
        f"  {name:<12} valid={valid} "
        f"cop=({sample.copx:+.3f},{sample.copy:+.3f}) grf={sample.grf:+.3f} "
        f"rpy=({sample.roll:+.3f},{sample.pitch:+.3f},{sample.yaw:+.3f}) "
        f"gyro=({sample.gyrox:+.3f},{sample.gyroy:+.3f},{sample.gyroz:+.3f}) "
        f"acc=({sample.ax:+.3f},{sample.ay:+.3f},{sample.az:+.3f})"
    )


def print_data_packet(packet: MocapDataPacket) -> None:
    print(
        f"DATA seq={packet.header.sequence:8d} "
        f"t={packet.header.timestamp_us:14d} us "
        f"valid=0x{packet.valid_mask:04X}"
    )
    print_leg("left_thigh", packet.left_thigh,
              packet.unit_is_valid(VALID_LEFT_THIGH))
    print_leg("left_shank", packet.left_shank,
              packet.unit_is_valid(VALID_LEFT_SHANK))
    print_leg("right_thigh", packet.right_thigh,
              packet.unit_is_valid(VALID_RIGHT_THIGH))
    print_leg("right_shank", packet.right_shank,
              packet.unit_is_valid(VALID_RIGHT_SHANK))
    print_foot("left_foot", packet.left_foot,
               packet.unit_is_valid(VALID_LEFT_FOOT))
    print_foot("right_foot", packet.right_foot,
               packet.unit_is_valid(VALID_RIGHT_FOOT))


def print_heartbeat(packet: HeartbeatPacket) -> None:
    state = STATE_NAMES.get(packet.system_state, str(packet.system_state))
    print(
        f"Heartbeat: state={state}, uptime={packet.uptime_ms / 1000:.1f}s, "
        f"valid=0x{packet.valid_mask:04X}, "
        f"stream_sent={packet.stream_packets_sent}, "
        f"tx_drop={packet.tx_drop_count}, "
        f"rx_crc_error={packet.rx_crc_error_count}, "
        f"tick_overrun={packet.stream_tick_overrun_count}"
    )


def run_session(args: argparse.Namespace) -> None:
    recorder = CsvRecorder(args.csv, args.print_every)
    recorder.open()
    client = MocapClient(args.port, args.baudrate, args.serial_timeout)
    client.on_data = recorder.on_packet

    last_heartbeat_print = 0.0

    def heartbeat_callback(packet: HeartbeatPacket) -> None:
        nonlocal last_heartbeat_print
        now = time.monotonic()
        if now - last_heartbeat_print >= args.heartbeat_print_period:
            print_heartbeat(packet)
            last_heartbeat_print = now

    client.on_heartbeat = heartbeat_callback
    started = False

    try:
        print(f"Opening {args.port} ...")
        client.open()
        heartbeat = client.wait_for_heartbeat(args.heartbeat_timeout)
        print("STM32 detected.")
        print_heartbeat(heartbeat)

        response = client.start_stream()
        started = True
        print(
            "START_STREAM acknowledged: "
            f"state={STATE_NAMES.get(response.system_state, response.system_state)}, "
            f"valid=0x{response.valid_mask:04X}"
        )

        start_time = time.monotonic()
        while True:
            time.sleep(0.1)
            if args.duration > 0 and time.monotonic() - start_time >= args.duration:
                break
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if started:
            try:
                response = client.stop_stream()
                print(
                    "STOP_STREAM acknowledged: "
                    f"state={STATE_NAMES.get(response.system_state, response.system_state)}"
                )
            except Exception as exc:
                print(f"Could not confirm STOP_STREAM: {exc}", file=sys.stderr)
        client.close()
        recorder.close()
        s = client.parser.stats
        print(
            "Parser statistics: "
            f"ok={s.packets_ok}, crc_errors={s.crc_errors}, "
            f"header_errors={s.header_errors}, discarded={s.discarded_bytes}, "
            f"sequence_gaps={s.sequence_gaps}, "
            f"estimated_lost={s.estimated_lost_packets}"
        )


def idle_monitor(args: argparse.Namespace) -> None:
    client = MocapClient(args.port, args.baudrate, args.serial_timeout)
    client.on_heartbeat = print_heartbeat
    try:
        print(f"Opening {args.port} ...")
        client.open()
        heartbeat = client.wait_for_heartbeat(args.heartbeat_timeout)
        print("STM32 detected.")
        print_heartbeat(heartbeat)
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        client.close()


def self_test() -> None:
    values = tuple(float(i) + 0.25 for i in range(60))
    data_packet = encode_packet(
        message_type=MSG_DATA,
        flags=FLAG_STREAM | FLAG_TIMESTAMP,
        unit_count=6,
        sequence=42,
        timestamp_us=1_234_567,
        meta0=VALID_ALL,
        meta1=0,
        payload=struct.pack(DATA_FORMAT, *values),
    )
    assert len(data_packet) == 268

    parser = MocapStreamParser()
    assert parser.feed(b"\x99\x88" + data_packet[:19]) == []
    packets = parser.feed(data_packet[19:])
    assert len(packets) == 1
    packet = packets[0]
    assert isinstance(packet, MocapDataPacket)
    assert packet.left_thigh.roll == 0.25
    assert packet.left_shank.roll == 9.25
    assert packet.right_thigh.roll == 18.25
    assert packet.right_shank.roll == 27.25
    assert packet.left_foot.copx == 36.25
    assert packet.left_foot.grf == 38.25
    assert packet.right_foot.copx == 48.25
    assert packet.right_foot.grf == 50.25

    command = encode_command(7, CMD_START_STREAM)
    assert len(command) == 28

    heartbeat = encode_packet(
        message_type=MSG_HEARTBEAT,
        flags=FLAG_TIMESTAMP,
        unit_count=6,
        sequence=3,
        timestamp_us=1_000_000,
        meta0=STATE_IDLE,
        meta1=VALID_ALL,
        payload=struct.pack(HEARTBEAT_FORMAT, 1000, 50, 2, 1, 0),
    )
    assert len(heartbeat) == 48

    print("Self-test passed.")
    print(f"Header size: {HEADER_SIZE} bytes")
    print(f"Data payload size: {DATA_PAYLOAD_SIZE} bytes")
    print(f"Data packet size: {len(data_packet)} bytes")
    print(f"Heartbeat packet size: {len(heartbeat)} bytes")
    print(f"Command packet size: {len(command)} bytes")
    print(f"Response packet size: {HEADER_SIZE + RESPONSE_PAYLOAD_SIZE + CRC_SIZE} bytes")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="STM32 mocap USB CDC protocol V3 client"
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--serial-timeout", type=float, default=0.05)
    parser.add_argument("--heartbeat-timeout", type=float, default=3.0)

    sub = parser.add_subparsers(dest="mode", required=True)

    run = sub.add_parser("run", help="start, acquire and stop")
    run.add_argument("--duration", type=float, default=0.0,
                     help="seconds; 0 means until Ctrl+C")
    run.add_argument("--csv", type=Path, default=None)
    run.add_argument("--print-every", type=int, default=20)
    run.add_argument("--heartbeat-print-period", type=float, default=5.0)
    run.set_defaults(func=run_session)

    idle = sub.add_parser("idle", help="only monitor heartbeats")
    idle.set_defaults(func=idle_monitor)

    test = sub.add_parser("self-test", help="protocol test without hardware")
    test.set_defaults(func=lambda args: self_test())
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
