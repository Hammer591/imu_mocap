#!/usr/bin/env python3
"""Parse the four-IMU STM32 USB CDC packet stream on Jetson.

Protocol summary (little-endian):
    0      2   SOF = AA 55
    2      1   protocol version = 1
    3      1   message type = 0x01
    4      1   flags
    5      1   IMU count = 4
    6      2   payload length = 144
    8      4   sequence
    12     8   STM32 timestamp_us
    20     2   valid_mask
    22     2   reserved
    24   144   36 x float32
    168    4   CRC32, little-endian

CRC32 is CRC-32/ISO-HDLC and covers bytes [2:168], namely version through
last payload byte. It excludes the two SOF bytes and the CRC field itself.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import struct
import sys
import time
from pathlib import Path
from typing import BinaryIO, Iterable, Optional

try:
    import serial
    from serial import SerialException
except ImportError:  # Allows --self-test without pyserial installed.
    serial = None

    class SerialException(Exception):
        pass


SOF = b"\xAA\x55"
PROTOCOL_VERSION = 0x01
MESSAGE_TYPE_IMU4 = 0x01
IMU_COUNT = 4
FLOATS_PER_IMU = 9
FLOAT_COUNT = IMU_COUNT * FLOATS_PER_IMU
PAYLOAD_SIZE = FLOAT_COUNT * 4
PACKET_SIZE = 172
CRC_START = 2
CRC_END = 168  # Python slice end; bytes 2..167 are included.

FLAG_STREAM_DATA = 1 << 0
FLAG_TIMESTAMP_VALID = 1 << 1
FLAG_SENSOR_WARNING = 1 << 2

# Header includes SOF and ends immediately before float payload.
HEADER_FORMAT = "<2sBBBBHIQHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
FLOAT_FORMAT = f"<{FLOAT_COUNT}f"
CRC_FORMAT = "<I"

if HEADER_SIZE != 24:
    raise RuntimeError(f"Unexpected header size: {HEADER_SIZE}")

FIELD_NAMES = ("ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz")
CSV_FLOAT_COLUMNS = [
    f"imu{imu_index}_{field_name}"
    for imu_index in range(IMU_COUNT)
    for field_name in FIELD_NAMES
]


class PacketError(ValueError):
    """Raised when one candidate packet fails validation."""


@dataclasses.dataclass(frozen=True)
class ImuPacket:
    version: int
    message_type: int
    flags: int
    imu_count: int
    payload_length: int
    sequence: int
    timestamp_us: int
    valid_mask: int
    reserved: int
    imu_data: tuple[float, ...]
    received_crc32: int

    def imu(self, imu_index: int) -> tuple[float, ...]:
        """Return one IMU as (ax, ay, az, gx, gy, gz, mx, my, mz)."""
        if not 0 <= imu_index < IMU_COUNT:
            raise IndexError(f"IMU index must be 0..{IMU_COUNT - 1}")
        base = imu_index * FLOATS_PER_IMU
        return self.imu_data[base : base + FLOATS_PER_IMU]

    def imu_is_valid(self, imu_index: int) -> bool:
        if not 0 <= imu_index < IMU_COUNT:
            raise IndexError(f"IMU index must be 0..{IMU_COUNT - 1}")
        return bool(self.valid_mask & (1 << imu_index))


@dataclasses.dataclass
class ParserStatistics:
    packets_ok: int = 0
    crc_errors: int = 0
    header_errors: int = 0
    discarded_bytes: int = 0
    sequence_gaps: int = 0
    estimated_lost_packets: int = 0


def crc32_iso_hdlc(data: bytes | bytearray | memoryview) -> int:
    """Match mocap_crc32() in the STM32 C implementation exactly."""
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def parse_packet(packet: bytes) -> ImuPacket:
    """Validate and decode exactly one 172-byte packet."""
    if len(packet) != PACKET_SIZE:
        raise PacketError(f"bad packet length: {len(packet)} != {PACKET_SIZE}")

    (
        sof,
        version,
        message_type,
        flags,
        imu_count,
        payload_length,
        sequence,
        timestamp_us,
        valid_mask,
        reserved,
    ) = struct.unpack_from(HEADER_FORMAT, packet, 0)

    if sof != SOF:
        raise PacketError(f"bad SOF: {sof.hex(' ')}")
    if version != PROTOCOL_VERSION:
        raise PacketError(f"bad protocol version: {version}")
    if message_type != MESSAGE_TYPE_IMU4:
        raise PacketError(f"bad message type: 0x{message_type:02X}")
    if imu_count != IMU_COUNT:
        raise PacketError(f"bad IMU count: {imu_count}")
    if payload_length != PAYLOAD_SIZE:
        raise PacketError(f"bad payload length: {payload_length}")

    received_crc32 = struct.unpack_from(CRC_FORMAT, packet, CRC_END)[0]
    calculated_crc32 = crc32_iso_hdlc(packet[CRC_START:CRC_END])
    if received_crc32 != calculated_crc32:
        raise PacketError(
            "bad CRC32: "
            f"received=0x{received_crc32:08X}, "
            f"calculated=0x{calculated_crc32:08X}"
        )

    imu_data = struct.unpack_from(FLOAT_FORMAT, packet, HEADER_SIZE)

    return ImuPacket(
        version=version,
        message_type=message_type,
        flags=flags,
        imu_count=imu_count,
        payload_length=payload_length,
        sequence=sequence,
        timestamp_us=timestamp_us,
        valid_mask=valid_mask,
        reserved=reserved,
        imu_data=imu_data,
        received_crc32=received_crc32,
    )


class MocapStreamParser:
    """Recover complete packets from an arbitrary USB byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats = ParserStatistics()
        self._last_sequence: Optional[int] = None

    def feed(self, data: bytes) -> list[ImuPacket]:
        if data:
            self.buffer.extend(data)

        decoded: list[ImuPacket] = []

        while True:
            sof_index = self.buffer.find(SOF)

            if sof_index < 0:
                # Keep one trailing AA, because it may be the first SOF byte.
                keep = 1 if self.buffer.endswith(SOF[:1]) else 0
                discard_count = len(self.buffer) - keep
                if discard_count > 0:
                    del self.buffer[:discard_count]
                    self.stats.discarded_bytes += discard_count
                break

            if sof_index > 0:
                del self.buffer[:sof_index]
                self.stats.discarded_bytes += sof_index

            if len(self.buffer) < PACKET_SIZE:
                break

            candidate = bytes(self.buffer[:PACKET_SIZE])

            try:
                packet = parse_packet(candidate)
            except PacketError as exc:
                if "CRC32" in str(exc):
                    self.stats.crc_errors += 1
                else:
                    self.stats.header_errors += 1

                # Discard only the first candidate byte, then search SOF again.
                del self.buffer[0]
                self.stats.discarded_bytes += 1
                continue

            del self.buffer[:PACKET_SIZE]
            self.stats.packets_ok += 1
            self._update_sequence_stats(packet.sequence)
            decoded.append(packet)

        return decoded

    def _update_sequence_stats(self, sequence: int) -> None:
        if self._last_sequence is not None:
            expected = (self._last_sequence + 1) & 0xFFFFFFFF
            if sequence != expected:
                lost = (sequence - expected) & 0xFFFFFFFF
                self.stats.sequence_gaps += 1
                # Only count plausible forward jumps. A reset produces a huge wrap value.
                if lost < 0x80000000:
                    self.stats.estimated_lost_packets += lost
        self._last_sequence = sequence


def packet_to_csv_row(packet: ImuPacket, host_time_ns: int) -> list[object]:
    return [
        host_time_ns,
        packet.sequence,
        packet.timestamp_us,
        packet.flags,
        packet.valid_mask,
        *packet.imu_data,
    ]


def print_packet(packet: ImuPacket) -> None:
    timestamp_valid = bool(packet.flags & FLAG_TIMESTAMP_VALID)
    warning = bool(packet.flags & FLAG_SENSOR_WARNING)
    valid_bits = "".join(
        "1" if packet.imu_is_valid(i) else "0" for i in reversed(range(IMU_COUNT))
    )

    print(
        f"seq={packet.sequence:10d}  "
        f"stm32_time={packet.timestamp_us:15d} us  "
        f"valid=0b{valid_bits}  "
        f"flags=0x{packet.flags:02X}  "
        f"timestamp_valid={timestamp_valid}  warning={warning}"
    )

    for imu_index in range(IMU_COUNT):
        ax, ay, az, gx, gy, gz, mx, my, mz = packet.imu(imu_index)
        print(
            f"  IMU{imu_index} valid={packet.imu_is_valid(imu_index)}  "
            f"acc=({ax:+.4f}, {ay:+.4f}, {az:+.4f})  "
            f"gyro=({gx:+.4f}, {gy:+.4f}, {gz:+.4f})  "
            f"mag=({mx:+.4f}, {my:+.4f}, {mz:+.4f})"
        )


def open_csv(path: Optional[Path]) -> tuple[Optional[BinaryIO], Optional[csv.writer]]:
    if path is None:
        return None, None

    # Kept as a separate handle so it can be closed reliably in finally.
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    writer.writerow(
        [
            "host_time_ns",
            "sequence",
            "stm32_timestamp_us",
            "flags",
            "valid_mask",
            *CSV_FLOAT_COLUMNS,
        ]
    )
    handle.flush()
    return handle, writer


def receive_serial(
    port: str,
    baudrate: int,
    timeout: float,
    csv_path: Optional[Path],
    print_every: int,
    reconnect_delay: float,
) -> None:
    if serial is None:
        raise RuntimeError("pyserial is not installed: python3 -m pip install pyserial")

    parser = MocapStreamParser()
    csv_handle = None
    csv_writer = None
    rows_since_flush = 0

    try:
        csv_handle, csv_writer = open_csv(csv_path)

        while True:
            try:
                print(f"Opening {port} ...", flush=True)
                with serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=timeout,
                    write_timeout=1.0,
                    rtscts=False,
                    dsrdtr=False,
                ) as ser:
                    ser.reset_input_buffer()
                    print(f"Connected: {port}", flush=True)

                    while True:
                        chunk = ser.read(max(ser.in_waiting, 1))
                        if not chunk:
                            continue

                        host_time_ns = time.monotonic_ns()
                        packets = parser.feed(chunk)

                        for packet in packets:
                            if print_every > 0 and parser.stats.packets_ok % print_every == 0:
                                print_packet(packet)

                            if csv_writer is not None:
                                csv_writer.writerow(packet_to_csv_row(packet, host_time_ns))
                                rows_since_flush += 1
                                if rows_since_flush >= 100:
                                    assert csv_handle is not None
                                    csv_handle.flush()
                                    rows_since_flush = 0

            except (SerialException, OSError) as exc:
                print(f"Serial disconnected/error: {exc}", file=sys.stderr, flush=True)
                print(f"Retrying in {reconnect_delay:.1f} s ...", flush=True)
                time.sleep(reconnect_delay)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if csv_handle is not None:
            csv_handle.flush()
            csv_handle.close()

        stats = parser.stats
        print(
            "Parser statistics: "
            f"ok={stats.packets_ok}, "
            f"crc_errors={stats.crc_errors}, "
            f"header_errors={stats.header_errors}, "
            f"discarded_bytes={stats.discarded_bytes}, "
            f"sequence_gaps={stats.sequence_gaps}, "
            f"estimated_lost={stats.estimated_lost_packets}"
        )


def build_test_packet(sequence: int = 123, timestamp_us: int = 456789) -> bytes:
    imu_values = tuple(float(i) + 0.25 for i in range(FLOAT_COUNT))
    flags = FLAG_STREAM_DATA | FLAG_TIMESTAMP_VALID
    valid_mask = 0x000F

    packet_without_crc = struct.pack(
        HEADER_FORMAT,
        SOF,
        PROTOCOL_VERSION,
        MESSAGE_TYPE_IMU4,
        flags,
        IMU_COUNT,
        PAYLOAD_SIZE,
        sequence,
        timestamp_us,
        valid_mask,
        0,
    ) + struct.pack(FLOAT_FORMAT, *imu_values)

    if len(packet_without_crc) != CRC_END:
        raise RuntimeError("Internal test packet layout error")

    crc = crc32_iso_hdlc(packet_without_crc[CRC_START:CRC_END])
    return packet_without_crc + struct.pack(CRC_FORMAT, crc)


def self_test() -> None:
    packet_bytes = build_test_packet()
    assert len(packet_bytes) == PACKET_SIZE

    packet = parse_packet(packet_bytes)
    assert packet.sequence == 123
    assert packet.timestamp_us == 456789
    assert packet.valid_mask == 0x000F
    assert packet.imu_data[0] == 0.25
    assert packet.imu_data[-1] == 35.25

    # Verify half-packet, sticky packet, and leading garbage recovery.
    parser = MocapStreamParser()
    assert parser.feed(b"garbage" + packet_bytes[:50]) == []
    decoded = parser.feed(packet_bytes[50:] + packet_bytes)
    assert len(decoded) == 2
    assert decoded[0].sequence == 123
    assert decoded[1].sequence == 123

    damaged = bytearray(packet_bytes)
    damaged[30] ^= 0x01
    try:
        parse_packet(bytes(damaged))
    except PacketError as exc:
        assert "CRC32" in str(exc)
    else:
        raise AssertionError("CRC corruption was not detected")

    print("Self-test passed.")
    print(f"Packet size: {PACKET_SIZE} bytes")
    print(f"CRC32: 0x{packet.received_crc32:08X}")
    print_packet(packet)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read and parse STM32 four-IMU USB CDC packets."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="CDC device path")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=921600,
        help="Logical CDC baud rate; USB transfer speed is not set by this value",
    )
    parser.add_argument("--timeout", type=float, default=0.05, help="Serial read timeout")
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Optional CSV output path, for example mocap.csv",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=20,
        help="Print every N valid packets; use 0 to disable printing",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=1.0,
        help="Seconds before reopening a disconnected CDC device",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run protocol/parser self-test without opening a serial device",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.self_test:
        self_test()
        return

    if args.print_every < 0:
        raise SystemExit("--print-every must be >= 0")

    receive_serial(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        csv_path=args.csv,
        print_every=args.print_every,
        reconnect_delay=args.reconnect_delay,
    )


if __name__ == "__main__":
    main()
