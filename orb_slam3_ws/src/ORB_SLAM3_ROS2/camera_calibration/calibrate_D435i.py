import pyrealsense2 as rs
import numpy as np
import cv2
import glob


class CameraCalib:
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((7 * 10, 3), np.float32)
    objp[:, :2] = np.mgrid[0:10, 0:7].T.reshape(-1, 2)

    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    def __init__(self):
        # enable streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

    def calibrate(self):
        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convert color frame to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                cv2.imshow("gray", gray_image)
                cv2.waitKey(1)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray_image, (10, 7), None)

                # If found, add object points, image points (after refining them)
                if ret == True:
                    objpoints.append(objp)

                    corners2 = cv2.cornerSubPix(
                        gray_image, corners, (11, 11), (-1, -1), criteria
                    )
                    imgpoints.append(corners2)

                    # Draw and display the corners
                    cv2.drawChessboardCorners(gray_image, (10, 7), corners2, ret)
                    cv2.imshow("chessboard", gray_image)
                    cv2.waitKey(1)

                    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                        objpoints, imgpoints, gray_image.shape[::-1], None, None
                    )

                    print(f"matrix:\n {mtx}")
                    print(f"dist: {dist}\n")

        finally:
            # Stop streaming
            pipeline.stop()

            # Close the OpenCV window
            cv2.destroyAllWindows()

def main():
    if __name__ == "__main__":
        cam = CameraCalib()
        cam.calibrate()
