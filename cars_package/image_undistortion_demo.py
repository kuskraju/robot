import cv2
import numpy as np

from cars import Camera, Connection, RESOLUTIONS


def main():
    cv2.namedWindow("demo")
    connection = Connection()
    cam = Camera(connection=connection)

    map1 = np.load('camera_calibration_data/map1.npy')
    map2 = np.load('camera_calibration_data/map2.npy')

    while True:
        connection.keep_stream_alive()
        img = cam.get_frame()
        undistorted_img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

        cv2.imshow("demo", np.hstack([img, undistorted_img]))
        keypress = cv2.pollKey() & 0xFF
        if keypress == ord('q'):
            break


if __name__ == "__main__":
    main()
