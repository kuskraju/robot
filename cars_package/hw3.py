import time
import numpy as np
import cv2
from cars import Motors, Connection, Direction, Camera


def convolve2D(image, kernel=np.ones((2, 2)), padding=2, strides=1):
    # Cross Correlation
    kernel = np.flipud(np.fliplr(kernel))

    # Gather Shapes of Kernel + Image + Padding
    xKernShape = kernel.shape[0]
    yKernShape = kernel.shape[1]
    xImgShape = image.shape[0]
    yImgShape = image.shape[1]

    # Shape of Output Convolution
    xOutput = int(((xImgShape - xKernShape + 2 * padding) / strides) + 1)
    yOutput = int(((yImgShape - yKernShape + 2 * padding) / strides) + 1)
    output = np.zeros((xOutput, yOutput))

    # Apply Equal Padding to All Sides
    if padding != 0:
        imagePadded = np.zeros((image.shape[0] + padding*2, image.shape[1] + padding*2))
        imagePadded[int(padding):int(-1 * padding), int(padding):int(-1 * padding)] = image
    else:
        imagePadded = image

    # Iterate through image
    for y in range(image.shape[1]):
        # Exit Convolution
        if y > image.shape[1] - yKernShape:
            break
        # Only Convolve if y has gone down by the specified Strides
        if y % strides == 0:
            for x in range(image.shape[0]):
                # Go to next row once kernel is out of bounds
                if x > image.shape[0] - xKernShape:
                    break
                try:
                    # Only Convolve if x has moved by the specified Strides
                    if x % strides == 0:
                        output[x, y] = (kernel * imagePadded[x: x + xKernShape, y: y + yKernShape]).sum()
                except:
                    break

    return output


def move_straight_test():
    cv2.namedWindow("demo")
    connection = Connection()
    cam = Camera(connection=connection)
    motors = Motors(connection=connection)

    for i in range(15):
        connection.keep_stream_alive()
        motors.command(80, Direction.FORWARD)
        img = cam.get_frame()
        #cv2.imwrite(f"move_straight/{i}.jpg", img)
        time.sleep(0.2)


def turn_around_test(blurred=False):
    cv2.namedWindow("demo")
    connection = Connection()
    cam = Camera(connection=connection)
    motors = Motors(connection=connection)

    cam.flash_on()
    time.sleep(1)
    cam.flash_off()

    i = 0
    while True:
        connection.keep_stream_alive()
        motors.command(80, Direction.LEFT)
        i += 1
        time.sleep(0.2)
        if not blurred:
            time.sleep(1)
        img = cam.get_frame()
        cv2.imshow("demo", img)
        if blurred:
            cv2.imwrite(f"turn_around_blurred/{i}.jpg", img)
        else:
            cv2.imwrite(f"turn_around/{i}.jpg", img)
        keypress = cv2.pollKey() & 0xFF
        if keypress == ord('q'):
            break


def simple_marker():
    cv2.namedWindow("demo")
    connection = Connection()
    cam = Camera(connection=connection)
    motors = Motors(connection=connection)

    camera_matrix = np.load('camera_calibration_data/cameraMatrix.npy')
    dist_coeffs = np.load('camera_calibration_data/distCoeffs.npy')
    size = (800, 600)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    detectorParams = cv2.aruco.DetectorParameters_create()
    detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

    alpha = 1
    rect_camera_matrix = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, size, alpha)[0]

    map1_r, map2_r = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, np.eye(3), rect_camera_matrix, size, cv2.CV_32FC1)

    cam.flash_on()
    time.sleep(1)
    cam.flash_off()

    while True:
        connection.keep_stream_alive()
        img = cam.get_frame()
        rect_img = cv2.remap(img, map1_r, map2_r, cv2.INTER_LINEAR)
        corners, ids, _ = cv2.aruco.detectMarkers(rect_img, dictionary, None, None, detectorParams)
        cv2.imshow("demo", rect_img)
        if ids is not None:
            if ids[0][0] == 5:
                motors.command(80, Direction.FORWARD)
            elif ids[0][0] == 6:
                motors.command(80, Direction.BACKWARD)
            elif ids[0][0] == 8:
                motors.command(80, Direction.LEFT)
            elif ids[0][0] == 3:
                motors.command(80, Direction.RIGHT)
            time.sleep(0.2)
        cv2.waitKey(1)


def clockwise_right(id):
    cv2.namedWindow("demo0")
    connection = Connection()
    cam = Camera(connection=connection)
    motors = Motors(connection=connection)

    camera_matrix = np.load('camera_calibration_data/cameraMatrix.npy')
    dist_coeffs = np.load('camera_calibration_data/distCoeffs.npy')
    size = (800, 600)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    detectorParams = cv2.aruco.DetectorParameters_create()
    detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

    alpha = 1
    rect_camera_matrix = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, size, alpha)[0]

    map1_r, map2_r = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, np.eye(3), rect_camera_matrix, size, cv2.CV_32FC1)

    while True:
        connection.keep_stream_alive()
        img = cam.get_frame()
        rect_img = cv2.remap(img, map1_r, map2_r, cv2.INTER_LINEAR)
        cv2.imshow("demo0", rect_img)
        cv2.waitKey(100)
        corners, ids, _ = cv2.aruco.detectMarkers(rect_img, dictionary, None, None, detectorParams)
        if ids is not None and [id] in ids:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.042, rect_camera_matrix, 0)
            t = tvecs[0][0]
            pita = np.sqrt(np.dot(t, t))
            cv2.aruco.drawAxis(rect_img, camera_matrix, 0, rvecs[0], tvecs[0], 0.1)
            lower_red = np.array([0, 0, 200])
            upper_red = np.array([100, 100, 255])
            mask_red = cv2.inRange(rect_img, lower_red, upper_red).astype(int)
            lower_blue = np.array([200, 0, 0])
            upper_blue = np.array([255, 100, 100])
            mask_blue = cv2.inRange(rect_img, lower_blue, upper_blue).astype(int)
            mask_blue = convolve2D(mask_blue)
            mask_red = convolve2D(mask_red)
            rb, cb = np.nonzero(mask_blue)
            rows, cols = np.nonzero(np.abs(np.abs(mask_blue - mask_red).astype(np.uint8) - (mask_red + mask_blue).astype(np.uint8)) > 0)
            if pita < 0.25:
                print("Marker {} reached.".format(id))
                return 0
            elif np.mean(cols) - np.min(cb) > 50 and np.mean(cols) < 400:
                motors.command(85, Direction.LEFT)
                time.sleep(0.2)
                for i in range(int(3 * pita)):
                    motors.command(80, Direction.FORWARD)
                    time.sleep(0.1)
                motors.command(80, Direction.FORWARD)
            elif np.mean(cols) < 200:
                motors.command(70, Direction.LEFT)
            elif np.mean(cols) > 700:
                motors.command(65, Direction.RIGHT)
            else:
                for i in range(int(3 * pita)):
                    motors.command(80, Direction.FORWARD)
                    time.sleep(0.1)
                motors.command(80, Direction.FORWARD)
            time.sleep(0.5)
        else:
            motors.command(70, Direction.RIGHT)
            time.sleep(0.5)
        connection.keep_stream_alive()
        cam.get_frame()
        time.sleep(0.1)
        cv2.imshow("demo0", rect_img)
        cv2.waitKey(100)


def clockwise_left(id):
    cv2.namedWindow("demo0")
    connection = Connection()
    cam = Camera(connection=connection)
    motors = Motors(connection=connection)

    camera_matrix = np.load('camera_calibration_data/cameraMatrix.npy')
    dist_coeffs = np.load('camera_calibration_data/distCoeffs.npy')
    size = (800, 600)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    detectorParams = cv2.aruco.DetectorParameters_create()
    detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

    alpha = 1
    rect_camera_matrix = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, size, alpha)[0]

    map1_r, map2_r = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, np.eye(3), rect_camera_matrix, size, cv2.CV_32FC1)

    while True:
        connection.keep_stream_alive()
        img = cam.get_frame()
        rect_img = cv2.remap(img, map1_r, map2_r, cv2.INTER_LINEAR)
        cv2.imshow("demo0", rect_img)
        cv2.waitKey(100)
        corners, ids, _ = cv2.aruco.detectMarkers(rect_img, dictionary, None, None, detectorParams)
        if ids is not None and [id] in ids:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.042, rect_camera_matrix, 0)
            t = tvecs[0][0]
            pita = np.sqrt(np.dot(t, t))
            cv2.aruco.drawAxis(rect_img, camera_matrix, 0, rvecs[0], tvecs[0], 0.1)
            lower_red = np.array([0, 0, 200])
            upper_red = np.array([100, 100, 255])
            mask_red = cv2.inRange(rect_img, lower_red, upper_red).astype(int)
            lower_blue = np.array([200, 0, 0])
            upper_blue = np.array([255, 100, 100])
            mask_blue = cv2.inRange(rect_img, lower_blue, upper_blue).astype(int)
            mask_blue = convolve2D(mask_blue)
            mask_red = convolve2D(mask_red)
            rb, cb = np.nonzero(mask_blue)
            rows, cols = np.nonzero(np.abs(np.abs(mask_blue - mask_red).astype(np.uint8) - (mask_red + mask_blue).astype(np.uint8)) > 0)
            if pita < 0.25:
                print("Marker {} reached.".format(id))
                return 0
            elif np.mean(cols) - np.min(cb) > 50 and np.mean(cols) > 400:
                motors.command(85, Direction.RIGHT)
                time.sleep(0.2)
                for i in range(int(3 * pita)):
                    motors.command(80, Direction.FORWARD)
                    time.sleep(0.1)
                motors.command(80, Direction.FORWARD)
            elif np.mean(cols) > 600:
                motors.command(70, Direction.RIGHT)
            elif np.mean(cols) < 100:
                motors.command(65, Direction.LEFT)
            else:
                for i in range(int(3 * pita)):
                    motors.command(80, Direction.FORWARD)
                    time.sleep(0.1)
                motors.command(80, Direction.FORWARD)
            time.sleep(0.5)
        else:
            motors.command(70, Direction.LEFT)
            time.sleep(0.7)
        connection.keep_stream_alive()
        cam.get_frame()
        time.sleep(0.1)
        cv2.imshow("demo0", rect_img)
        cv2.waitKey(100)


def clockwise(ids, direction):
    if direction == "right":
        for id in ids:
            clockwise_right(id)
    elif direction == "left":
        for id in ids:
            clockwise_left(id)


def x_go(ids):
    clockwise_left(ids[0])
    clockwise_left(ids[1])
    clockwise_right(ids[2])
    clockwise_left(ids[3])


if __name__ == "__main__":
    # move_straight_test()
    # turn_around_test(blurred=True)
    # turn_around_test(blurred=False)
    # simple_marker()
    # clockwise([3, 8, 5, 6], "right")
    # clockwise([6, 5, 8, 3], "left")
    x_go([3, 5, 6, 8])