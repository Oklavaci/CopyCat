import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685

from _thread import *


"""
BİSMİLLAH

Thread will be tried.
"""


def stop():
	r1.value=False
	r2.value=False
	l1.value=False
	l2.value=False

def right():
	pca.channels[0].duty_cycle = 0x1fff

	pca.channels[4].duty_cycle = 0x4fff
	pca.channels[5].duty_cycle = 0x8fff

def left():
	pca.channels[0].duty_cycle = 0x4fff

	pca.channels[4].duty_cycle = 0x8fff
	pca.channels[5].duty_cycle = 0x4fff

def speed_up():
	pca.channels[0].duty_cycle = 0xffff

	pca.channels[4].duty_cycle = 0x8fff
	pca.channels[5].duty_cycle = 0x8fff

# speed down behaves like backward
def speed_down():
	pca.channels[0].duty_cycle = 0x0000

	pca.channels[4].duty_cycle = 0x2fff
	pca.channels[5].duty_cycle = 0x2fff


# area from coordinates
def PolyArea(x,y):
	return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

def drive():
	global area, edge, right_thr, left_thr, upper_area, lower_area, id, threadStop 
	while not threadStop:
		if id is not None:
			if edge > right_thr:
				print("turn right")
				right()
				time.sleep(0.015)
				cv2.putText(frame, "turn right", (420,460), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2, cv2.LINE_AA)
			
            elif edge < left_thr:
				print("turn left")
				left()
				time.sleep(0.015)
				cv2.putText(frame, "turn left", (420,460), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2, cv2.LINE_AA)
            
            elif area > upper_area:
                print("speed down")
                cv2.putText(frame, "speed down", (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2, cv2.LINE_AA)
                speed_down()
                time.sleep(0.025)

            elif area < lower_area:
                print("speed up")
                cv2.putText(frame, "speed up", (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2, cv2.LINE_AA)
                speed_up()
                time.sleep(0.025)
        else:
            stop() # stopping the motor


if __name__ == "__main__":

    os.chdir("/home/suleyman/scripts")

    # Initialize PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 490

    # Right motor BOARD = 11,12
    r1 = digitalio.DigitalInOut(board.D17)
    r2 = digitalio.DigitalInOut(board.D18)
    r1.direction = digitalio.Direction.OUTPUT
    r2.direction = digitalio.Direction.OUTPUT

    # Left motor BOARD = 15,16
    l1 = digitalio.DigitalInOut(board.D22)
    l2 = digitalio.DigitalInOut(board.D23)
    l1.direction = digitalio.Direction.OUTPUT
    l2.direction = digitalio.Direction.OUTPUT

    marker_size = 100
    global area, edge, right_thr, left_thr, upper_area, lower_area, id, threadStop
    threadStop = False # this means thread will not be terminated

    def gstreamer_pipeline(
        sensor_id=0,
        capture_width=1920,
        capture_height=1080,
        display_width=640,
        display_height=480,
        framerate=30,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc sensor-id=%d !"
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )


    print("READY")
    pca.channels[0].duty_cycle = 0x8fff  # Channel 0 for ENA
    time.sleep(1)
    print("OK")
    pca.channels[0].duty_cycle = 0x0000 

    # Motor direction control
    r1.value = False
    r2.value = True
    l1.value = True
    l2.value = False

    # calibration
    with open('cam_cal.npy', 'rb') as f:
        camera_matrix= np.load(f)
        camera_distortion = np.load(f)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    cv2.namedWindow('Image Feed')
    cv2.moveWindow('Image Feed',159,-25)

    start_new_thread(drive,()) # thread to drive the motor

    while True:
        
        ret, frame = cap.read()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, id, rejected = aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)

        if id is not None:

            aruco.drawDetectedMarkers(frame, corners)

            rvec_all, tvec_all, _objPts = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec = rvec_all[0][0]
            tvec = tvec_all[0][0]

            corners = corners[0][0]

            x = corners[:,0]
            y = corners[:,1]

            edge = x[0]

            area = PolyArea(x,y)

            # area thresholds
            upper_area = 50000
            lower_area = 35000

            # left/right thresholds
            left_thr = 180
            right_thr = 320

            # print(area)

            cv2.drawFrameAxes(frame,camera_matrix, camera_distortion, rvec, tvec, 100)
                # tvec_str = "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                # cv2.putText(frame, tvec_str, (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2, cv2.LINE_AA)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            threadStop = True # terminating the thread
            break

    cap.release()
    cv2.destroyAllWindows()