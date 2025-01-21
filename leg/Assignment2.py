# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import cv2
import numpy as np
from ikpy.chain import Chain
from ikpy.utils import plot
import matplotlib.pyplot as plt
from leg_controller import Leg
from simple_pid import PID
import threading

def servo(ik):
    dxl.set_angle(ik, radian=True)
    print("finish")
	
def map( x,  in_min,  in_max,  out_min,  out_max) :
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (0, 128, 31)
greenUpper = (21, 255, 255)
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=2).start()
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture("/dev/video3")
# allow the camera or video file to warm up
time.sleep(2.0)
setpoint = 450//2
# keep looping
# port = 'COM3'
port = '/dev/ttyUSB0'
dxl = Leg(port=port)
np.set_printoptions(precision=3, suppress=True)
chain = Chain.from_urdf_file("Leg.urdf", 
                             active_links_mask=[False, True, True, True, True, True])
dxl = Leg(port=port)
target = [0.0, 0.0, 0.131]
orientation = [0, 0.0, 0]
frame_target = np.eye(4)
frame_target[:3, 3] = target
joints = [0] * len(chain.links)
ik = chain.inverse_kinematics(target, initial_position=joints, target_orientation=orientation, orientation_mode="all")
Kp = 1
Kd = 0.01
offset = -8
e_prev = 0

fourcc = cv2.VideoWriter_fourcc(*"XVID")  # You can choose the codec, e.g., MJPG, XVID, etc.
output_path = "Assignment2.avi"  # Set the path where you want to save the output video
fps = 30  # Set the frames per second
out = cv2.VideoWriter(output_path, fourcc, fps, (320, 240))


# pid = PID(1, 0, 0.001, setpoint=setpoint)
while True:
	# grab the current frame
	frame = vs.read()
	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame
	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break
	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=320,height=240)
	# print(frame.shape)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	
    	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		# print(x,y)
		
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			
		# center_y_normalized = y/240
		e =  (frame.shape[0]//2 + offset) - y
		P =  Kp * e
		D =  Kd * (e - e_prev)
		control = P + D
		print("Outpid before map : {}".format(control))
		control =  map(control,-60,60,-0.008,0.008)
		target = [0.0, control, 0.131]
		print("Error : {} , Output PID : {}, target : {}, setpoint : {}, y : {} ".format(e, control, target,frame.shape[0]//2,y))
		ik = chain.inverse_kinematics(target, initial_position=joints, target_orientation=orientation, orientation_mode="all")
		ik = np.delete(ik, 0)
		dxl.set_angle(ik, radian=True)
		e_prev = e
	# update the points queue
	pts.appendleft(center)
	
    # loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue
		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		# cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
	# show the frame to our screen
	cv2.imshow("Frame", frame)
	out.write(frame)
	key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()
# otherwise, release the camera
else:
	vs.release()
# close all windows
cv2.destroyAllWindows()