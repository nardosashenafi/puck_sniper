#!/usr/bin/env python

"""
USAGE
rosrun object_track object_track_no_bridge -s 1 -b 32
python object_track_no_bridge.py --source 0 (default) --buffer 32 (default)

"""

# import the necessary packages
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
import argparse
import time
import os
from collections import deque
from geometry_msgs.msg import Pose

# initilize camera settings
CAP_PROP_POS_MSEC = 0
CAP_PROP_POS_FRAMS = 1
CAP_PROP_POS_AVI_RATIO = 2
CAP_PROP_FRAME_WIDTH = 3
CAP_PROP_FRAME_HEIGHT = 4
CAP_PROP_FPS = 5
CAP_PROP_FRAME_COUNT = 7
CAP_PROP_FORMAT = 8
CAP_PROP_MODE = 9
CAP_PROP_BRIGHTNESS = 10
CAP_PROP_CONTRAST = 11
CAP_PROP_HUE = 13
CAP_PROP_GAIN = 14
CAP_PROP_EXPOSURE = 15
CAP_PROP_SHARPNESS = 20
CAP_PROP_AUTO_EXPOSURE = 21
CAP_PROP_TEMPERATURE = 23
CAP_PROP_ZOOM = 27
CAP_PROP_FOCUS = 28
CAP_PROP_AUTOFOCUS = 39

desired_u = 0
desired_v = 0
boundary_pt1 = (0,0)
boundary_pt2 = (0,0)

# initialize storage arrays
camera_matrix = []

# read camera matrix and translation vector text file
file_path_cam = "/home/nardos/Pictures/object_track_snaps/cameraMatrix.txt"

# proceed if both files exists
if os.path.exists(file_path_cam):

	# read camera matrix text file and store into array
	with open(file_path_cam) as cam_mtx_file:
		# read line into array
		for cam_line in cam_mtx_file.readlines():
			# loop over elements and split by comma
			for i in cam_line.split(','):
				# append list
				camera_matrix.append(i)

else:
	print("Files not found")
	sys.exit()


# method that does nothing, used for making trackbar
def nothing(x):
	pass

def convert_to_world_frame(u,v):

	X = u*1000
	Y = v*1000

	# bnd_PT1 = np.array([-500.0, boundary[1]*1000, 0.0, 1]).reshape((4,1))
	# bnd_PT2 = np.array([500.0, boundary[1]*1000, 0.0, 1]).reshape((4,1))
	# ellipse_center_vec = np.array([0.0, -100.0, 0.0, 1]).reshape((4,1))
	# ellipse_axes_vec = np.array([250.0, 330.0]).reshape((2,1))

	t_1 = 0.0 
	t_2 = 0.0
	t_3 = 100.0

	Xi = np.array([X, Y, 0.0, 1]).reshape((4,1))

	K = np.array([[float(camera_matrix[0]),0.0,float(camera_matrix[2])],
				  [0.0,float(camera_matrix[4]),float(camera_matrix[5])],
				  [0.0,0.0,1.0]])


	Pi = np.array([[1.0,0,0,0],
	               [0, 1, 0, 0],
				   [0, 0, 1, 0]])
	
	g = np.array([[1, 0, t_1],
	             [0, 1, t_2],
				#  [0, 0, -1, t_3],
				 [0, 0, 0, 1]])

	xi = np.linalg.inv(np.dot(K,g)) * t_3*Xi

	# bndr_pt1 = 1.0/t_3*np.linalg.multi_dot([K,Pi,g,bnd_PT1])
	# bndr_pt2 = 1.0/t_3*np.linalg.multi_dot([K,Pi,g,bnd_PT2])
	# converted_ellipse_center_vec = 1.0/t_3*np.linalg.multi_dot([K,Pi,g,ellipse_center_vec])
	# converted_ellipse_axes_vec = 1.0/t_3*np.linalg.multi_dot([K[0:2,0:2], g[0:2,0:2], ellipse_axes_vec])

	desired_u = int(np.floor(xi[0]))
	desired_v = int(np.floor(xi[1]))
	# boundary_pt1 = (int(np.floor(bndr_pt1[0])), int(np.floor(bndr_pt1[1])))
	# boundary_pt2 = (int(np.floor(bndr_pt2[0])), int(np.floor(bndr_pt2[1])))

	# ellipse_center = (int(np.floor(converted_ellipse_center_vec[0])), int(np.floor(converted_ellipse_center_vec[1])))
	# ellipse_axes = (int(np.floor(converted_ellipse_axes_vec[0])), int(np.floor(converted_ellipse_axes_vec[1])))

	return desired_u, desired_v


def main(args):
	
	# record video functionaility
	out_file_name = time.strftime("object_track.avi", time.localtime())
	out_file_path = "/home/nardos/Videos/" + out_file_name
	record = False
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = cv2.VideoWriter(out_file_path, fourcc, 29.97, (640, 480))

	# create ROS node and publish to a topic
	rospy.init_node('Object_Tracking', anonymous=False)
	pose_pub = rospy.Publisher("puck_xy_position", Pose, queue_size=10)
	# vis_sub = rospy.Subscriber("motor_angles", MotorCommands, vis_callback)
	pose = Pose()

	# construct the argument parse and parse the arguments. Set buffer size if needed
	ap = argparse.ArgumentParser()
	ap.add_argument("-s", "--source", type=int, default=0, help="webcam source")
	ap.add_argument("-b", "--buffer", type=int, default=32, help="max buffer size")
	args = vars(ap.parse_args())

	# initialize the list of tracked points, the frame counter, and the coordinate deltas
	pts = deque(maxlen=args["buffer"])
	counter = 0
	(u, v)  = (0, 0)
	(du, dv) = (0, 0)

	# grab/start the reference to the webcam. Take source from args input.
	vs = cv2.VideoCapture(args["source"])
	# set camera settings, frame height/width
	vs.set(CAP_PROP_FRAME_HEIGHT, 480)
	vs.set(CAP_PROP_FRAME_WIDTH, 640)
	vs.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
	vs.set(cv2.CAP_PROP_EXPOSURE, 0.07)
		
	# allow the camera to warm up
	time.sleep(2.0)

	# create window Frame and HSV
	cv2.namedWindow("Frame")
	# cv2.namedWindow("HSV")

	# store lower and upper hsv values
	lower_hsv = np.array([21, 0, 0])
	upper_hsv = np.array([73, 255, 255])
	
	# set constrast to value of 4
	# cv2.setTrackbarPos("Contrast", "Frame", 7)
	vs.set(CAP_PROP_CONTRAST, 7)

	# keep looping if video capture is open
	while vs.isOpened():

		# grab the current frame
		ret, frame = vs.read()

		if ret is not True:
			print("ret is nothing")
			break
		else:

			# blur frame, and convert it to the HSV color space
			blurred = cv2.GaussianBlur(frame, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

			# construct a mask for the color "green", then perform a series of
			# of dilations and erosions to remove any small blobs left in the mask
			mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
			kernel = np.ones((5,5), np.uint8)
			mask = cv2.erode(mask, kernel, iterations=2)
			mask = cv2.dilate(mask, kernel, iterations=2)

			# find contours in the mask and initialize the current (x, y) center of the ball
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cnts = cnts[0]
			center = None

			# only proceed if at least one contour was found
			# if(cnts is not None):
			if len(cnts) > 0:
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and centroid
				c = max(cnts, key=cv2.contourArea)
				((u, v), radius) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				pose.orientation.w = 1.0		# Is puck in the FOV? If yes, w = 1, else w = 0
				
				# only proceed if the radius meets a minimum size
				if radius > 3:
					# draw the circle and centroid on the frame, then update the list of tracked points
					cv2.circle(frame, (int(u), int(v)), int(radius), (0, 255, 255), 2)
					cv2.circle(frame, center, 5, (0, 0, 255), -1)
					pts.appendleft(center)
			else: 
				pose.orientation.w = 0.0		# Is puck in the FOV? If yes, w = 1, else w = 0

			# show the u and v position in the image plane (2D) of the center of circle on frame
			cv2.putText(frame, "u: {}, v: {}".format(u, v), (10, frame.shape[0] - 10),
					cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

			# use Pose msg class to define an u,v position that can be published easily
			# w_u, w_v = convert_to_world_frame(u,v)
			pose.position.x = u
			pose.position.y = v
			
			# publish the u and v position on the topic
			pose_pub.publish(pose)

			if record is True and out.isOpened():
				cv2.putText(frame, "Recording", (frame.shape[1] - 60, 10),
					cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
				out.write(frame)

			cv2.imshow("Frame", frame)
			key = cv2.waitKey(1) & 0xFF
			counter += 1
			
			# if the 'q' key is pressed, stop the loop
			if key == ord("q"):
				break

			if key == ord("r"):
				record = not record

	# stop the webcam feed
	out.release()
	vs.release()

	# Delete video file if nothing was recorded
	vid_file_info = os.stat(out_file_path)
	if vid_file_info.st_size < 6000:
		os.remove(out_file_path)
	
	# close all windows shut down node
	cv2.destroyAllWindows()
	time.sleep(2.0)
	rospy.signal_shutdown("Tracking Finished.")

# if the special variable __name__ is "main" then execute the code.
if __name__ == '__main__':

	sys.argv = rospy.myargv(argv=sys.argv)
	main(sys.argv)
