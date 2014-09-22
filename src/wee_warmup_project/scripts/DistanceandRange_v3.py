#!/usr/bin/env python
# Software License Agreement (BSD License)

# CompRobo Warmup Project 
# Adela Wee
# Fall 2014

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


#initialize variables
#NW being the top left or Northwest corner, SW is bottom right corner 
mean_distance_NW = -1.0
mean_distance_SW = -1.0
mean_distance_F  = -1.0

def scan_received(msg, pub):
	#Processes data from the laser scanner
	#msg is of type sensor_msgs/LaserScan
	global mean_distance_NW
	global mean_distance_SW
	global mean_distance_F

	#make an empty list
	valid_ranges_NW = [] 
	valid_ranges_SW = []
	valid_ranges_F  = []

	# Taking IR measurements from 43-47 degrees (NW corner)
	for i in range(43,47):
		#making sure measurements are within bounds
		#if they are, add them to the NW valid ranges list
		if msg.ranges[i] > 0 and msg.ranges[i] < 8:
			valid_ranges_NW.append(msg.ranges[i]) 
			
	if len(valid_ranges_NW) > 0:
		mean_distance_NW = sum(valid_ranges_NW)/float(len(valid_ranges_NW))

	else:
		mean_distance_NW  = -1.0

	# Taking IR measurements from 133-137 degrees (SW corner)
	for i in range(133,137):
		#making sure measurements are within bounds
		#if they are, add them to the SW valid ranges list
		if msg.ranges[i] > 0 and msg.ranges[i] < 8:
			valid_ranges_SW.append(msg.ranges[i]) 
	if len(valid_ranges_SW) > 0:
		mean_distance_SW = sum(valid_ranges_SW)/float(len(valid_ranges_SW))
	else:
		mean_distance_SW = -1.0

	# Taking IR measurements from 350 to 10 deg (front) 
	for i in range(5) + range(354,360):
		#making sure measurements are within bounds
		#if they are, add them to the F valid ranges list
		if msg.ranges[i] > 0 and msg.ranges[i] < 8:
			valid_ranges_F.append(msg.ranges[i]) 
	if len(valid_ranges_F) > 0:
		mean_distance_F = sum(valid_ranges_F)/float(len(valid_ranges_F))
		print mean_distance_F
	else:
		mean_distance_F = -1.0


def wall_follow():
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
	rospy.init_node('teleop', anonymous=True)
	r = rospy.Rate(10) #run at 10 hz

	#if time, add in a dist_buffer = 
	while not rospy.is_shutdown():
		
		#move robot! this is kind of a state machine
		if mean_distance_NW != -1.0 and mean_distance_SW != -1.0 and mean_distance_F != -1.0:
			if mean_distance_NW - mean_distance_SW >= 0.3:
				# In this case, the SW corner is closer to the wall, robot is angled to the right.
				# 1st vector is linear velocity, 2nd vector is rotational velocity
				# so we'll have the robot moving forward and turning slightly right (CW)
				#positive z is rotation ccw, neg is cw
				velocity_msg = Twist(Vector3((0.2*mean_distance_F), 0.0, 0.0), Vector3(0.0, 0.0, -0.5*mean_distance_SW))
				print ("going left")	

			elif mean_distance_NW - mean_distance_SW <= -0.3:
				# In this case, the NE corner is closer to the wall, robot is angled to the left.
				velocity_msg = Twist(Vector3((0.2*mean_distance_F), 0.0, 0.0), Vector3(0.0, 0.0, 0.5*mean_distance_SW))
				print (mean_distance_NW - mean_distance_SW)
				print ("going right")

			elif abs(mean_distance_SW-mean_distance_NW) < 0.3:
				#in this case, both are equal to each other
				velocity_msg = Twist(Vector3((0.2*mean_distance_F), 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
				print ("go forward")

			elif mean_distance_F < 0.6:
				#in this case, robot is going to turn and avoid wall
				velocity_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.5))
				print ("turn right to avoid wall")

		else:
			#move forwards at 0.2 m/s
			velocity_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
			print ("move forwards")

		pub.publish(velocity_msg)
		r.sleep()

if __name__ == '__main__':
	try:
		wall_follow()
	except rospy.ROSInterruptException: pass
