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
#NE being northeast (top right) corner, SE is bottom right corner
mean_distance_NE = -1.0
mean_distance_SE = -1.0
mean_distance_F  = -1.0

def scan_received(msg, pub):
	#Processes data from the laser scanner
	#msg is of type sensor_msgs/LaserScan
	global mean_distance_NE
	global mean_distance_SE
	global mean_distance_F

	#make an empty list
	valid_ranges_NE = [] 
	valid_ranges_SE = []
	valid_ranges_F  = []

	# Taking IR measurements from 43-47 degrees (NE corner)
	for i in range(43,47):
		#making sure measurements are within bounds
		#if they are, add them to the NE valid ranges list
		if msg.ranges[i] > 0 and msg.ranges[i] < 8:
			valid_ranges_NE.append(msg.ranges[i]) 
	if len(valid_ranges_NE) > 0:
		mean_distance_NE = sum(valid_ranges_NE)/float(len(valid_ranges_NE))
	else:
		mean_distance_NE  = -1.0

	# Taking IR measurements from 133-137 degrees (SE corner)
	for i in range(133,137):
		#making sure measurements are within bounds
		#if they are, add them to the SE valid ranges list
		if msg.ranges[i] > 0 and msg.ranges[i] < 8:
			valid_ranges_SE.append(msg.ranges[i]) 
	if len(valid_ranges_SE) > 0:
		mean_distance_SE = sum(valid_ranges_SE)/float(len(valid_ranges_SE))
	else:
		mean_distance_SE = -1.0

	# Taking IR measurements from 350 to 10 deg (front)
	for i in range(10) + range(350,360):
		#making sure measurements are within bounds
		#if they are, add them to the F valid ranges list
		if msg.ranges[i] > 0 and msg.ranges[i] < 8:
			valid_ranges_F.append(msg.ranges[i]) 
	if len(valid_ranges_F) > 0:
		mean_distance_F = sum(valid_ranges_F)/float(len(valid_ranges_F))
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
		if mean_distance_NE != -1.0 and mean_distance_SE != -1.0 and mean_distance_F != 1.0:
			if mean_distance_NE > mean_distance_SE:
				# In this case, the SE corner is closer to the wall, robot is angled to the left.
				# 1st vector is linear velocity, 2nd vector is rotational velocity
				# so we'll have the robot moving forward and turning slightly right
				#positive z is rotation ccw, neg is cw
				velocity_msg = Twist(Vector3((0.2*mean_distance_F - 1.0), 0.0, 0.0), Vector3(0.0, 0.0, -0.2*mean_distance_SE))

			elif mean_distance_NE < mean_distance_SE:
				# In this case, the NE corner is closer to the wall, robot is angled to the right.
				velocity_msg = Twist(Vector3((0.2*mean_distance_F - 1.0), 0.0, 0.0), Vector3(0.0, 0.0, 0.2*mean_distance_SE))

			elif mean_distance_SE == mean_distance_NE:
				#in this case, both are equal to each other
				velocity_msg = Twist(Vector3((0.2*mean_distance_F - 1.0), 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

			elif mean_distance_F < 0.2:
				#in this case, robot is going to turn and avoid wall
				velocity_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.2))

		else:
			#move forwards and towards the wall at 0.2 m/s
			velocity_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, -0.2))

		pub.publish(velocity_msg)
		r.sleep()

if __name__ == '__main__':
	try:
		wall_follow()
	except rospy.ROSInterruptException: pass
