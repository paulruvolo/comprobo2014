#!/usr/bin/env python

import statistics  
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


class MeanDistance(): #define object of type distance
#allows you to pass packaging of distance data
#NE being northeast (top right) corner, SE is bottom right corner, F is front
#initializing distances as unrealistic values so that we know if we're getting realistic data
    self.SE = -1.0
    self.NE = -1.0
    self.F  = -1.0


class WallFollower():
    def __init__(self,distFromWall):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
        rospy.init_node('teleop', anonymous=True)
        self.distFromWall = distFromWall
        self.distance = getDistance() #this is where we get the obj Distance using getDistance
    def getDistance():
        validNE = [] #init lists
        validSE = []
        validF  = []
        meanDistance = MeanDistance() #defined type of data called MeanDistance
        #msg gets passed in from the import mgs at top
        #gets laser range distance from subscriber
        for i in range(43,360): #store raw data in lists
            if msg.ranges[i] > 0 and msg.ranges[i] < 8:
                if i in range(43,47):
                    validNE.append(msg.ranges[i]) 
                elif i in range(133,137):
                    validSE.append(msg.ranges[i])
                elif i in range(350, 360):
                    validF.append(msg.ranges[i])
        #check if data has been received
        if len(validSE) > 0:
            meanDistance.SE = statistics.mean(validSE) #calcs mean and stores into meanDistance object initialized above
        if len(validNE) > 0:
            meanDistance.NE = statistics.mean(Distance.NE)
        if len(validF) > 0:
            meanDistance.F  = statistics.mean(Distance.F)
        return meanDistance #meanDistance contains all averaged data
    

    def wallFollow(self):
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

if __name__ == '__main__':
    r = rospy.Rate(10) #run program at 10 Hz
        while not rospy.is_shutdown():
            wallfollower = WallFollower(1.0) #instatiate WallFollower with distance from wall 1.0 m
            wallfollower.wallFollow() #tell the robot to go follow the wall

        except rospy.ROSInterruptException: pass
    
