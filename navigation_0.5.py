#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

ANGULAR_VEL = 1.0
LINEAR_VEL = 0.1
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 90            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        # Error handling for data
        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif scan_filter[i] == 0:
                scan_filter[i] = 1
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        # Go forward
        turtlebot_moving = True
        remaining_angle = 0

        while not rospy.is_shutdown():
            lidar_distances_left = []
            lidar_distances_right = []
            lidar_distances_front = []
            if self.is_key_available():
                char = self.get_key()
                if char.lower() == 'q':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    return
            
            # Get the values
            data = self.get_scan()
            for i in range(len(data)):
                if (0 <= i <= 29):
                    lidar_distances_left.append(data[i])
                elif (30 <= i <= 59):
                    lidar_distances_front.append(data[i])
                else:
                    lidar_distances_right.append(data[i])

            # Get the minimum values from the arrays
            min_distance_front = min(lidar_distances_front)
            min_distance_right = min(lidar_distances_right)
            min_distance_left = min(lidar_distances_left)

            if min_distance_front < SAFE_STOP_DISTANCE:
                
                # something is in front
                if turtlebot_moving:
                    if remaining_angle == 0:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = False

                if (min_distance_right < SAFE_STOP_DISTANCE):
                
                    if (min_distance_left < SAFE_STOP_DISTANCE):
                        print("box")
                        # In a box turn 180 deg
                        twist.angular.z = ANGULAR_VEL
                        self._cmd_pub.publish(twist)
                        time.sleep(1)
                        twist.angular.z = 0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = True
                    else:  
                        print("left")
                        # Nothing to the left so turn left  
                        twist.angular.z = ANGULAR_VEL
                        self._cmd_pub.publish(twist)
                        time.sleep(0.5)
                        twist.angular.z = 0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = True
                        
                        print("front: " + str(min_distance_front))
                        print("left: " + str(min_distance_left))
                        print("right: " + str(min_distance_right))
                        
                else:
                    print("right")
                    # Nothing to the right so turn right  
                    twist.angular.z = -ANGULAR_VEL
                    self._cmd_pub.publish(twist)
                    time.sleep(0.5)
                    twist.angular.z = 0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = True

            else:
                print("front")
                # Nothing is in front of the robot, therefore keep moving
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle front: %f', min_distance_front)

    def is_key_available(self):
        # Check whether a key press is available
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def get_key(self):
        # Get the pressed key
        return sys.stdin.read(1)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
