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
MAX_ANGULAR_VEL = 1
MAX_LINEAR_VEL = 0.21
LINEAR_VEL = 0.1
STOP_DISTANCE = 0.2 * 2
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
        obstacle_right = False
        obstacle_left = False
        remaining_angle = 0
        angle_left = 0
        angle_right = 0
        object_angle_left = 0
        object_angle_right = 0
        count = 0

        while not rospy.is_shutdown():
            lidar_distances_left = []
            lidar_distances_right = []
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
                if (0 <= i <= 44):
                    lidar_distances_right.append(data[i])
                else:
                    lidar_distances_left.append(data[i])

            right_boolean = []
            left_boolean = []

            # Get the minimum values from the arrays
            for i in range(len(lidar_distances_left)):
                if (lidar_distances_right[i] <  2 * SAFE_STOP_DISTANCE):
                    right_boolean.append('True')
                else :
                    right_boolean.append('False')
                
                if (lidar_distances_left[i] < 2 * SAFE_STOP_DISTANCE):
                    left_boolean.append('True')
                else :
                    left_boolean.append('False')

            #reversed_left_boolean = left_boolean
            #reversed_left_boolean.reverse()
            reversed_right_boolean = right_boolean
            reversed_right_boolean.reverse()

            try:
                angle_right =  -45 + left_boolean.index('True')
            except:
                angle_right = 0
            
            try:
                angle_left = 45 - reversed_right_boolean.index('True')
                
            except:
                angle_left = 0  

            
            if (remaining_angle == 0):        
                if (right_boolean.count('False') == 45 and left_boolean.count('False') == 45):
                    twist.linear.x = MAX_LINEAR_VEL
                    twist.angular.z = 0
                    self._cmd_pub.publish(twist)
                    twist.angular.z = 0
                    self._cmd_pub.publish(twist)
                
                elif (right_boolean.count('False') == 45 and left_boolean.count('True') > 0):
                    #print('turn right')
                    remaining_angle = angle_right

                elif (left_boolean.count('False') == 45 and right_boolean.count('True') > 0):
                    #print('turn left')
                    remaining_angle = angle_left
                
                elif (right_boolean.count('True') > 0 and left_boolean.count('True') > 0):
                    #print('middle')
                    lowest_angle = min(angle_left, abs(angle_right))
                    if (lowest_angle == abs(angle_right)):
                        remaining_angle = angle_right
                    else :
                        remaining_angle = angle_left

            count += 1
            print(remaining_angle)
            if (remaining_angle != 0):
                if (count == 1):
                    remaining_angle /= 2
                #print(remaining_angle)
                angle_scalar = remaining_angle / (min(min(lidar_distances_left), min(lidar_distances_right)) * 100)
                if (angle_scalar > 1):
                    angle_scalar = 1
                #print(angle_scalar)
                #print(min(min(lidar_distances_right), min(lidar_distances_left)) * 100)
                if (min(min(lidar_distances_right), min(lidar_distances_left)) * 100 <= 20):
                    if (remaining_angle >= 0):
                        angle_scalar = 1
                        twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                        twist.linear.x = 0
                        self._cmd_pub.publish(twist)
                        time.sleep(0.1)
                        remaining_angle = 0
                    else:
                        angle_scalar = -1
                        twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                        twist.linear.x = 0
                        self._cmd_pub.publish(twist)
                        time.sleep(0.1)
                        remaining_angle = 0
                
                #print(angle_scalar)
                lin_scalar = 1 - abs(angle_scalar)
                if (lin_scalar < 0):
                    lin_scalar = 0

                twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                twist.linear.x = lin_scalar * MAX_LINEAR_VEL
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                twist.angular.z = 0
                self._cmd_pub.publish(twist)
                left_boolean = []
                right_boolean = []

            if (count == 3):
                remaining_angle = 0
                count = 0
    
        def turn(self, angle):
            if (angle >= 0):
                sign = 1
            else:
                sign = -1
            for i in range(2):
                if (abs(angle) > 30):
                    angle_scalar = 0.8 * sign
                elif (abs(angle) < 30 and abs(angle) > 20):
                    angle_scalar = 0.6 * sign
                elif (abs(angle) < 20 and abs(angle) > 10):
                    angle_scalar = 0.4 * sign
                elif (abs(angle) < 10 and abs(angle) > 0):
                    angle_scalar = 0.2 * sign
                elif (abs(angle) == 0):
                    angle_scalar = 0

                twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                twist.linear.x = lin_scalar * MAX_LINEAR_VEL
                self._cmd_pub.publish(twist)
                time.sleep(0.2)


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