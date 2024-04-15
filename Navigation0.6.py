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
        obstacle_front = False
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

            front_boolean = []
            right_boolean = []
            left_boolean = []
            # Get the minimum values from the arrays
            min_distance_front = min(lidar_distances_front)
            for i in range(len(lidar_distances_front)):
                if (lidar_distances_front[i] < SAFE_STOP_DISTANCE):
                    front_boolean.append('True')
                else :
                    front_boolean.append('False')
                
                if (lidar_distances_right[i] < SAFE_STOP_DISTANCE):
                    right_boolean.append('True')
                else :
                    right_boolean.append('False')
                
                if (lidar_distances_left[i] < SAFE_STOP_DISTANCE):
                    left_boolean.append('True')
                else :
                    left_boolean.append('False')


            


            min_distance_right = min(lidar_distances_right)
            angle_right = lidar_distances_right.index(min_distance_right)

            min_distance_left = min(lidar_distances_left)
            angle_left = lidar_distances_left.index(min_distance_left)
            

            if (remaining_angle == 0):
                if (min_distance_front < SAFE_STOP_DISTANCE):
                    obstacle_front = True
                else :
                    obstacle_front = False

                if (min_distance_right < SAFE_STOP_DISTANCE):
                    obstacle_right = True
                else :
                    obstacle_right = False
                
                if (min_distance_left < SAFE_STOP_DISTANCE):
                    obstacle_left = True
                else :
                    obstacle_left = False


                # Calculate remaining angle
                if (not obstacle_front):
                    remaining_angle = 0
                elif (obstacle_front and not obstacle_right):
                    remaining_angle = 30
                elif (obstacle_front and obstacle_right and not obstacle_left):
                    remaining_angle = -30
                elif (obstacle_front and obstacle_right and obstacle_left):
                    # box
                    remaining_angle = 180
            
            print(remaining_angle, obstacle_front, obstacle_right, obstacle_left)




            if (remaining_angle == 0):
                twist.linear.x = MAX_LINEAR_VEL
                twist.angular.z = 0
                self._cmd_pub.publish(twist)
                time.sleep(0.5)
                twist.angular.z = 0
                self._cmd_pub.publish(twist)




            if (remaining_angle == 30 or remaining_angle == 15):
                twist.angular.z = 0.5 * MAX_ANGULAR_VEL
                twist.linear.x = 0.5 * MAX_LINEAR_VEL
                self._cmd_pub.publish(twist)
                time.sleep(0.5)
                remaining_angle -= 15
                twist.angular.z = 0
                self._cmd_pub.publish(twist)

            if (remaining_angle == -30 or remaining_angle == -15):
                twist.angular.z = -0.5 * MAX_ANGULAR_VEL
                twist.linear.x = 0.5 * MAX_LINEAR_VEL
                self._cmd_pub.publish(twist)
                time.sleep(0.5)
                remaining_angle += 15
                twist.angular.z = 0
                self._cmd_pub.publish(twist)

            # Box
            if (remaining_angle == 180):
                twist.angular.z = ANGULAR_VEL
                self._cmd_pub.publish(twist)
                time.sleep(1)
                twist.angular.z = 0
                self._cmd_pub.publish(twist)


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
