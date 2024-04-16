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
SAFE_CURCE_DISTANCE = 2* SAFE_STOP_DISTANCE

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
                if (lidar_distances_right[i] <  SAFE_CURCE_DISTANCE):
                    right_boolean.append('True')
                else :
                    right_boolean.append('False')
                
                if (lidar_distances_left[i] < SAFE_CURCE_DISTANCE):
                    left_boolean.append('True')
                else :
                    left_boolean.append('False')

            reversed_right_boolean = right_boolean
            reversed_right_boolean.reverse()

            # With the boolean arrays for left and right, we check were the first obstacle is, looking from the middle and out.
            try:
                object_angle_right =  -45 + left_boolean.index('True') # Using index, we get the number of angles from the middle and out, 
            except:                                                    # meaning we sum that with -45 to get the correct angle
                object_angle_right = 0
            
            try:
                object_angle_left = 45 - reversed_right_boolean.index('True')
            except:
                object_angle_left = 0  
            
            # Update remaining angle
            if (remaining_angle == 0):  # If there is no calculated remaining angle, we need to check if there needs to be. 
                remaining_angle = check_for_remaining_angle(object_angle_left, object_angle_right)
        
            # print(remaining_angle)

            # If the remaining_angle is still 0, then there is no object, and we continue
            if (remaining_angle == 0):
                turn(remaining_angle)
                continue
            

            # ------------ Now we check how much the bot has to turn -------------- #
            elif (remaining_angle != 0):
                closest_object_left = min(lidar_distances_left)
                closest_object_right = min(lidar_distances_right)


                # This is our calculated angle_scalar, to determine the radius of the turn
                # The closer the bot is to the object, the higher the angle_scaler is
                angle_scalar = remaining_angle / ((min(closest_object_left, closest_object_right)) * 100)

                ## For troubleshooting, the angle_scalar can get higher than 1
                if (angle_scalar > 1):
                    angle_scalar = 1

                #print(angle_scalar)
                #print(min(min(lidar_distances_right), min(lidar_distances_left)) * 100)


                if (min( closest_object_left, closest_object_right ) * 100 <= SAFE_CURCE_DISTANCE):
                    ### If we are closer than our SAFE_CURVE_DISTANCE to the object, we need to make a sharp turn ### 
                    sharp_turn(remaining_angle)
                    continue
    
                
                lin_scalar = 1 - abs(angle_scalar)
                if (lin_scalar < 0):
                    lin_scalar = 0

                twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                twist.linear.x = lin_scalar * MAX_LINEAR_VEL
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                # twist.angular.z = 0
                # self._cmd_pub.publish(twist)
                left_boolean = []
                right_boolean = []


        def check_for_remaining_angle(self, object_angle_left, object_angle_right):
            if (right_boolean.count('False') == 45 and left_boolean.count('False') == 45):
                # If there are no objects, continue forwards
                return 0
            
            elif (right_boolean.count('False') == 45 and left_boolean.count('True') > 0):
                # turn right
                return object_angle_right

            elif (left_boolean.count('False') == 45 and right_boolean.count('True') > 0):
                # turn left
                return object_angle_left
            
            elif (right_boolean.count('True') > 0 and left_boolean.count('True') > 0):
                #print('middle')
                lowest_angle = min(angle_left, abs(angle_right))
                if (lowest_angle == abs(angle_right)):
                    return angle_right
                else :
                    return angle_left
    
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

        def sharp_turn(self, remaining_angle):
            if (remaining_angle >= 0):
                # And the calculated remaining angle of the object is to the left, we turn right
                angle_scalar = -1 # Right is minus
                twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                twist.linear.x = 0
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                remaining_angle = 0
            else:
                # else the object must be to the right, turn left
                angle_scalar = 1
                twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                twist.linear.x = 0
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                remaining_angle = 0


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