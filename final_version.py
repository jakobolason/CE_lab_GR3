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


import smbus
import RPi.GPIO as GPIO
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import select
import threading

bus = smbus.SMBus(1)

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO_LED = 27

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO_LED, GPIO.OUT)  # Initial state as output

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO_LED, False)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)


MAX_ANGULAR_VEL = 1.4
MAX_LINEAR_VEL = 0.21
STOP_DISTANCE = 0.19
SAFE_STOP_DISTANCE = STOP_DISTANCE
SAFE_STOP_DISTANCE_TO_ACT = 2.8 * SAFE_STOP_DISTANCE
VIEWING_ANGLE = 60
CRITICAL_STOP_DISTANCE = 0.16

# start time
start_time = time.time()

def blink_led(sleep):
    # Blink the led
    #print("Blink")
    GPIO.output(GPIO_TRIGECHO_LED, 1)
    time.sleep(sleep)
    GPIO.output(GPIO_TRIGECHO_LED, 0)
    time.sleep(sleep)
    GPIO.output(GPIO_TRIGECHO_LED, 1)
    time.sleep(sleep)
    GPIO.output(GPIO_TRIGECHO_LED, 0)
    time.sleep(sleep)


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
        samples_view = VIEWING_ANGLE            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        # Error handling for data
        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = SAFE_STOP_DISTANCE_TO_ACT + 0.01
            elif scan_filter[i] == 0:
                scan_filter[i] = SAFE_STOP_DISTANCE_TO_ACT + 0.01
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = SAFE_STOP_DISTANCE_TO_ACT + 0.01
            elif scan_filter[i] > 3 * SAFE_STOP_DISTANCE_TO_ACT:
                scan_filter[i] = 3 * SAFE_STOP_DISTANCE_TO_ACT
        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        victims_found = 0
        self.iterations = 0
        self.accumulated_speed = 0
        self.collisions = 0
        self.collision = False
        self.count_color = 0
        self.tunnel = False

        # Run for 2 minutes
        while ((time.time() - start_time) < 120):

            # Read the data from the sensor
            data = bus.read_i2c_block_data(0x44, 0x09, 6)

            # Red
            block_low_red = data[2] / 256
            block_high_red = data[3]
            red = (block_low_red + block_high_red)

            # Green
            block_low_green = data[0] / 256
            block_high_green = data[1]
            green = (block_low_green + block_high_green) # Convert value to between 0 and 256.

            # Blue
            block_low_blue = data[4] / 256
            # Blue is inacurate, we found that multiplying it with 1.9 works
            block_high_blue = int(data[5] * 1.9)
            blue = (block_low_blue + block_high_blue)

            #print("Color: (" + str(red) + ", " + str(green) + ", " + str(blue) + ")")

            if (red >= green and red >= blue and self.count_color >= 10):
                print("VICTIM")
                self.count_color = 0
                my_timer = threading.Timer(0.0, blink_led, args=[0.25])
                my_timer.start()
                victims_found += 1


            if (green > red and green > blue):
                #print("green")
                self.count_color += 1

            if (blue > red and blue > green):
                #print("blue")
                self.count_color += 1


            lidar_distances_left = []
            lidar_distances_right = []

            if self.is_key_available():
                char = self.get_key()
                if char.lower() == 'q':
                    print("----------------------------------------------")
                    print("Total Victims: " + str(victims_found))
                    average_vel = self.accumulated_speed / self.iterations
                    print("Average Linear speed: " + str(average_vel))
                    print("Collisions: " + str(self.collisions))
                    print("----------------------------------------------")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    return
            
            # Get the values
            data = self.get_scan()
            for i in range(len(data)):
                if (0 <= i <= VIEWING_ANGLE // 2 - 1):
                    lidar_distances_right.append(data[i])
                else:
                    lidar_distances_left.append(data[i])

            # Reverse the left lidar list so both arrays goes from the 0th angle and then indexing outwards 
            lidar_distances_right = lidar_distances_right[::-1]


            def turn(self, angle):
                #sleep_time = 0.1
                self.collision_error = 0
            
                # Get the distane to the nearest object
                closets_distance = min(min(lidar_distances_left), min(lidar_distances_right))

                # Calculate the linear scalar based on the angle we need to turn and the distance to the closest object
                lin_scalar_from_angle = num_to_range(abs(angle), 0, VIEWING_ANGLE, 1, 0)
                lin_scalar_from_distance =  num_to_range(closets_distance, SAFE_STOP_DISTANCE, SAFE_STOP_DISTANCE_TO_ACT, 0, 1)

                if lin_scalar_from_distance == 1:
                    lin_scalar = 1
                elif lin_scalar_from_distance == 0:
                    lin_scalar = 0
                elif lin_scalar_from_angle == 1:
                    lin_scalar = 1
                else:
                    lin_scalar = lin_scalar_from_distance * lin_scalar_from_angle

                if (self.tunnel):
                    lin_scalar = 1
                    self.tunnel = False


                # Check for collision
                for i in range(len(lidar_distances_left)):
                    if (lidar_distances_left[i] < CRITICAL_STOP_DISTANCE or lidar_distances_right[i] < CRITICAL_STOP_DISTANCE):
                        self.collision_error += 1


                if (self.collision_error >= 3 and not self.collision):
                    self.collisions += 1
                    self.collision = True
                    print("Collision")
                    twist.angular.z = 0
                    twist.linear.x = 0
                    self._cmd_pub.publish(twist)
                    blink_led(0.25)

                elif (closets_distance > CRITICAL_STOP_DISTANCE):
                    self.collision = False

                # map the angle to the angle_scalar for postive and negative angles
                if (angle >= 0):
                    # Soft turn (0.75 to 0)
                    angle_scalar = -1 + lin_scalar
                elif (angle < 0):
                    #print("negative")
                    angle_scalar = 1 - lin_scalar
                
                print(angle, lin_scalar, angle_scalar)

                # Turn
                twist.angular.z = angle_scalar * MAX_ANGULAR_VEL
                twist.linear.x = lin_scalar * MAX_LINEAR_VEL
                self.iterations += 1
                self.accumulated_speed += lin_scalar * MAX_LINEAR_VEL
                self._cmd_pub.publish(twist)

            
            def calculate_angle(self):
                angle = 0
                closets_distance = min(min(lidar_distances_left), min(lidar_distances_right))
                self.count = 0

                # The robot is not close to anything so move forward
                if (closets_distance > SAFE_STOP_DISTANCE_TO_ACT):
                    angle = 0
                    return angle

                else:    
                    # Something is within SAFE_STOP_DISTANCE_TO_ACT and we need to make a decision on what to do
                    # Checks if there is no obstacle left and an obstacle is detected right
                    if (min(lidar_distances_left) > SAFE_STOP_DISTANCE_TO_ACT and min(lidar_distances_right) < SAFE_STOP_DISTANCE_TO_ACT):
                        # we must turn left
                        closest_object_right_angle = lidar_distances_right.index(closets_distance)
                        angle = -VIEWING_ANGLE / 2 + closest_object_right_angle
                        return angle

                    # Checks if there is no obstacle right and an obstacle is detected left 
                    elif (min(lidar_distances_left) < SAFE_STOP_DISTANCE_TO_ACT and min(lidar_distances_right) > SAFE_STOP_DISTANCE_TO_ACT):
                        # We must turn right
                        closest_object_left_angle = lidar_distances_left.index(closets_distance)
                        angle = VIEWING_ANGLE / 2 - closest_object_left_angle
                        return angle             
            
                    else:    
                        # There is an obstacle in the left and right sides within SAFE_STOP_DISTANCE_TO_ACT
                        # We need to figure out what side to turn is most optimal
                        # We take the average distance over the entire left and right arrays
                        avg_range_left = sum(lidar_distances_left) / len(lidar_distances_left)
                        avg_range_right = sum(lidar_distances_right) / len(lidar_distances_right)


                        # Box case
                        if (avg_range_left < CRITICAL_STOP_DISTANCE * 2 and avg_range_right < CRITICAL_STOP_DISTANCE * 2):
                            twist.angular.z = 1
                            print("BOX-BOX")
                            twist.linear.x = 0
                            self._cmd_pub.publish(twist)
                            time.sleep(1.5)
                            angle = 0
                            return 0


                        # Tunnel
                        lidar_right_8 = lidar_distances_right[:7]
                        lidar_left_8 = lidar_distances_left[:7]
                        for i in range(len(lidar_right_8)):
                            if (lidar_right_8[i] > SAFE_STOP_DISTANCE_TO_ACT * 0.85):
                                self.count += 1
                            if (lidar_left_8[i] > SAFE_STOP_DISTANCE_TO_ACT * 0.85):
                                self.count += 1
                        if (self.count == (len(lidar_left_8) + len(lidar_right_8))):
                            angle = 0
                            self.tunnel = True
                            #print("tunnel")
                            return angle

                        # The highest must be the most optimal way to turn
                        if (avg_range_left > avg_range_right):
                            # Turn left
                            closets_distance_right = min(lidar_distances_right)
                            closest_object_right_angle = lidar_distances_right.index(closets_distance_right)
                            angle = - VIEWING_ANGLE / 2 + closest_object_right_angle
                            return angle
                        else:
                            # Turn right
                            closets_distance_left = min(lidar_distances_left)
                            closest_object_left_angle = lidar_distances_left.index(closets_distance_left)
                            angle = VIEWING_ANGLE / 2 - closest_object_left_angle
                            return angle


            # Maps a number between two values to another two values
            def num_to_range(num, inMin, inMax, outMin, outMax):
                if (num > inMax):
                    return outMax
                elif (num < inMin):
                    return outMin
                return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax- outMin))

            # calculate the angle to turn
            angle = calculate_angle(self)

            # turn that angle
            turn(self, angle)
        
        # End of 2 minutes
        print("----------------------------------------------")
        print("Total Victims: " + str(victims_found))
        average_vel = self.accumulated_speed / self.iterations
        print("Average Linear speed: " + str(average_vel))
        print("Collisions: " + str(self.collisions))
        print("----------------------------------------------")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
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
