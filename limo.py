#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#

import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import cos, sin, pi, isnan, isinf
import math

# Parameters
linear_speed = 0.2
left_turn_angle = 60 * (pi / 180)  
right_turn_angle = 120 * (pi / 180)  
turn_angle = 90 * (pi/180)
angular_speed = 0.35  
min_dist = 0.85  
min_angle_deg = -20.0  
max_angle_deg = 20.0 

class Move:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('move')

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_tag_callback, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.color_sub = rospy.Subscriber("object_detect_pose", Pose, self.color_callback) 

        self.obstacle_detected = False
        self.obstacle_handled = False  
        self.red_detected = False
        self.ar_tag_detected_marker1 = False
        self.ar_tag_detected_marker2 = False
        self.ignore_lidar = False  
       
        self.drive_data = Twist()
        self.rate = rospy.Rate(10)  

        self.objPose = Pose()
        self.objPose.position.x = 0
        self.objPose.position.y = 0
        self.objPose.position.z = 0

    

    def drive_marker_1(self):
        rospy.loginfo("marker 1:  right turn")
        self.turn_L(turn_angle)
        self.default_B()  

    def drive_marker_2(self, qr_x, qr_y, qr_z):
        rospy.loginfo("marker 2:  left turn")
        self.turn_R(turn_angle)
        self.default_B()  

    def ar_tag_callback(self, msg):
        self.ar_tag_detected_marker1 = False
        self.ar_tag_detected_marker2 = False

        for marker in msg.markers:
            self.qr_x = marker.pose.pose.position.x
            self.qr_y = marker.pose.pose.position.y
            self.qr_z = marker.pose.pose.position.z
            rospy.loginfo("Target{} Pose: x:{}, y:{}, z:{}".format(marker.id, self.qr_x, self.qr_y, self.qr_z))

            if self.qr_x < 1.9:
                if marker.id == 1:
                    self.ar_tag_detected_marker1 = True
                    self.ignore_lidar = True
                elif marker.id == 2:
                    self.ar_tag_detected_marker2 = True
                    self.ignore_lidar = True
                    
  
    def move_go_straight(self):
        self.drive_data.linear.x = linear_speed
        self.drive_data.angular.z = 0.0
        self.vel_pub.publish(self.drive_data)
        self.rate.sleep()

    def turn_L(self, angle):
        duration = angle / angular_speed

        self.drive_data.linear.x = 0.2
        self.drive_data.angular.z = angular_speed

        end_time = rospy.Time.now().to_sec() + duration
        while rospy.Time.now().to_sec() < end_time:
            self.vel_pub.publish(self.drive_data)
            self.rate.sleep()

        self.drive_data.angular.z = 0.0
        self.vel_pub.publish(self.drive_data)

    def turn_R(self, angle):
        duration = angle / angular_speed

        self.drive_data.linear.x = 0.2
        self.drive_data.angular.z = -angular_speed

        end_time = rospy.Time.now().to_sec() + duration
        while rospy.Time.now().to_sec() < end_time:
            self.vel_pub.publish(self.drive_data)
            self.rate.sleep()

        self.drive_data.angular.z = 0.0
        self.vel_pub.publish(self.drive_data)

    def stop(self):
        self.drive_data.linear.x = 0
        self.drive_data.angular.z = 0
        self.vel_pub.publish(self.drive_data)
        rospy.signal_shutdown("Red color detected, shut down")

    def default_B(self):
        rospy.loginfo("Move straight")
        self.move_go_straight()

    def scan_callback(self, data):
        if not self.ignore_lidar and not self.obstacle_handled:  
            self.obstacle_detected = False
            valid_readings = False 

            for i, distance in enumerate(data.ranges):
                if isnan(distance) or isinf(distance) or distance == 0.0:
                    continue
                
                angle = data.angle_min + data.angle_increment * i
                angle_deg = angle * 180 / pi  

                # Check if the angle is within the specified range and the distance is less than the minimum distance
                if min_angle_deg < angle_deg < max_angle_deg and distance < min_dist:
                    rospy.loginfo("Obstacle detected at distance {:.2f} meters and angle {:.2f} degrees".format(distance, angle_deg))
                    self.obstacle_detected = True
                    valid_readings = True
                    break  

            if not valid_readings:
                rospy.loginfo("No valid")

    # Main
    def run(self):
        while not rospy.is_shutdown():
            if self.red_detected:
                rospy.loginfo("Stop")
                self.stop()

            elif self.obstacle_detected and not self.obstacle_handled:
                # Obstacle detected by lidar: perform obstacle avoidance maneuvers
                self.turn_L(left_turn_angle)
                self.turn_R(right_turn_angle)
                self.turn_L(left_turn_angle)
                self.ignore_lidar = True
                self.obstacle_detected = False  
                self.obstacle_handled = True  
                self.default_B()

            elif self.ar_tag_detected_marker1:
                # AR tag detected: perform a 90-degree right turn
                rospy.loginfo("AR tag detected. right turn")
                self.drive_marker_1()
                self.ar_tag_detected_marker1 = False

            elif self.ar_tag_detected_marker2:
                # AR tag detected: perform a specific action
                rospy.loginfo("AR tag detected. specific action")
                self.drive_marker_2(self.qr_x, self.qr_y, self.qr_z)  
                self.ar_tag_detected_marker2 = False

            else:
                self.default_B()

if __name__ == '__main__':
    try:
        node = Move()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("finished.")

