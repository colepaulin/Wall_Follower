#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from copy import deepcopy

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")*(-1)
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):

        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.callback)


    def scan_slice(self,scan,angles):
        half = int(len(scan.ranges)) /2
        if self.SIDE == -1:
            scan.ranges = scan.ranges[half:]
            sliced_angles = angles[half:]
        if self.SIDE == 1:
            scan.ranges = scan.ranges[:half]
            sliced_angles = angles[:half]

        return scan,sliced_angles

    def datapoint_angles(self,scan):
        return np.arange(scan.angle_min,scan.angle_max,scan.angle_increment)


    def find_the_wall(self,sliced_range,sliced_angles):
        x_dist_ranges_temp = np.cos(sliced_angles)*sliced_range
        y_dist_ranges_temp = np.sin(sliced_angles)*sliced_range

        ### get rid of edge cases. we only want scans that are close-ish to a wall
        long_values_index = np.where(y_dist_ranges_temp > 2.0*self.DESIRED_DISTANCE) ## 3.5 passes evens
        x_dist_ranges = np.delete(x_dist_ranges_temp,long_values_index)
        y_dist_ranges = np.delete(y_dist_ranges_temp,long_values_index)

        [theta, wall_distance] = np.polyfit(x_dist_ranges,y_dist_ranges,1)
        return theta, wall_distance

    def control(self,theta,wall_distance):
        error = self.DESIRED_DISTANCE - abs(wall_distance)
        Kp = 5.2
        Kd = 5.0*self.SIDE
        control_action = self.SIDE * Kp * error + self.SIDE * Kd * self.VELOCITY * theta
        return control_action


    def callback(self,scan):
        angles = self.datapoint_angles(scan)
        sliced_scan,sliced_angles = self.scan_slice(scan,angles) # uses either the left or right side of the data
        reg = self.find_the_wall(sliced_scan.ranges,sliced_angles)
        (theta,wall_distance) = (reg[0],reg[1])
        control_action = self.control(theta,wall_distance)
        drive_stamp = AckermannDriveStamped()
        drive_stamp.header.stamp = rospy.Time.now()
        drive_stamp.header.frame_id = "wall_follower"
        drive_stamp.drive.steering_angle = control_action
        drive_stamp.drive.steering_angle_velocity = 0
        drive_stamp.drive.speed = self.VELOCITY
        drive_stamp.drive.acceleration = 0
        drive_stamp.drive.jerk = 0
        self.pub.publish(drive_stamp)



if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
