#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size = 10)
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.callback)



    # TODO:
    # Write your callback functions here.\

    def AckermannInstance_thenpublish(self,steering_angle_tp,steering_angle_velocity_tp,speed_tp,acceleration_tp,jerk_tp):
        drive_stamp = AckermannDriveStamped()
        drive_stamp.header.stamp = rospy.Time.now()
        drive_stamp.header.frame_id = "wall_follower"
        drive_stamp.drive.steering_angle = steering_angle_tp
        drive_stamp.drive.steering_angle_velocity = steering_angle_velocity_tp
        drive_stamp.drive.speed = speed_tp
        drive_stamp.drive.acceleration = acceleration_tp
        drive_stamp.drive.jerk = jerk_tp
        self.drive_pub.publish(drive_stamp)

    def Scan_Slice(self,):

    def callback(self,scan):
        

        self.AckermannInstance_thenpublish(0,1,2,0,0)




if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
