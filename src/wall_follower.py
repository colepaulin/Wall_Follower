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

    def Scan_Slice(self,scan,angles):

        half_length = int(len(scan.ranges))/2
        if self.SIDE == -1:
            sliced_range = scan.ranges[half_length:]
            sliced_angles = angles[half_length:]
        if self.SIDE == 1:
            sliced_range = scan.ranges[:half_length]
            sliced_angles = angles[:half_length]
        return sliced_range,sliced_angles

    def datapoint_angles(self,scan):
        increment = scan.angle_increment
        angle_min = scan.angle_min
        angles = [angle_min]
        for i in range(1,len(scan.ranges)):
            prev = angles[-1]
            angles.append(prev + increment)
        return np.array(angles)


    def find_the_wall(self,sliced_range,sliced_angles):
        x_dist_ranges = np.cos(sliced_angles)*sliced_range
        y_dist_ranges = np.sin(sliced_angles)*sliced_range
        # x_squared = np.square(x_dist_ranges)
        # y_squared = np.square(y_dist_ranges)
        #
        # distances = np.sqrt(x_squared+y_squared)
        # wall_distance = np.mean(distances)
        [theta, wall_distance] = np.polyfit(x_dist_ranges,y_dist_ranges,1)
        return theta, wall_distance

    def control(self,theta,wall_distance):
        error = self.DESIRED_DISTANCE - wall_distance
        Kp = 5
        Kd = 1
        control_action = Kp*error + Kd*self.VELOCITY*theta
        return control_action


    def callback(self,scan):
        angles = self.datapoint_angles(scan)
        sliced_range,sliced_angles = self.Scan_Slice(scan,angles) # uses either the left or right side of the data

        (theta,wall_distance) = self.find_the_wall(sliced_range,sliced_angles)
        control_action = self.control(theta,wall_distance)


        drive_stamp = AckermannDriveStamped()
        drive_stamp.header.stamp = rospy.Time.now()
        drive_stamp.header.frame_id = "wall_follower"
        drive_stamp.drive.steering_angle = control_action
        drive_stamp.drive.steering_angle_velocity = 0
        drive_stamp.drive.speed = self.VELOCITY
        drive_stamp.drive.acceleration = 0
        drive_stamp.drive.jerk = 0
        self.drive_pub.publish(drive_stamp)



if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
