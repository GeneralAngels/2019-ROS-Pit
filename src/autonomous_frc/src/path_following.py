#!/usr/bin/env python
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
from math import *
import tf
import rospy


def sign(num):
    if num < 0:
        return -1
    else:
        return 1

class Path_following:
    def __init__(self):
        self.kv = 2
        self.kwf = 3 # omega forward
        self.kwb = 1 # omega backwards
        self.path = Path()
        self.path.poses.append(PoseStamped())
        self.point_index = 0
        self.min_radius = 0.5
        self.my_odometry = Odometry()

    def calc_errors(self):
        quaternion = self.my_odometry.pose.pose.orientation
        my_heading = tf.transformations.euler_from_quaternion((quaternion.x,quaternion.y,quaternion.z,quaternion.w))[2]
        my_point = self.my_odometry.pose.pose.position
        target_point = self.path.poses[self.point_index].pose.position
        error_position =  sqrt((my_point.x - target_point.x)**2+(my_point.y - target_point.y)**2)
        target_heading = atan2((target_point.y - my_point.y),(target_point.x - my_point.x))
        error_heading =  my_heading - target_heading
        if error_heading>3.14159:
            error_heading-=6.2832
        if error_heading<-3.14159:
            error_heading+=6.2832

        return (error_position,error_heading)

    def calc_signal(self,errors):
        direction = sign(cos(errors[1]))
        signal_linear_velocity = errors[0] * direction * self.kv
        if direction>0:
            signal_angular_velocity = errors[1] * self.kwf
        else:
            signal_angular_velocity = errors[1] * self.kwb

        return (signal_linear_velocity,signal_angular_velocity)

    def calc_point_index(self):
        my_point = self.my_odometry.pose.pose.position
        target_point = self.path.poses[self.point_index].pose.position
        if sqrt((my_point.x - target_point.x)**2+(my_point.y - target_point.y)**2) <= self.min_radius:
            self.point_index +=1

    def path_callback(self,path):
        self.path = path
        self.point_index =  0

    def odom_callback(self,odom):
        self.my_odometry = odom



if __name__ == '__main__':
    rospy.init_node('path_following')
    my_follower = Path_following()
    rate = rospy.Rate(50)
    rospy.Subscriber('path', Path, my_follower.path_callback)
    rospy.Subscriber('odom', Odometry, my_follower.odom_callback)
    cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=20 )
    drive_command = Twist()
    stop_flag=False

    while not rospy.is_shutdown():
        if len(my_follower.path.poses) <= my_follower.point_index:
            if stop_flag:
                drive_command.angular.z = 0
                drive_command.linear.x = 0
                cmd_publisher.publish(drive_command)
                stop_flag=False
        else:
            errors = my_follower.calc_errors()
            linear , angular =  my_follower.calc_signal(errors)
            drive_command.angular.z = -angular
            drive_command.linear.x = linear
            cmd_publisher.publish(drive_command)
            my_follower.calc_point_index()
            stop_flag=True
        rate.sleep()
