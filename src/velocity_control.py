#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
# from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import tf.transformations as trans
# from PID import PIDRegulator
from cv_bridge import CvBridge, CvBridgeError
import cv2
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np

class Environment():
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('rexrov/cmd_vel', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber('/vx', Float32, self.control_callback)
        # self.velocity_publisher = rospy.Subscriber('odom', Twist, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.vx = 0.
        # self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber('/rexrov/rexrov/camera/camera_image',Image,self.image_callback)
        # self.image_original = None

        # linear_velocity = Twist()

        # linear_velocity.linear.x = 0.2

    def control_callback(self, data):
        print('aqui', data.data)
        self.vx = data.data


    def shutdown(self):
        print('Shutting!')
        linear_velocity = Twist()

        linear_velocity.linear.x = 0
        linear_velocity.linear.y = 0
        linear_velocity.linear.z = 0

        self.velocity_publisher.publish(linear_velocity)
        rospy.sleep(1)

    def step(self):
        linear_velocity = Twist()
        

        # cmd_vel.linear = geometry_msgs.Vector3(*v_linear)
        # cmd_vel.angular = geometry_msgs.Vector3(*v_angular)

        print("control: %0.3f"%(self.vx))

        # linear_velocity.linear.x = 2.*self.vx
        linear_velocity.linear.x = 0.5
        linear_velocity.linear.y = 0.
        linear_velocity.linear.z = 0.
        linear_velocity.angular.z = 0.
        linear_velocity.angular.z = 1.*-self.vx

        self.velocity_publisher.publish(linear_velocity)


    
    


if __name__ == '__main__':
    rospy.init_node('control')

    rate = rospy.Rate(100)

    env = Environment()
    # linear_velocity = 0.2
    while not rospy.is_shutdown():
        # print('foi')
        env.step()
        rate.sleep()
        