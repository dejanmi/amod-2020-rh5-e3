#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, String


class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                      WheelEncoderStamped, self.callback_left)
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                      WheelEncoderStamped, self.callback_right)
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed',
                                                      WheelsCmdStamped, self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'/{self.veh_name}/odometry_node/integrated_distance_left',
                                                            String, queue_size=10)
        self.pub_integrated_distance_right = rospy.Publisher(f'/{self.veh_name}/odometry_node/integrated_distance_right',
                                                            String, queue_size=10)

        self.log("Initialized")

        self.encoder_ticks_left = 0
        self.encoder_ticks_right = 0
        self.integrated_distance_left = 0.0
        self.integrated_distance_right = 0.0
        self.direction_of_travel_left = 0
        self.direction_of_travel_right = 0
        self.radius_r = 0
        self.radius_l = 0
        self.radius = 0

    def callback_right(self, data):
        self.encoder_ticks_right = data.data

    def callback_left(self, data):
        self.encoder_ticks_left = data.data

    def cb_encoder_data(self, wheel):
        """ Update encoder distance information from ticks.
        """
        if wheel == "right":
            self.integrated_distance_right = 2 * np.pi * self._radius * \
                                             self.encoder_ticks_right / 135
        else:
            self.integrated_distance_left = 2 * np.pi * \
                                            self._radius * self.encoder_ticks_left / 135


    def cb_executed_commands(self, data):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        if data.vel_left < 0.0:
            self.direction_of_travel_left = -1
        elif data.vel_left > 0.0:
            self.direction_of_travel_left = 1
        else:
            self.direction_of_travel_left = 0

        if data.vel_right < 0.0:
            self.direction_of_travel_right = -1
        elif data.vel_right > 0.0:
            self.direction_of_travel_right = 1
        else:
            self.direction_of_travel_right = 0

    def calculate_radius(self, wheel):
        if wheel == "right" and self.encoder_ticks_right != 0:
            self.radius_r = 1.0 * 135 / (2 * np.pi * self.encoder_ticks_right)
        elif wheel == "left" and self.encoder_ticks_left != 0:
            self.radius_l = 1.0 * 135 / (2 * np.pi * self.encoder_ticks_left)

    def publish_distance(self):
        while not rospy.is_shutdown():
            self.cb_encoder_data("right")
            self.cb_encoder_data("left")
            self.calculate_radius("right")
            self.calculate_radius("left")
            self.pub_integrated_distance_left.publish(str(self.integrated_distance_left))
            self.pub_integrated_distance_right.publish(str(self.integrated_distance_right))
            self.radius = (self.radius_r + self.radius_l) / 2
            # print(f'radius left: {self.radius_l}')
            # print(f'radius right: {self.radius_r}')
            # print(f'radius: {self.radius}')

if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    node.publish_distance()
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
