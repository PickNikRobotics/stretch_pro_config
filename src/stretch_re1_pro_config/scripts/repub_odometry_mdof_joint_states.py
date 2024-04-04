#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import MultiDOFJointState

import tf2_geometry_msgs
from geometry_msgs.msg import Transform, Pose, Vector3, Wrench

class OdometryMultiDOFJointStateRepublisher(Node):
    """! The Odometry to MultiDOFJointState republisher node.

    Subscribes to Odometry messages, copies the data into a MultiDOFJointState,
    which are published to '/multi_dof_joint_states'.
    """

    def __init__(self, odom_topic, mdof_topic):
        """! Initialize Odometry subscriber and MultiDOFJointState publisher.

        @param odom_topic topic name to use for Odometry subscription
        @param mdof_topic topic name to use for MultiDOFJointState publishing

        @return Instance of the OdometryMultiDOFJointStateRepublisher node
        """

        super().__init__('odometry_mdof_state_republisher')
        self.odom_sub_ = self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)
        self.mdof_state_pub_ = self.create_publisher(MultiDOFJointState, mdof_topic, 1)

    def odom_callback(self, odom_msg):
        """! Subscription callback to run for incoming Odometry messages

        Each Odometry message is copied into a MultiDOFJointState message and republished 
        to a different topic.

        """
        mdof_state_msg = MultiDOFJointState()
        mdof_state_msg.joint_names = ["position"]
        transform = Transform()
        transform.translation.x = odom_msg.pose.pose.position.x
        transform.translation.y = odom_msg.pose.pose.position.y
        transform.translation.z = odom_msg.pose.pose.position.z
        transform.rotation = odom_msg.pose.pose.orientation
        mdof_state_msg.transforms = [transform]
        mdof_state_msg.twist = [odom_msg.twist.twist]
        mdof_state_msg.wrench = [Wrench()]
        self.mdof_state_pub_.publish(mdof_state_msg)

def main(args=None):
  rclpy.init(args=args)

  # TODO(moveit_studio#7004) Expose topic names and joint name to launch config
  odometry_repub = OdometryMultiDOFJointStateRepublisher('/diff_drive_controller/odom', '/multi_dof_joint_states')

  rclpy.spin(odometry_repub)

  odometry_repub.destroy_node()

  rclpy.shutdown()

if __name__ == '__main__':
    main()
