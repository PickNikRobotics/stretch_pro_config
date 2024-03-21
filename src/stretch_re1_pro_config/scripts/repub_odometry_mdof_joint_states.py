import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import MultiDOFJointState

import tf2_geometry_msgs
from geometry_msgs.msg import Transform, Pose, Vector3, Wrench

class OdometryMultiDOFJointStateRepublisher(Node):
    def __init__(self):
        super().__init__('odometry_mdof_state_republisher')
        self.odom_sub_ = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 1)
        self.mdof_state_pub_ = self.create_publisher(MultiDOFJointState, '/multi_dof_joint_states', 1)

    def odom_callback(self, odom_msg):
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

  odometry_repub = OdometryMultiDOFJointStateRepublisher()

  rclpy.spin(odometry_repub)

  odometry_repub.destroy_node()

  rclpy.shutdown()

if __name__ == '__main__':
    main()
