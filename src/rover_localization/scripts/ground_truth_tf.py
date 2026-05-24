#!/usr/bin/env python3
"""
Ground truth TF publisher.
Subscribes to /odometry/ground_truth (Gazebo bridge),
publishes TF odom → base_footprint with correct z-offset.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# base_footprint → base_link offset from URDF (wheel_radius + chassis_height/2)
BASE_LINK_Z_OFFSET = 0.0625


class GroundTruthTfNode(Node):
    def __init__(self):
        super().__init__('ground_truth_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, '/odometry/ground_truth', self.odom_cb, 10
        )
        self.get_logger().info(
            'Ground truth TF publisher: subscribing to /odometry/ground_truth, '
            'publishing odom→base_footprint'
        )

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        # base_link pose - z_offset = base_footprint pose
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z - BASE_LINK_Z_OFFSET
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GroundTruthTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()