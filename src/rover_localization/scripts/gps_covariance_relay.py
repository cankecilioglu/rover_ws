#!/usr/bin/env python3
"""
Gazebo NavSat plugin noise uygular ama position_covariance field'ını
boş bırakır. Bu node /gps topic'ini alıp, URDF'teki noise stddev'lerine
karşılık gelen covariance ile /gps/fix olarak yeniden yayınlar.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsCovarianceRelay(Node):
    def __init__(self):
        super().__init__('gps_covariance_relay')

        # URDF'teki noise stddev'lerine uygun (yatay ~1m, dikey ~1.5m)
        self.declare_parameter('horizontal_stddev', 1.0)
        self.declare_parameter('vertical_stddev', 1.5)

        h = self.get_parameter('horizontal_stddev').value
        v = self.get_parameter('vertical_stddev').value

        # 3x3 row-major: diagonal = variance (stddev²)
        self.cov = [h*h, 0.0, 0.0,
                    0.0, h*h, 0.0,
                    0.0, 0.0, v*v]

        self.sub = self.create_subscription(NavSatFix, '/gps', self.cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.get_logger().info(
            f'GPS covariance relay started (h_std={h}m, v_std={v}m)')

    def cb(self, msg: NavSatFix):
        msg.position_covariance = self.cov
        msg.position_covariance_type = 2  # DIAGONAL_KNOWN
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GpsCovarianceRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()