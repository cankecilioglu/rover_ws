#!/usr/bin/env python3
"""
Aşama 6.3: OccupancyGrid yayını eklendi.
- /terrain/slope_grid       (engebe haritası)
- /terrain/roughness_grid   (pürüzlülük haritası)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid


class TerrainHeatmapNode(Node):
    GRID_SIZE = 100
    CELL_SIZE = 0.20
    MAP_EXTENT = GRID_SIZE * CELL_SIZE
    MAP_ORIGIN_X = -MAP_EXTENT / 2
    MAP_ORIGIN_Y = -MAP_EXTENT / 2

    # Normalizasyon: 30° eğim → tam kırmızı (100). 30°'ye kadar lineer.
    MAX_SLOPE_RAD = math.radians(30.0)
    MAX_ROUGHNESS = 5.0   # az² birimi, tune edilebilir

    def __init__(self):
        super().__init__('terrain_heatmap_node')

        # Grid state
        self.slope_sum  = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)
        self.rough_sum  = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)
        self.visit_count = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.int32)

        # En son okumalar
        self.last_pitch = 0.0
        self.last_roll  = 0.0
        self.last_az    = 9.81
        self.last_x = 0.0
        self.last_y = 0.0
        self.imu_count  = 0
        self.odom_count = 0

        # Subscribers
        self.create_subscription(Imu, '/imu', self.imu_cb, 100)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 50)

        # Publishers — heatmap grids
        self.slope_pub = self.create_publisher(OccupancyGrid, '/terrain/slope_grid', 10)
        self.rough_pub = self.create_publisher(OccupancyGrid, '/terrain/roughness_grid', 10)

        # Timers
        self.create_timer(1.0, self.print_status)        # 1Hz log
        self.create_timer(0.2, self.publish_grids)       # 5Hz publish

        self.get_logger().info(
            f'Heatmap node started. Grid: {self.GRID_SIZE}×{self.GRID_SIZE} '
            f'({self.CELL_SIZE*100:.0f}cm/cell, {self.MAP_EXTENT:.0f}m)'
        )
        self.get_logger().info(
            'Publishing: /terrain/slope_grid, /terrain/roughness_grid @ 5Hz'
        )

    @staticmethod
    def quat_to_euler(qx, qy, qz, qw):
        roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
        yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        return roll, pitch, yaw

    def world_to_cell(self, x, y):
        col = int((x - self.MAP_ORIGIN_X) / self.CELL_SIZE)
        row = int((y - self.MAP_ORIGIN_Y) / self.CELL_SIZE)
        if 0 <= row < self.GRID_SIZE and 0 <= col < self.GRID_SIZE:
            return row, col
        return None

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        self.last_roll, self.last_pitch, _ = self.quat_to_euler(q.x, q.y, q.z, q.w)
        self.last_az = msg.linear_acceleration.z
        self.imu_count += 1

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.last_x = p.x
        self.last_y = p.y
        self.odom_count += 1

        cell = self.world_to_cell(self.last_x, self.last_y)
        if cell is None:
            return
        row, col = cell

        slope = math.sqrt(self.last_pitch**2 + self.last_roll**2)
        roughness = (self.last_az - 9.81) ** 2

        self.slope_sum[row, col] += slope
        self.rough_sum[row, col] += roughness
        self.visit_count[row, col] += 1

    def _array_to_occupancy(self, arr, max_val):
        """Float array → int8 (0-100), ziyaret edilmemiş hücreler -1 (unknown)."""
        with np.errstate(invalid='ignore', divide='ignore'):
            normalized = (arr / max_val * 100.0).clip(0, 100)
        data = normalized.astype(np.int8)
        data[self.visit_count == 0] = -1
        return data

    def _make_grid_msg(self, data: np.ndarray, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.info.map_load_time = stamp
        msg.info.resolution = self.CELL_SIZE
        msg.info.width = self.GRID_SIZE
        msg.info.height = self.GRID_SIZE
        msg.info.origin.position.x = self.MAP_ORIGIN_X
        msg.info.origin.position.y = self.MAP_ORIGIN_Y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.flatten().tolist()
        return msg

    def publish_grids(self):
        now = self.get_clock().now().to_msg()

        with np.errstate(invalid='ignore', divide='ignore'):
            avg_slope = self.slope_sum / np.maximum(self.visit_count, 1)
            avg_rough = self.rough_sum / np.maximum(self.visit_count, 1)

        slope_data = self._array_to_occupancy(avg_slope, self.MAX_SLOPE_RAD)
        rough_data = self._array_to_occupancy(avg_rough, self.MAX_ROUGHNESS)

        self.slope_pub.publish(self._make_grid_msg(slope_data, now))
        self.rough_pub.publish(self._make_grid_msg(rough_data, now))

    def print_status(self):
        cell = self.world_to_cell(self.last_x, self.last_y)
        cell_str = f'cell=({cell[0]:3d},{cell[1]:3d})' if cell else 'cell=OUT '

        visited = int((self.visit_count > 0).sum())
        if self.visit_count.sum() > 0:
            with np.errstate(invalid='ignore', divide='ignore'):
                avg_slope = self.slope_sum / np.maximum(self.visit_count, 1)
            max_slope_deg = math.degrees(float(avg_slope.max()))
        else:
            max_slope_deg = 0.0

        self.get_logger().info(
            f'pos=({self.last_x:+5.2f},{self.last_y:+5.2f}) {cell_str} | '
            f'pitch={math.degrees(self.last_pitch):+5.1f}° | '
            f'visited={visited}/10000 | '
            f'max_slope={max_slope_deg:5.1f}° | '
            f'rates: imu={self.imu_count} odom={self.odom_count}'
        )
        self.imu_count = 0
        self.odom_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = TerrainHeatmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()