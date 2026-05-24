#!/usr/bin/env python3
"""
Proprioceptive terrain heatmap node.

Aşama 6.4 polish:
- Cell başına MAX değer (ortalama yerine) — kısa spike'lar dilüe olmaz
- MAX_SLOPE = 25° (geniş dinamik range: hill ~46%, bump spike ~%80-100)
- MAX_ROUGHNESS = 5.0 m/s² (bump kenar spike'larına yer)
- Roughness: IIR high-pass (alpha=0.95) — statik tilt filtreli
- Outlier rejection: 40° üstü slope, 10 m/s² üstü roughness noise sayılır
- Grid 200×200 (40m × 40m), ground truth odometry kullanır

Yayınlar:
- /terrain/slope_grid       (eğim haritası, OccupancyGrid)
- /terrain/roughness_grid   (pürüzlülük haritası, OccupancyGrid)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid


class TerrainHeatmapNode(Node):
    # Grid parametreleri
    GRID_SIZE = 200                       # 200×200 hücre
    CELL_SIZE = 0.20                      # 20 cm / hücre
    MAP_EXTENT = GRID_SIZE * CELL_SIZE    # 40 m × 40 m alan
    MAP_ORIGIN_X = -MAP_EXTENT / 2
    MAP_ORIGIN_Y = -MAP_EXTENT / 2

    # Normalizasyon (heatmap renk skalası)
    MAX_SLOPE_RAD = math.radians(25.0)    # 25° = tam doyumlu
    MAX_ROUGHNESS = 5.0                   # m/s² high-pass tepe değeri = tam doyumlu

    # Outlier rejection (fiziksel olmayan değerleri yoksay)
    SLOPE_OUTLIER_RAD = math.radians(40.0)   # 40° üstü → noise
    ROUGHNESS_OUTLIER = 10.0                  # 10 m/s² üstü → noise

    # IIR low-pass filter (gravity projection takibi için)
    # alpha = 0.95, 100Hz IMU'da ~0.8 Hz cutoff
    LOWPASS_ALPHA = 0.95

    def __init__(self):
        super().__init__('terrain_heatmap_node')

        # Grid arrays — her cell'de görülen MAX değer tutulur
        self.slope_max   = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)
        self.rough_max   = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)
        self.visit_count = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.int32)

        # En son okumalar
        self.last_pitch = 0.0
        self.last_roll  = 0.0
        self.last_az    = 9.81
        self.last_x = 0.0
        self.last_y = 0.0
        self.imu_count  = 0
        self.odom_count = 0

        # High-pass filter state
        self.az_lowpass  = 9.81           # filter başlangıcı = gravity
        self.az_highpass = 0.0            # hızlı titreşim bileşeni

        # Outlier sayacı (diagnostic)
        self.slope_outliers = 0
        self.rough_outliers = 0

        # Subscribers
        self.create_subscription(Imu, '/imu', self.imu_cb, 100)
        self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_cb, 50)

        # Publishers — heatmap grids
        self.slope_pub = self.create_publisher(OccupancyGrid, '/terrain/slope_grid', 10)
        self.rough_pub = self.create_publisher(OccupancyGrid, '/terrain/roughness_grid', 10)

        # Timers
        self.create_timer(1.0, self.print_status)        # 1Hz log
        self.create_timer(0.2, self.publish_grids)       # 5Hz publish

        total_cells = self.GRID_SIZE * self.GRID_SIZE
        self.get_logger().info(
            f'Heatmap node started. Grid: {self.GRID_SIZE}×{self.GRID_SIZE} = {total_cells} cells '
            f'({self.CELL_SIZE*100:.0f}cm/cell, {self.MAP_EXTENT:.0f}m × {self.MAP_EXTENT:.0f}m)'
        )
        self.get_logger().info(
            f'Mode: max-per-cell | Color scale: slope max = {math.degrees(self.MAX_SLOPE_RAD):.0f}°, '
            f'roughness max = {self.MAX_ROUGHNESS} m/s²'
        )
        self.get_logger().info(
            f'Outlier rejection: slope > {math.degrees(self.SLOPE_OUTLIER_RAD):.0f}°, '
            f'roughness > {self.ROUGHNESS_OUTLIER} m/s²'
        )
        self.get_logger().info(
            'Publishing: /terrain/slope_grid, /terrain/roughness_grid @ 5Hz'
        )

    @staticmethod
    def quat_to_euler(qx, qy, qz, qw):
        """Quaternion (x, y, z, w) → (roll, pitch, yaw) radyan."""
        roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
        yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        return roll, pitch, yaw

    def world_to_cell(self, x, y):
        """Dünya koordinatlarını grid satır/sütununa çevir; sınır dışı için None."""
        col = int((x - self.MAP_ORIGIN_X) / self.CELL_SIZE)
        row = int((y - self.MAP_ORIGIN_Y) / self.CELL_SIZE)
        if 0 <= row < self.GRID_SIZE and 0 <= col < self.GRID_SIZE:
            return row, col
        return None

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        self.last_roll, self.last_pitch, _ = self.quat_to_euler(q.x, q.y, q.z, q.w)

        az = msg.linear_acceleration.z

        # Low-pass: gravity projection'u yavaşça izle (statik bileşen)
        self.az_lowpass = (self.LOWPASS_ALPHA * self.az_lowpass
                           + (1 - self.LOWPASS_ALPHA) * az)
        # High-pass = orijinal - low-pass → sadece hızlı titreşimler
        self.az_highpass = az - self.az_lowpass

        self.last_az = az
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

        # Eğim büyüklüğü: pitch ve roll kombinasyonu
        slope = math.sqrt(self.last_pitch**2 + self.last_roll**2)

        # Pürüzlülük: high-pass az'nin mutlak değeri (m/s²)
        roughness = abs(self.az_highpass)

        # Outlier rejection — fiziksel olmayan değerleri sayma
        if slope > self.SLOPE_OUTLIER_RAD:
            self.slope_outliers += 1
        else:
            # Max-per-cell: bu hücrede gördüğümüz en büyük slope'u tut
            if slope > self.slope_max[row, col]:
                self.slope_max[row, col] = slope

        if roughness > self.ROUGHNESS_OUTLIER:
            self.rough_outliers += 1
        else:
            if roughness > self.rough_max[row, col]:
                self.rough_max[row, col] = roughness

        self.visit_count[row, col] += 1

    def _array_to_occupancy(self, arr, max_val):
        """Float array → int8 (0-100); ziyaret edilmemiş hücreler -1 (unknown)."""
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

        # Max değerler doğrudan kullanılır (ortalama bölmesi yok)
        slope_data = self._array_to_occupancy(self.slope_max, self.MAX_SLOPE_RAD)
        rough_data = self._array_to_occupancy(self.rough_max, self.MAX_ROUGHNESS)

        self.slope_pub.publish(self._make_grid_msg(slope_data, now))
        self.rough_pub.publish(self._make_grid_msg(rough_data, now))

    def print_status(self):
        cell = self.world_to_cell(self.last_x, self.last_y)
        cell_str = f'cell=({cell[0]:3d},{cell[1]:3d})' if cell else 'cell=OUT     '

        total_cells = self.GRID_SIZE * self.GRID_SIZE
        visited = int((self.visit_count > 0).sum())

        if self.visit_count.sum() > 0:
            max_slope_deg = math.degrees(float(self.slope_max.max()))
            max_rough = float(self.rough_max.max())
        else:
            max_slope_deg = 0.0
            max_rough = 0.0

        self.get_logger().info(
            f'pos=({self.last_x:+6.2f},{self.last_y:+6.2f}) {cell_str} | '
            f'pitch={math.degrees(self.last_pitch):+5.1f}° '
            f'az_hp={self.az_highpass:+5.2f} | '
            f'visited={visited}/{total_cells} | '
            f'peak: slope={max_slope_deg:4.1f}° rough={max_rough:.2f} | '
            f'outliers: s={self.slope_outliers} r={self.rough_outliers} | '
            f'rates: imu={self.imu_count} odom={self.odom_count}'
        )
        self.imu_count = 0
        self.odom_count = 0
        # Outlier sayaçlarını sıfırlamıyoruz, kümülatif (debug için kullanışlı)


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