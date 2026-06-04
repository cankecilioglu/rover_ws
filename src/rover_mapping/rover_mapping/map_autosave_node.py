#!/usr/bin/env python3
import os
import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

# Heatmap node ile AYNI olmali (occupancy 0-100 -> fiziksel birim cevrimi icin)
MAX_SLOPE_DEG = 25.0      # occupancy 100 = bu aci
MAX_ROUGHNESS = 5.0       # occupancy 100 = bu deger (m/s^2)


class MapAutosave(Node):
    def __init__(self):
        super().__init__('map_autosave_node')
        self.declare_parameter('map_dir', os.path.expanduser('~/rover_maps'))
        self.map_dir = self.get_parameter('map_dir').value

        self.slope_msg = None
        self.rough_msg = None

        self.create_subscription(OccupancyGrid, '/terrain/slope_grid', self.slope_cb, 1)
        self.create_subscription(OccupancyGrid, '/terrain/roughness_grid', self.rough_cb, 1)

        self.srv = self.create_service(Trigger, 'save_map', self.on_save_request)

        self.get_logger().info(
            f'Map autosave hazir. Dizin: {self.map_dir} | '
            f'manuel: ros2 service call /save_map std_srvs/srv/Trigger "{{}}"')

    def slope_cb(self, msg):
        self.slope_msg = msg

    def rough_cb(self, msg):
        self.rough_msg = msg

    def on_save_request(self, request, response):
        path = self.save_all('manual')
        response.success = path is not None
        response.message = f'Kaydedildi: {path}' if path else 'Kaydedilecek harita yok'
        return response

    def _grid_to_array(self, msg):
        return np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width))  # -1 unknown, 0..100 deger

    def _save_one(self, msg, name, out_dir, max_value, unit_label):
        arr = self._grid_to_array(msg)
        np.save(os.path.join(out_dir, f'{name}.npy'), arr)  # raw (analiz/reload)
        info = msg.info
        with open(os.path.join(out_dir, f'{name}.yaml'), 'w') as f:
            f.write(f'resolution: {info.resolution}\n')
            f.write(f'width: {info.width}\n')
            f.write(f'height: {info.height}\n')
            f.write(f'origin: [{info.origin.position.x}, '
                    f'{info.origin.position.y}, {info.origin.position.z}]\n')
            f.write(f'frame_id: {msg.header.frame_id}\n')
            f.write(f'max_value: {max_value}\n')
            f.write(f'unit: "{unit_label}"\n')
        self._save_png(arr, os.path.join(out_dir, f'{name}.png'),
                       info, max_value, unit_label, name)

    def _save_png(self, arr, path, info, max_value, unit_label, title):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().warn(f'matplotlib yok, PNG atlandi ({e}). pip install matplotlib')
            return
        phys = arr.astype(float)
        phys[arr < 0] = np.nan                 # ziyaret edilmemis -> seffaf
        phys = phys / 100.0 * max_value         # occupancy -> fiziksel birim
        ox, oy = info.origin.position.x, info.origin.position.y
        extent = [ox, ox + info.width * info.resolution,
                  oy, oy + info.height * info.resolution]
        fig, ax = plt.subplots(figsize=(8, 8))
        cmap = plt.cm.turbo.copy()
        cmap.set_bad(color='#202020')           # bos hucreler koyu gri
        im = ax.imshow(phys, origin='lower', extent=extent,
                       cmap=cmap, vmin=0, vmax=max_value)
        ax.set_title(title)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        fig.colorbar(im, ax=ax, shrink=0.8).set_label(unit_label)
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)

    def save_all(self, tag):
        if self.slope_msg is None and self.rough_msg is None:
            self.get_logger().warn('Henuz harita gelmedi, kayit yok.')
            return None
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(self.map_dir, f'{stamp}_{tag}')
        os.makedirs(out_dir, exist_ok=True)
        if self.slope_msg is not None:
            self._save_one(self.slope_msg, 'slope', out_dir, MAX_SLOPE_DEG, 'slope (deg)')
        if self.rough_msg is not None:
            self._save_one(self.rough_msg, 'roughness', out_dir, MAX_ROUGHNESS, 'roughness (m/s^2)')
        self.get_logger().info(f'>>> Harita kaydedildi: {out_dir}')
        return out_dir


def main():
    rclpy.init()
    node = MapAutosave()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.save_all('shutdown')   # surus bitince (Ctrl-C) otomatik kayit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()