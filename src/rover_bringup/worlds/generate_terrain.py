#!/usr/bin/env python3
"""
Prosedürel arazi üretir.
- terrain.png: Gazebo heightmap (16-bit grayscale, 257×257)
- terrain.obj: Mesh alternatif (vertex normals dahil)

Spawn alanı (origin etrafında) HER ZAMAN z=0'da — blend mask
ile garantili. Max slope ~25-30° hedefli, smooth noise kullanılır.
"""
import numpy as np
from PIL import Image
from pathlib import Path
from scipy.ndimage import gaussian_filter

# Gazebo heightmap requirement: 2^n + 1
RESOLUTION = 257
EXTENT = 20.0                    # 20m × 20m alan
MAX_HEIGHT = 0.4                 # Toplam yükseklik (m)
SPAWN_FLAT_RADIUS = 2.0          # Düz spawn alanı yarıçapı (m)
SPAWN_BLEND_WIDTH = 1.5          # Blend transition genişliği (m)

# Heightfield grid
x_lin = np.linspace(-EXTENT/2, EXTENT/2, RESOLUTION)
y_lin = np.linspace(-EXTENT/2, EXTENT/2, RESOLUTION)
X, Y = np.meshgrid(x_lin, y_lin)

# Çok ölçekli dalga (smoother frequencies, max slope kontrol altında)
large  = 0.55 * np.sin(X * 0.5) * np.cos(Y * 0.4)        # max grad 0.275 m/m
medium = 0.25 * np.sin(X * 0.8 + Y * 0.5)                 # max grad 0.20 m/m
small  = 0.10 * np.sin(X * 1.5) * np.sin(Y * 1.5)         # max grad 0.15 m/m

np.random.seed(42)
noise_raw = 0.04 * np.random.randn(RESOLUTION, RESOLUTION)
noise = gaussian_filter(noise_raw, sigma=2.0)             # smooth, gradient kontrol

Z_raw = large + medium + small + noise

# CRITICAL: spawn zone garantili 0 olsun
# 1) Önce min'i 0'a kaydır (tüm değerler pozitif)
Z_raw = Z_raw - Z_raw.min()

# 2) Spawn zone blend mask (0 at center, smoothstep to 1)
R = np.sqrt(X**2 + Y**2)
blend = np.clip((R - SPAWN_FLAT_RADIUS) / SPAWN_BLEND_WIDTH, 0.0, 1.0)
blend = blend * blend * (3 - 2 * blend)  # smoothstep

# 3) Blend uygula — spawn = 0 exactly, periphery scales
Z = Z_raw * blend

# 4) Normalize to [0, MAX_HEIGHT] (min zaten 0 spawn'da)
Z = Z / Z.max() * MAX_HEIGHT

# === Diagnostic ===
spacing = EXTENT / (RESOLUTION - 1)
dz_dy, dz_dx = np.gradient(Z, spacing)
slope_rad = np.arctan(np.sqrt(dz_dx**2 + dz_dy**2))
max_slope_deg = np.degrees(slope_rad.max())
mean_slope_deg = np.degrees(slope_rad.mean())

# Origin'deki yükseklik
center = RESOLUTION // 2
spawn_height_mm = Z[center, center] * 1000

print(f"Terrain stats:")
print(f"  Grid:        {RESOLUTION}×{RESOLUTION} ({EXTENT}m × {EXTENT}m, spacing={spacing*100:.1f}cm)")
print(f"  Height:      min={Z.min():.3f}m, max={Z.max():.3f}m")
print(f"  Slope:       max={max_slope_deg:.1f}°, mean={mean_slope_deg:.1f}°")
print(f"  Spawn zone:  radius={SPAWN_FLAT_RADIUS}m, height@origin={spawn_height_mm:.2f}mm")

# === Heightmap PNG (Gazebo formatı) ===
out_dir = Path(__file__).parent
png_path = out_dir / 'terrain.png'

png_16bit = (Z / MAX_HEIGHT * 65535).astype(np.uint16)
Image.fromarray(png_16bit, mode='I;16').save(png_path)
print(f"\nHeightmap saved: {png_path} (16-bit, {png_16bit.shape})")

# === Mesh OBJ (alternatif) ===
nx = -dz_dx
ny = -dz_dy
nz = np.ones_like(Z)
n_mag = np.sqrt(nx**2 + ny**2 + nz**2)
nx /= n_mag
ny /= n_mag
nz /= n_mag

obj_path = out_dir / 'terrain.obj'
with open(obj_path, 'w') as f:
    f.write("# Procedural terrain mesh with normals\n")
    f.write(f"# {RESOLUTION*RESOLUTION} vertices, {(RESOLUTION-1)**2 * 2} triangles\n\n")
    for i in range(RESOLUTION):
        for j in range(RESOLUTION):
            f.write(f"v {X[i,j]:.4f} {Y[i,j]:.4f} {Z[i,j]:.4f}\n")
    for i in range(RESOLUTION):
        for j in range(RESOLUTION):
            f.write(f"vn {nx[i,j]:.4f} {ny[i,j]:.4f} {nz[i,j]:.4f}\n")
    for i in range(RESOLUTION - 1):
        for j in range(RESOLUTION - 1):
            v1 = i * RESOLUTION + j + 1
            v2 = v1 + 1
            v3 = v1 + RESOLUTION
            v4 = v3 + 1
            f.write(f"f {v1}//{v1} {v2}//{v2} {v3}//{v3}\n")
            f.write(f"f {v2}//{v2} {v4}//{v4} {v3}//{v3}\n")

print(f"Mesh saved:      {obj_path}")
print(f"  Vertices: {RESOLUTION*RESOLUTION}, Triangles: {(RESOLUTION-1)**2 * 2}")