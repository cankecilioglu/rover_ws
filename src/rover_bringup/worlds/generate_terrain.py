#!/usr/bin/env python3
"""
Prosedürel arazi üretir (vertex normals dahil).
- terrain.obj: dartsim/bullet uyumlu mesh
- heightmap.png: görsel referans
"""
import numpy as np
from PIL import Image
from pathlib import Path

RESOLUTION = 256
EXTENT = 20.0
MAX_HEIGHT = 0.15

# Heightfield
x_lin = np.linspace(-EXTENT/2, EXTENT/2, RESOLUTION)
y_lin = np.linspace(-EXTENT/2, EXTENT/2, RESOLUTION)
X, Y = np.meshgrid(x_lin, y_lin)

large = 0.55 * np.sin(X * 0.5) * np.cos(Y * 0.4)
medium = 0.30 * np.sin(X * 1.5 + Y * 0.7) * np.cos(Y * 1.2)
small = 0.10 * np.sin(X * 4.0) * np.sin(Y * 3.5)
np.random.seed(42)
noise = 0.05 * np.random.randn(RESOLUTION, RESOLUTION)

Z = large + medium + small + noise
Z = (Z - Z.min()) / (Z.max() - Z.min())
Z = Z * MAX_HEIGHT

# Vertex normals — heightfield gradient'inden
# Normal = normalize(-dz/dx, -dz/dy, 1)
spacing = EXTENT / (RESOLUTION - 1)
dz_dy, dz_dx = np.gradient(Z, spacing)
nx = -dz_dx
ny = -dz_dy
nz = np.ones_like(Z)

n_mag = np.sqrt(nx**2 + ny**2 + nz**2)
nx /= n_mag
ny /= n_mag
nz /= n_mag

# OBJ yaz
out_dir = Path(__file__).parent
obj_path = out_dir / 'terrain.obj'
with open(obj_path, 'w') as f:
    f.write("# Procedural terrain mesh with normals\n")
    f.write(f"# {RESOLUTION*RESOLUTION} vertices, {(RESOLUTION-1)**2 * 2} triangles\n\n")

    # Vertices
    for i in range(RESOLUTION):
        for j in range(RESOLUTION):
            f.write(f"v {X[i,j]:.4f} {Y[i,j]:.4f} {Z[i,j]:.4f}\n")

    # Vertex normals (aynı sırayla)
    for i in range(RESOLUTION):
        for j in range(RESOLUTION):
            f.write(f"vn {nx[i,j]:.4f} {ny[i,j]:.4f} {nz[i,j]:.4f}\n")

    # Faces: f v//vn v//vn v//vn (OBJ 1-indexed)
    for i in range(RESOLUTION - 1):
        for j in range(RESOLUTION - 1):
            v1 = i * RESOLUTION + j + 1
            v2 = v1 + 1
            v3 = v1 + RESOLUTION
            v4 = v3 + 1
            f.write(f"f {v1}//{v1} {v3}//{v3} {v2}//{v2}\n")
            f.write(f"f {v2}//{v2} {v3}//{v3} {v4}//{v4}\n")

print(f"Mesh saved: {obj_path}")
print(f"  Vertices: {RESOLUTION*RESOLUTION}")
print(f"  Normals:  {RESOLUTION*RESOLUTION}")
print(f"  Triangles: {(RESOLUTION-1)**2 * 2}")

# Görsel referans PNG
png = (Z / MAX_HEIGHT * 255).astype(np.uint8)
Image.fromarray(png, mode='L').save(out_dir / 'heightmap.png')