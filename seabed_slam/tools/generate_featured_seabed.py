#!/usr/bin/env python3
from pathlib import Path
import os

import numpy as np
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
from scipy.spatial import Delaunay


ROOT = Path('/home/fabian/ros2_ws')
CSV_PATH = ROOT / 'src/seabed_slam/data/missouri_i64/site-23_MissouriRiver_I-64_2020-08_CUBE-uncert.csv'
MESH_DIR = ROOT / 'src/seabed_slam/models/missouri_i64_featured/meshes'
WORLD_DIR = ROOT / 'src/seabed_slam/worlds'
META_PATH = ROOT / 'src/seabed_slam/data/missouri_i64/missouri_i64_featured_crop.txt'

GRID_RESOLUTION_M = float(os.environ.get('SEABED_GRID_RESOLUTION_M', '1.0'))


def load_points():
    data = np.genfromtxt(CSV_PATH, delimiter=',', names=True, dtype=float, encoding=None)
    x = data['X']
    y = data['Y']
    z = data['Z']
    mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[mask], y[mask], z[mask]


def choose_featured_crop(x, y, z, window_x=100.0, window_y=72.0, stride=20.0):
    xmin, xmax = float(x.min()), float(x.max())
    ymin, ymax = float(y.min()), float(y.max())
    best = None

    for cx in np.arange(xmin + window_x / 2.0, xmax - window_x / 2.0, stride):
        mx = np.abs(x - cx) <= window_x / 2.0
        if mx.sum() < 5000:
            continue
        for cy in np.arange(ymin + window_y / 2.0, ymax - window_y / 2.0, stride):
            m = mx & (np.abs(y - cy) <= window_y / 2.0)
            n = int(m.sum())
            if n < 10000:
                continue
            zz = z[m]
            relief = float(zz.max() - zz.min())
            std = float(zz.std())
            pspread = float(np.percentile(zz, 95) - np.percentile(zz, 5))
            score = relief * 0.55 + std * 1.5 + pspread * 0.35 + min(n, 50000) / 50000.0
            cand = (score, cx, cy, n, relief, std, pspread)
            if best is None or cand > best:
                best = cand

    if best is None:
        raise RuntimeError('No dense crop found')
    return best, window_x, window_y


def downsample(points, cell=0.75):
    ix = np.floor((points[:, 0] - points[:, 0].min()) / cell).astype(np.int64)
    iy = np.floor((points[:, 1] - points[:, 1].min()) / cell).astype(np.int64)
    keys = ix * 1_000_000 + iy
    order = np.lexsort((points[:, 2], keys))
    points = points[order]
    keys = keys[order]
    starts = np.r_[0, np.flatnonzero(keys[1:] != keys[:-1]) + 1]

    reduced = []
    for a, b in zip(starts, list(starts[1:]) + [len(points)]):
        reduced.append(points[a:b].mean(axis=0))
    return np.array(reduced)


def triangulate(points):
    px = points[:, 0] - points[:, 0].mean()
    py = points[:, 1] - points[:, 1].mean()
    pz = points[:, 2] - points[:, 2].max() - 0.5
    points2 = np.column_stack([px, py])
    simplices = Delaunay(points2).simplices

    keep = []
    for tri in simplices:
        p = points2[tri]
        edges = (
            np.linalg.norm(p[0] - p[1]),
            np.linalg.norm(p[1] - p[2]),
            np.linalg.norm(p[2] - p[0]),
        )
        if max(edges) <= 2.5:
            keep.append(tri)

    return list(zip(px, py, pz)), np.array(keep, dtype=np.int64)


def regular_grid_mesh(points, resolution=0.5):
    x_min, x_max = float(points[:, 0].min()), float(points[:, 0].max())
    y_min, y_max = float(points[:, 1].min()), float(points[:, 1].max())
    gx = np.arange(x_min, x_max + resolution * 0.5, resolution)
    gy = np.arange(y_min, y_max + resolution * 0.5, resolution)
    xx, yy = np.meshgrid(gx, gy)

    source_xy = points[:, :2]
    linear = LinearNDInterpolator(source_xy, points[:, 2])
    nearest = NearestNDInterpolator(source_xy, points[:, 2])
    zz = linear(xx, yy)
    missing = ~np.isfinite(zz)
    if np.any(missing):
        zz[missing] = nearest(xx[missing], yy[missing])

    px = xx - (x_min + x_max) / 2.0
    py = yy - (y_min + y_max) / 2.0
    pz = zz - float(np.max(zz)) - 0.5

    vertices = list(zip(px.ravel(), py.ravel(), pz.ravel()))
    width = len(gx)
    height = len(gy)
    triangles = []
    for row in range(height - 1):
        for col in range(width - 1):
            i00 = row * width + col
            i10 = row * width + col + 1
            i01 = (row + 1) * width + col
            i11 = (row + 1) * width + col + 1
            triangles.append((i00, i10, i11))
            triangles.append((i00, i11, i01))

    return vertices, np.array(triangles, dtype=np.int64), resolution, int(np.sum(missing))

def compute_vertex_normals(vertices, triangles):
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int64).copy()

    # Face normals
    face_normals = np.cross(
        vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
        vertices[triangles[:, 2]] - vertices[triangles[:, 0]],
    )

    # Since this is a heightfield seabed, normals should point mostly upward.
    # Flip any downward-facing triangles.
    downward = face_normals[:, 2] < 0.0
    if np.any(downward):
        triangles[downward] = triangles[downward][:, [0, 2, 1]]

        face_normals = np.cross(
            vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
            vertices[triangles[:, 2]] - vertices[triangles[:, 0]],
        )

    # Area-weighted vertex normals
    vertex_normals = np.zeros_like(vertices)

    np.add.at(vertex_normals, triangles[:, 0], face_normals)
    np.add.at(vertex_normals, triangles[:, 1], face_normals)
    np.add.at(vertex_normals, triangles[:, 2], face_normals)

    lengths = np.linalg.norm(vertex_normals, axis=1)
    valid = lengths > 1e-12

    vertex_normals[valid] /= lengths[valid, None]
    vertex_normals[~valid] = np.array([0.0, 0.0, 1.0])

    return triangles, vertex_normals

def write_dae(mesh_path, vertices, triangles):
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles, vertex_normals = compute_vertex_normals(vertices, triangles)

    positions = ' '.join(
        f'{x:.6f} {y:.6f} {z:.6f}'
        for x, y, z in vertices
    )

    normals = ' '.join(
        f'{nx:.6f} {ny:.6f} {nz:.6f}'
        for nx, ny, nz in vertex_normals
    )

    # Collada index stream:
    # offset 0 = vertex index
    # offset 1 = normal index
    #
    # Here normal index == vertex index.
    vertex_normal_indices = ' '.join(
        f'{int(i)} {int(i)}'
        for tri in triangles
        for i in tri
    )

    mesh_path.write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>generated from USGS Missouri River I-64 CUBE bathymetry</authoring_tool>
    </contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>

  <library_effects>
    <effect id="seabed_effect">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <ambient>
              <color>0.35 0.35 0.35 1</color>
            </ambient>
            <diffuse>
              <color>0.62 0.62 0.62 1</color>
            </diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>

  <library_materials>
    <material id="seabed_material" name="seabed_material">
      <instance_effect url="#seabed_effect"/>
    </material>
  </library_materials>

  <library_geometries>
    <geometry id="seabed_mesh" name="MissouriI64FeaturedSeabed">
      <mesh>
        <source id="seabed_positions">
          <float_array id="seabed_positions_array" count="{len(vertices) * 3}">
            {positions}
          </float_array>
          <technique_common>
            <accessor source="#seabed_positions_array" count="{len(vertices)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <source id="seabed_normals">
          <float_array id="seabed_normals_array" count="{len(vertex_normals) * 3}">
            {normals}
          </float_array>
          <technique_common>
            <accessor source="#seabed_normals_array" count="{len(vertex_normals)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <vertices id="seabed_vertices">
          <input semantic="POSITION" source="#seabed_positions"/>
        </vertices>

        <triangles material="seabed_material" count="{len(triangles)}">
          <input semantic="VERTEX" source="#seabed_vertices" offset="0"/>
          <input semantic="NORMAL" source="#seabed_normals" offset="1"/>
          <p>{vertex_normal_indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>

  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="seabed_node" name="seabed">
        <instance_geometry url="#seabed_mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="seabed_material" target="#seabed_material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>

  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
''')


def write_world(world_path, mesh_path):
    mesh_uri = 'file://' + str(mesh_path)
    mesh_scale = '0.5 0.5 0.5'
    world_path.write_text(f'''<sdf version="1.7">
  <world name="missouri_i64_featured_seabed">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>-90 -130 85 0 0 0</pose>
      <diffuse>1.0 0.98 0.90 1</diffuse>
      <specular>0.08 0.08 0.08 1</specular>
      <attenuation><range>600</range><constant>0.75</constant><linear>0.01</linear><quadratic>0.0005</quadratic></attenuation>
      <direction>0.78 0.50 -0.38</direction>
    </light>
    <light name="soft_fill" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>80 90 80 0 0 0</pose>
      <diffuse>0.18 0.18 0.18 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <direction>-0.45 -0.55 -0.70</direction>
    </light>
    <model name="missouri_i64_featured_seabed">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="seabed_link">
        <collision name="seabed_collision"><geometry><mesh><uri>{mesh_uri}</uri><scale>{mesh_scale}</scale></mesh></geometry></collision>
        <visual name="seabed_visual"><cast_shadows>true</cast_shadows><geometry><mesh><uri>{mesh_uri}</uri><scale>{mesh_scale}</scale></mesh></geometry><material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.72 0.72 0.72 1</diffuse><specular>0.03 0.03 0.03 1</specular></material></visual>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.30 0.30 0.30 1</background><shadows>1</shadows><grid>false</grid></scene>
    <gui fullscreen="0"><camera name="user_camera"><pose>0 -150 80 0 0.55 1.5708</pose><view_controller>orbit</view_controller></camera></gui>
  </world>
</sdf>
''')


def main():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    WORLD_DIR.mkdir(parents=True, exist_ok=True)

    x, y, z = load_points()
    best, window_x, window_y = choose_featured_crop(x, y, z)
    score, cx, cy, n, relief, std, pspread = best
    crop = (np.abs(x - cx) <= window_x / 2.0) & (np.abs(y - cy) <= window_y / 2.0)
    points = np.column_stack([x[crop], y[crop], z[crop]])
    crop_relief = float(points[:, 2].max() - points[:, 2].min())
    crop_std = float(points[:, 2].std())
    crop_pspread = float(np.percentile(points[:, 2], 95) - np.percentile(points[:, 2], 5))
    vertices, triangles, grid_resolution, interpolated_edge_points = regular_grid_mesh(
        points,
        resolution=GRID_RESOLUTION_M,
    )

    mesh_path = MESH_DIR / 'missouri_i64_featured.dae'
    world_path = WORLD_DIR / 'missouri_i64_featured_seabed.world'
    write_dae(mesh_path, vertices, triangles)
    write_world(world_path, mesh_path)

    xs = [p[0] for p in vertices]
    ys = [p[1] for p in vertices]
    zs = [p[2] for p in vertices]
    META_PATH.write_text(
        f'source_csv={CSV_PATH}\\n'
        f'center_x={cx}\\ncenter_y={cy}\\nwindow_x_m={window_x}\\nwindow_y_m={window_y}\\n'
        f'source_points={len(points)}\\n'
        f'grid_resolution_m={grid_resolution}\\n'
        f'interpolated_or_edge_filled_points={interpolated_edge_points}\\n'
        f'grid_points={len(vertices)}\\ntriangles={len(triangles)}\\n'
        f'automatic_relief_m={relief}\\nautomatic_z_std_m={std}\\nautomatic_percentile_spread_m={pspread}\\n'
        f'relief_m={crop_relief}\\nz_std_m={crop_std}\\npercentile_spread_m={crop_pspread}\\n'
        f'extent_x_m={max(xs) - min(xs)}\\nextent_y_m={max(ys) - min(ys)}\\ndepth_m={-min(zs)}\\n'
        f'mesh={mesh_path}\\nworld={world_path}\\n'
    )
    print(META_PATH.read_text())


if __name__ == '__main__':
    main()
