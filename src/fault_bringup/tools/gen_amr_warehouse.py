#!/usr/bin/env python3
"""Generate worlds/amr_warehouse.world: a 14 m x 10 m indoor logistics (AMR) warehouse.

Only Gazebo primitives (box) + inline colors -> no model downloads, fully reproducible.
Re-run after editing:  python3 src/fault_bringup/tools/gen_amr_warehouse.py

Frame: x = east, y = north, origin = central intersection.
  outer walls         x in [-7, 7], y in [-5, 5]
  main aisle (N-S)    x in [-1, 1]            Start zone south, Delivery zone north
  cross aisle (E-W)   y in [-1, 1]            Charging / parking bays at the west end
  storage quadrants   2 shelf rows each (4.0 m x 0.6 m), 1.0 m aisle between rows,
                      1.0 m passages to the main aisles and to the walls (no dead ends)
  pallets             in the cross aisle and the north main aisle (obstacle-avoidance spots)
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'worlds', 'amr_warehouse.world')

C = {  # r g b
    'floor_aisle': (0.62, 0.63, 0.65), 'floor_storage': (0.42, 0.43, 0.46),
    'wall': (0.80, 0.78, 0.72), 'wall_base': (0.30, 0.32, 0.36),
    'yellow': (0.95, 0.78, 0.05), 'white': (0.95, 0.95, 0.95),
    'start': (0.15, 0.65, 0.25), 'goal': (0.15, 0.40, 0.85), 'charge': (0.10, 0.55, 0.55),
    'rack_blue': (0.10, 0.25, 0.60), 'rack_orange': (0.95, 0.45, 0.05),
    'carton': (0.72, 0.52, 0.30), 'carton_dark': (0.58, 0.40, 0.22), 'wrap': (0.80, 0.85, 0.88),
    'pallet': (0.55, 0.38, 0.20), 'dock': (0.20, 0.22, 0.25), 'dock_light': (0.20, 0.95, 0.35),
}
links = []


def box(name, x, y, z, sx, sy, sz, color, collide=True):
    r, g, b = C[color]
    coll = (f'<collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'
            f'</geometry></collision>') if collide else ''
    links.append(
        f'<link name="{name}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>{coll}'
        f'<visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
        f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
        f'<specular>0.1 0.1 0.1 1</specular></material></visual></link>')


def floor_patch(name, x0, x1, y0, y1, color, z=0.001):
    """Visual-only floor paint (no collision, so it never disturbs the wheels)."""
    box(name, (x0 + x1) / 2, (y0 + y1) / 2, z, x1 - x0, y1 - y0, 0.002, color, collide=False)


def line(name, x0, y0, x1, y1, color='yellow', w=0.06):
    if x0 == x1:
        floor_patch(name, x0 - w / 2, x0 + w / 2, min(y0, y1), max(y0, y1), color, z=0.003)
    else:
        floor_patch(name, min(x0, x1), max(x0, x1), y0 - w / 2, y0 + w / 2, color, z=0.003)


def shelf(name, cx, cy, length=4.0, depth=0.6, height=1.6, seed=0):
    # solid base (0.3 m) so the LiDAR (~0.18 m) sees the rack and the robot cannot go under it
    box(f'{name}_base', cx, cy, 0.15, length, depth, 0.30, 'rack_blue')
    for i, px in enumerate((-length / 2 + 0.04, 0.0, length / 2 - 0.04)):
        for j, py in enumerate((-depth / 2 + 0.04, depth / 2 - 0.04)):
            box(f'{name}_post{i}{j}', cx + px, cy + py, height / 2, 0.08, 0.08, height, 'rack_blue')
    for k, z in enumerate((0.85, 1.40)):
        box(f'{name}_beam{k}', cx, cy, z, length, depth, 0.05, 'rack_orange')
    # cartons on the base and both levels (deterministic pseudo-random sizes)
    for k, z in enumerate((0.30, 0.875, 1.425)):
        x = cx - length / 2 + 0.15
        n = 0
        while True:
            h = (seed * 7 + k * 13 + n * 5) % 5
            w = 0.45 + 0.08 * h
            if x + w > cx + length / 2 - 0.1:
                break
            bh = 0.30 + 0.04 * ((h + n) % 4)
            box(f'{name}_carton{k}_{n}', x + w / 2, cy, z + bh / 2, w, depth - 0.12, bh,
                'carton' if (n + k + seed) % 3 else 'carton_dark', collide=False)
            x += w + 0.08
            n += 1


def pallet(name, cx, cy, sx=1.0, sy=0.8, layers=2):
    box(f'{name}_deck', cx, cy, 0.075, sx, sy, 0.15, 'pallet')
    for i in range(layers):
        box(f'{name}_load{i}', cx, cy, 0.15 + 0.25 * i + 0.125, sx - 0.1, sy - 0.1, 0.25,
            'carton' if i % 2 == 0 else 'wrap')


# ---- floor zones --------------------------------------------------------------------------
floor_patch('floor_main_ns', -1, 1, -5, 5, 'floor_aisle')
floor_patch('floor_cross_ew', -7, 7, -1, 1, 'floor_aisle')
for qx in (-1, 1):
    for qy in (-1, 1):
        x0, x1 = sorted((qx * 1, qx * 7))
        y0, y1 = sorted((qy * 1, qy * 5))
        floor_patch(f'floor_storage_{qx}_{qy}', x0, x1, y0, y1, 'floor_storage', z=0.0005)
floor_patch('zone_start', -0.8, 0.8, -4.9, -2.9, 'start', z=0.002)
floor_patch('zone_delivery', -0.8, 0.8, 3.1, 4.9, 'goal', z=0.002)
floor_patch('zone_charge', -6.95, -5.2, -0.95, 0.95, 'charge', z=0.002)

# ---- floor markings: aisle edges (broken at the intersection) + bay lines ------------------
for sgn in (-1, 1):
    line(f'mark_ns_s_{sgn}', sgn * 1.0, -5, sgn * 1.0, -1)
    line(f'mark_ns_n_{sgn}', sgn * 1.0, 1, sgn * 1.0, 5)
    line(f'mark_ew_w_{sgn}', -7, sgn * 1.0, -1, sgn * 1.0)
    line(f'mark_ew_e_{sgn}', 1, sgn * 1.0, 7, sgn * 1.0)
for i, y in enumerate((-0.95, 0.0, 0.95)):
    line(f'mark_bay_{i}', -6.95, y, -5.2, y, 'white', 0.05)
line('mark_stop_s', -0.8, -2.9, 0.8, -2.9, 'white', 0.08)
line('mark_stop_n', -0.8, 3.1, 0.8, 3.1, 'white', 0.08)
for i, y in enumerate((-2.2, -1.6, 1.6, 2.2)):   # dashed centre line of the main aisle
    line(f'mark_center_{i}', 0.0, y - 0.2, 0.0, y + 0.2, 'white', 0.05)

# ---- walls ---------------------------------------------------------------------------------
T, H = 0.15, 1.2
box('wall_n', 0, 5 + T / 2, H / 2, 14 + 2 * T, T, H, 'wall')
box('wall_s', 0, -5 - T / 2, H / 2, 14 + 2 * T, T, H, 'wall')
box('wall_e', 7 + T / 2, 0, H / 2, T, 10, H, 'wall')
box('wall_w', -7 - T / 2, 0, H / 2, T, 10, H, 'wall')
for n, (x, y, sx, sy) in enumerate([(0, 4.99, 14, 0.02), (0, -4.99, 14, 0.02),
                                     (6.99, 0, 0.02, 10), (-6.99, 0, 0.02, 10)]):
    box(f'wall_stripe{n}', x, y, 0.15, sx, sy, 0.3, 'wall_base', collide=False)

# ---- storage racks: 2 rows per quadrant ----------------------------------------------------
seed = 0
for qx in (-1, 1):
    for qy in (-1, 1):
        for row, ry in enumerate((2.3, 3.9)):
            shelf(f'rack_{"EW"[qx > 0]}{"SN"[qy > 0]}{row}', qx * 4.0, qy * ry, seed=seed)
            seed += 1

# ---- charging docks (west wall) ------------------------------------------------------------
for i, y in enumerate((-0.475, 0.475)):
    box(f'dock{i}', -6.8, y, 0.25, 0.3, 0.45, 0.5, 'dock')
    box(f'dock{i}_light', -6.64, y, 0.40, 0.02, 0.30, 0.06, 'dock_light', collide=False)

# ---- pallets / obstacles -------------------------------------------------------------------
pallet('pallet_cross_e', 3.6, -0.45, 1.0, 0.8, 2)   # cross aisle east: 1.05 m free on the north side
pallet('pallet_cross_w', -3.2, 0.45, 1.0, 0.8, 1)   # cross aisle west: 1.05 m free on the south side
pallet('pallet_main_n', 0.45, 2.0, 0.8, 0.8, 2)     # main aisle north: 1.05 m free on the west side
pallet('pallet_delivery', 0.0, 4.65, 0.8, 0.5, 1)   # drop-off pallet at the north wall (goal in front of it)

WORLD = f'''<?xml version="1.0"?>
<!-- GENERATED by tools/gen_amr_warehouse.py - edit the generator, not this file. -->
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <scene>
      <shadows>false</shadows>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.80 0.85 1</background>
    </scene>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>0 -9.5 11.0 0 0.90 1.5708</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver><type>quick</type><iters>150</iters><precon_iters>0</precon_iters><sor>1.4</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling></solver>
        <constraints><cfm>0.00001</cfm><erp>0.2</erp>
          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer></constraints>
      </ode>
    </physics>
    <model name="amr_warehouse">
      <static>true</static>
      {chr(10).join('      ' + l for l in links).strip()}
    </model>
  </world>
</sdf>
'''

with open(OUT, 'w') as fp:
    fp.write(WORLD)
print(f'wrote {os.path.normpath(OUT)} ({len(links)} links)')
