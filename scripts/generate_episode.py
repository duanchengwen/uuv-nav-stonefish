#!/usr/bin/env python3
"""Generate one USIM-compatible UUV charging-station navigation episode.

The generator writes:
  <out>/scene.scn      Stonefish scenario (BlueROV2 + lake + charging dock + obstacles)
  <out>/path.csv       dense A* path (x,y,z,yaw) at ~0.25 m spacing
  <out>/obstacles.csv  circular obstacle approximation used by A* and collision checks
  <out>/episode.cfg    episode metadata consumed by the C++ recorder
  <out>/preview.png    top-down diagnostic view
"""
from __future__ import annotations

import argparse
import heapq
import math
import random
from dataclasses import dataclass
from pathlib import Path

import threading
import cv2
import numpy as np

SEABED_Z = 3.0
DEPTH = 2.2
AREA = (-25.0, 25.0, -15.0, 15.0)
GRID = 0.5
ROBOT_CLEARANCE = 0.55
DOCK_X, DOCK_Y = 22.5, -11.0
GOAL_X, GOAL_Y = 18.0, -11.0
BOARD_X, BOARD_Y = 20.3, -11.0
GOAL_YAW = 0.0
_PREVIEW_LOCK = threading.Lock()


@dataclass
class Circle:
    x: float
    y: float
    r: float
    zmin: float
    zmax: float
    kind: str


def cell_of(x: float, y: float) -> tuple[int, int]:
    return (int(round((x - AREA[0]) / GRID)), int(round((y - AREA[2]) / GRID)))


def world_of(ix: int, iy: int) -> tuple[float, float]:
    return (AREA[0] + ix * GRID, AREA[2] + iy * GRID)


def nx() -> int:
    return int((AREA[1] - AREA[0]) / GRID) + 1


def ny() -> int:
    return int((AREA[3] - AREA[2]) / GRID) + 1


def blocked(ix: int, iy: int, circles: list[Circle], extra: float = ROBOT_CLEARANCE) -> bool:
    if ix < 0 or iy < 0 or ix >= nx() or iy >= ny():
        return True
    x, y = world_of(ix, iy)
    for c in circles:
        if math.hypot(x - c.x, y - c.y) < c.r + extra:
            return True
    return False


def astar(start: tuple[float, float], goal: tuple[float, float], circles: list[Circle],
          extra: float = ROBOT_CLEARANCE):
    sx, sy = cell_of(*start)
    gx, gy = cell_of(*goal)

    def nearest_free(cx: int, cy: int):
        if not blocked(cx, cy, circles, extra):
            return cx, cy
        for radius in range(1, 12):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    if not blocked(cx + dx, cy + dy, circles, extra):
                        return cx + dx, cy + dy
        return None

    s = nearest_free(sx, sy)
    g = nearest_free(gx, gy)
    if s is None or g is None:
        return None

    open_heap = [(0.0, s)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {s: 0.0}
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == g:
            path = [current]
            while current in came:
                current = came[current]
                path.append(current)
            return [world_of(*p) for p in reversed(path)]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nxt = (current[0] + dx, current[1] + dy)
                if blocked(nxt[0], nxt[1], circles, extra):
                    continue
                step = math.hypot(dx, dy)
                tentative = cost[current] + step
                if tentative < cost.get(nxt, 1e18):
                    cost[nxt] = tentative
                    h = math.hypot(nxt[0] - g[0], nxt[1] - g[1])
                    heapq.heappush(open_heap, (tentative + h, nxt))
                    came[nxt] = current
    return None


def line_of_sight(a: tuple[float, float], b: tuple[float, float], circles: list[Circle],
                  extra: float = ROBOT_CLEARANCE) -> bool:
    ax, ay = cell_of(*a)
    bx, by = cell_of(*b)
    dx, dy = abs(bx - ax), abs(by - ay)
    sx = 1 if ax < bx else -1
    sy = 1 if ay < by else -1
    err = dx - dy
    ix, iy = ax, ay
    while True:
        if blocked(ix, iy, circles, extra):
            return False
        if ix == bx and iy == by:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            ix += sx
        if e2 < dx:
            err += dx
            iy += sy


def simplify(path: list[tuple[float, float]], circles: list[Circle]) -> list[tuple[float, float]]:
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_of_sight(path[i], path[j], circles):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def densify(path: list[tuple[float, float]], step: float = 0.25) -> list[tuple[float, float, float, float]]:
    out: list[tuple[float, float, float, float]] = []
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(dist / step)))
        for k in range(n):
            t = k / n
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            yaw = math.atan2(y1 - y0, x1 - x0)
            out.append((x, y, DEPTH, yaw))
    x1, y1 = path[-1]
    out.append((x1, y1, DEPTH, GOAL_YAW))
    return out


def path_clearance(path: list[tuple[float, float, float, float]], circles: list[Circle]) -> float:
    best = 1e9
    for x, y, _, _ in path:
        for c in circles:
            best = min(best, math.hypot(x - c.x, y - c.y) - c.r)
    return best


def turning_angle(path: list[tuple[float, float, float, float]]) -> float:
    total = 0.0
    for i in range(1, len(path) - 1):
        a0 = path[i - 1][3]
        a1 = path[i][3]
        total += abs(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
    return total


def make_target_texture(path: Path) -> None:
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    img = np.zeros((256, 256, 3), dtype=np.uint8)
    for i in range(-256, 256, 64):
        cv2.rectangle(img, (i, 0), (i + 32, 256), (0, 220, 255), -1)
        cv2.rectangle(img, (i + 32, 0), (i + 64, 256), (20, 20, 20), -1)
    cv2.circle(img, (128, 128), 78, (0, 255, 0), -1)
    cv2.circle(img, (128, 128), 58, (20, 20, 20), -1)
    cv2.putText(img, "UUV", (72, 142), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


def scene_header(seed: int, jerlov: float, sun_az: float, sun_el: float) -> str:
    return f'''<?xml version="1.0"?>
<scenario>
\t<environment>
\t\t<ned latitude="56.136459" longitude="-2.706819"/>
\t\t<ocean>
\t\t\t<water density="1031.0" jerlov="{jerlov:.3f}"/>
\t\t\t<waves height="0.0"/>
\t\t\t<particles enabled="true"/>
\t\t\t<current type="uniform">
\t\t\t\t<velocity xyz="0.0 0.0 0.0"/>
\t\t\t</current>
\t\t\t<current type="jet">
\t\t\t\t<center xyz="0.0 0.0 0.0"/>
\t\t\t\t<outlet radius="100.0"/>
\t\t\t\t<velocity xyz="0.0 0.0 0.0"/>
\t\t\t</current>
\t\t</ocean>
\t\t<atmosphere>
\t\t\t<sun azimuth="{sun_az:.1f}" elevation="{sun_el:.1f}"/>
\t\t</atmosphere>
\t</environment>

\t<materials>
\t\t<material name="Neutral" density="1000.0" restitution="0.1"/>
\t\t<material name="Rock" density="3000.0" restitution="0.8"/>
\t\t<material name="Fiberglass" density="1500.0" restitution="0.3"/>
\t\t<material name="Aluminium" density="2710.0" restitution="0.5"/>
\t\t<material name="Steel" density="7850.0" restitution="0.5"/>
\t\t<friction_table>
\t\t\t<friction material1="Neutral" material2="Neutral" static="0.5" dynamic="0.2"/>
\t\t\t<friction material1="Neutral" material2="Rock" static="0.2" dynamic="0.1"/>
\t\t\t<friction material1="Neutral" material2="Fiberglass" static="0.5" dynamic="0.2"/>
\t\t\t<friction material1="Neutral" material2="Aluminium" static="0.1" dynamic="0.02"/>
\t\t\t<friction material1="Rock" material2="Rock" static="0.9" dynamic="0.7"/>
\t\t\t<friction material1="Rock" material2="Fiberglass" static="0.6" dynamic="0.4"/>
\t\t\t<friction material1="Rock" material2="Aluminium" static="0.6" dynamic="0.3"/>
\t\t\t<friction material1="Rock" material2="Steel" static="0.6" dynamic="0.3"/>
\t\t\t<friction material1="Fiberglass" material2="Fiberglass" static="0.5" dynamic="0.2"/>
\t\t\t<friction material1="Fiberglass" material2="Aluminium" static="0.5" dynamic="0.2"/>
\t\t\t<friction material1="Fiberglass" material2="Steel" static="0.5" dynamic="0.2"/>
\t\t\t<friction material1="Aluminium" material2="Aluminium" static="0.8" dynamic="0.5"/>
\t\t\t<friction material1="Steel" material2="Steel" static="0.6" dynamic="0.4"/>
\t\t</friction_table>
\t</materials>

\t<looks>
\t\t<look name="br2" gray="1.0" roughness="0.4" metalness="0.5" texture="bluerov2/br2.png"/>
\t\t<look name="black" gray="0.05" roughness="0.2"/>
\t\t<look name="blue" rgb="0.0 0.5 1.0" roughness="0.3"/>
\t\t<look name="yellow" rgb="1.0 0.9 0.0" roughness="0.3"/>
\t\t<look name="gray" gray="0.5" roughness="0.4" metalness="0.5"/>
\t\t<look name="ltgray" gray="0.75" roughness="1.0" metalness="0.0"/>
\t\t<look name="sand" gray="0.85" roughness="0.85" metalness="0.0" texture="SoilBeach0078_1_350.jpg"/>
\t\t<look name="rock" gray="0.6" roughness="0.8" metalness="0.1" texture="stone/stone.png"/>
\t\t<look name="grass" gray="0.75" roughness="0.9" metalness="0.0" texture="grassstone/grass.png"/>
\t\t<look name="container" gray="0.7" roughness="0.7" metalness="0.3" texture="container/container4.png"/>
\t\t<look name="pipe" gray="0.7" roughness="0.6" metalness="0.4" texture="pipe/pipe.png"/>
\t\t<look name="solar0" rgb="0.7 0.7 0.7" roughness="0.6" metalness="0.3" texture="solar_panel/solar_panel0.png"/>
\t\t<look name="solar1" rgb="0.7 0.7 0.7" roughness="0.6" metalness="0.3" texture="solar_panel/solar_panel1.png"/>
\t\t<look name="solar2" rgb="0.7 0.7 0.7" roughness="0.6" metalness="0.3" texture="solar_panel/solar_panel2.png"/>
\t\t<look name="charge0" rgb="0.7 0.7 0.7" roughness="0.6" metalness="0.3" texture="charge/charge0.png"/>
\t\t<look name="charge1" rgb="0.7 0.7 0.7" roughness="0.6" metalness="0.3" texture="charge/charge1.png"/>
\t\t<look name="dock_target" gray="1.0" roughness="0.4" metalness="0.1" texture="textures/dock_target.png"/>
\t\t<look name="beacon" rgb="0.1 1.0 0.2" roughness="0.2"/>
\t</looks>

\t<static name="Seabed" type="model">
\t\t<physical>
\t\t\t<mesh filename="flat.obj" scale="0.1"/>
\t\t\t<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Rock"/>
\t\t<look name="sand"/>
\t\t<world_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 {SEABED_Z}"/>
\t</static>
'''


def rock_block(i: int, x: float, y: float, scale: float, yaw: float) -> str:
    z = SEABED_Z - 0.14 * scale + 0.04
    return f'''
\t<static name="rock_{i:02d}" type="model" physics="submerged">
\t\t<physical>
\t\t\t<mesh filename="stone/stone.obj" scale="{scale:.3f}"/>
\t\t\t<origin rpy="3.14 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Rock"/>
\t\t<look name="rock"/>
\t\t<world_transform xyz="{x:.3f} {y:.3f} {z:.3f}" rpy="0.0 0.0 {yaw:.3f}"/>
\t</static>
'''


def container_block(i: int, x: float, y: float, scale: float, yaw: float) -> str:
    z = SEABED_Z - 0.02
    return f'''
\t<static name="container_{i:02d}" type="model" physics="submerged">
\t\t<physical>
\t\t\t<mesh filename="container/container4.obj" scale="{scale:.3f}"/>
\t\t\t<origin rpy="3.14 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Steel"/>
\t\t<look name="container"/>
\t\t<world_transform xyz="{x:.3f} {y:.3f} {z:.3f}" rpy="0.0 0.0 {yaw:.3f}"/>
\t</static>
'''


def pipe_block(x: float, y: float, z: float, scale: float, yaw: float) -> str:
    return f'''
\t<static name="pipe_00" type="model" physics="submerged">
\t\t<physical>
\t\t\t<mesh filename="pipe/pipe.obj" scale="{scale:.3f}"/>
\t\t\t<origin rpy="3.14 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Steel"/>
\t\t<look name="pipe"/>
\t\t<world_transform xyz="{x:.3f} {y:.3f} {z:.3f}" rpy="0.0 0.0 {yaw:.3f}"/>
\t</static>
'''


def grass_block(i: int, x: float, y: float, scale: float, yaw: float) -> str:
    z = SEABED_Z - 0.01
    return f'''
\t<static name="grass_{i:02d}" type="model" physics="submerged">
\t\t<physical>
\t\t\t<mesh filename="grassstone/grass.obj" scale="{scale:.3f}"/>
\t\t\t<origin rpy="3.14 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Rock"/>
\t\t<look name="grass"/>
\t\t<world_transform xyz="{x:.3f} {y:.3f} {z:.3f}" rpy="0.0 0.0 {yaw:.3f}"/>
\t</static>
'''


def dock_block() -> str:
    parts = []
    # Seabed platform
    parts.append(f'''
\t<static name="dock_platform" type="box" physics="submerged">
\t\t<dimensions xyz="4.5 3.6 0.25"/>
\t\t<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t<material name="Steel"/>
\t\t<look name="gray"/>
\t\t<world_transform xyz="{DOCK_X} {DOCK_Y} {SEABED_Z - 0.15}" rpy="0.0 0.0 0.0"/>
\t</static>
''')
    # Charging pads (USIM charge meshes)
    for idx, name in enumerate(("charge0", "charge1")):
        parts.append(f'''
\t<static name="dock_{name}" type="model" physics="submerged">
\t\t<physical>
\t\t\t<mesh filename="charge/{name}.obj" scale="1.5"/>
\t\t\t<origin rpy="3.14 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Aluminium"/>
\t\t<look name="{name}"/>
\t\t<world_transform xyz="{DOCK_X} {DOCK_Y} {SEABED_Z - 0.42}" rpy="0.0 0.0 0.0"/>
\t</static>
''')
    # Four support pylons from platform to surface
    for i, (px, py) in enumerate(((DOCK_X - 2.0, DOCK_Y - 1.5), (DOCK_X + 2.0, DOCK_Y - 1.5),
                                  (DOCK_X - 2.0, DOCK_Y + 1.5), (DOCK_X + 2.0, DOCK_Y + 1.5))):
        parts.append(f'''
\t<static name="dock_pylon_{i}" type="box" physics="submerged">
\t\t<dimensions xyz="0.24 0.24 2.9"/>
\t\t<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t<material name="Steel"/>
\t\t<look name="gray"/>
\t\t<world_transform xyz="{px:.3f} {py:.3f} 1.45" rpy="0.0 0.0 0.0"/>
\t</static>
''')
    # Floating solar canopy (surface)
    for dx in (-3.0, 0.0, 3.0):
        for dy in (-3.0, 0.0, 3.0):
            parts.append(f'''
\t<static name="solar_{int((dx+3)/3)}_{int((dy+3)/3)}" type="model" physics="submerged">
\t\t<physical>
\t\t\t<mesh filename="solar_panel/solar_panel0.obj" scale="0.8"/>
\t\t\t<origin rpy="3.14 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t</physical>
\t\t<material name="Steel"/>
\t\t<look name="solar0"/>
\t\t<world_transform xyz="{DOCK_X + dx:.3f} {DOCK_Y + dy:.3f} 2.0" rpy="0.0 0.0 0.0"/>
\t</static>
''')
    # Target board facing the approach direction (-X)
    parts.append(f'''
\t<static name="dock_target" type="box" physics="submerged">
\t\t<dimensions xyz="0.06 2.2 1.4"/>
\t\t<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t<material name="Aluminium"/>
\t\t<look name="dock_target"/>
\t\t<world_transform xyz="{DOCK_X - 2.2:.3f} {DOCK_Y} 1.95" rpy="0.0 0.0 0.0"/>
\t</static>
''')
    # Beacon block
    parts.append(f'''
\t<static name="dock_beacon" type="box" physics="submerged">
\t\t<dimensions xyz="0.25 0.25 0.9"/>
\t\t<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
\t\t<material name="Aluminium"/>
\t\t<look name="beacon"/>
\t\t<world_transform xyz="{DOCK_X - 1.8:.3f} {DOCK_Y - 1.1:.3f} 2.25" rpy="0.0 0.0 0.0"/>
\t</static>
''')
    return "".join(parts)


def dock_light() -> str:
    return f'''
\t<light name="dock_beacon_light">
\t\t<specs radius="0.18" illuminance="4500.0"/>
\t\t<color rgb="0.1 1.0 0.2"/>
\t\t<world_transform xyz="{DOCK_X - 1.8:.3f} {DOCK_Y - 1.1:.3f} 2.2" rpy="0.0 0.0 0.0"/>
\t</light>
'''


def point_segment_distance(x: float, y: float, a: tuple[float, float],
                           b: tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    l2 = dx * dx + dy * dy
    if l2 <= 0.0:
        return math.hypot(x - ax, y - ay)
    t = ((x - ax) * dx + (y - ay) * dy) / l2
    t = max(0.0, min(1.0, t))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def build_episode(out_dir: Path, seed: int, split: str, episode_index: int, attempts: int = 40,
                  success_radius: float = 0.30, success_yaw: float = 0.10,
                  start_frac: float = 0.0):
    rng = random.Random(seed)
    goal = (GOAL_X, GOAL_Y)
    board = (BOARD_X, BOARD_Y)
    start_frac = max(0.0, min(1.0, start_frac))
    counts = {}

    def bump(key):
        counts[key] = counts.get(key, 0) + 1

    for attempt in range(attempts):
        # Start 9-15 m from the goal, face the charging station, and keep the
        # target visible. start_frac biases the distance: 0 -> close, 1 -> far.
        dmin = 8.0 + 2.0 * start_frac
        dmax = 12.0 + 3.0 * start_frac
        for _ in range(200):
            sx = rng.uniform(4.0, 13.0)
            sy = rng.uniform(-11.0, 0.0)
            if dmin <= math.hypot(sx - goal[0], sy - goal[1]) <= dmax:
                break
        else:
            bump("no_start")
            continue
        start = (sx, sy)

        circles: list[Circle] = [
            Circle(DOCK_X, DOCK_Y, 2.5, SEABED_Z - 1.0, SEABED_Z, "dock")
        ]
        rocks = []
        containers = []
        pipe_info = None

        def add_rock(x: float, y: float, scale: float):
            yaw = rng.uniform(-math.pi, math.pi)
            r = 0.18 * scale + 0.18
            rocks.append((x, y, scale, yaw))
            circles.append(Circle(x, y, r, SEABED_Z - 0.35 * scale, SEABED_Z, "rock"))

        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        length = math.hypot(dx, dy)
        ux, uy = dx / length, dy / length
        px, py = -uy, ux

        # Random obstacle field: 3-6 gates with random fraction/side/offset,
        # 1-2 containers, an optional pipeline, and 10-24 scenery rocks.
        n_gates = rng.randint(3, 6)
        gate_fracs = sorted(rng.uniform(0.18, 0.88) for _ in range(n_gates))
        for frac in gate_fracs:
            side = 1.0 if rng.random() < 0.5 else -1.0
            off = rng.uniform(0.9, 4.2)
            bx = start[0] + dx * frac
            by = start[1] + dy * frac
            add_rock(bx + px * off * side + rng.uniform(-0.35, 0.35),
                     by + py * off * side + rng.uniform(-0.35, 0.35),
                     rng.uniform(3.0, 5.5))

        for _ in range(rng.randint(1, 2)):
            frac = rng.uniform(0.20, 0.82)
            side = 1.0 if rng.random() < 0.5 else -1.0
            off = rng.uniform(2.4, 5.8)
            bx = start[0] + dx * frac
            by = start[1] + dy * frac
            x = bx + px * off * side
            y = by + py * off * side
            scale = rng.uniform(0.75, 0.95)
            containers.append((x, y, scale, rng.uniform(-math.pi, math.pi)))
            circles.append(Circle(x, y, 1.75 * scale, SEABED_Z - 1.6 * scale, SEABED_Z,
                                  "container"))

        if rng.random() < 0.7:
            frac = rng.uniform(0.30, 0.70)
            side = 1.0 if rng.random() < 0.5 else -1.0
            off = rng.uniform(3.0, 6.0)
            bx = start[0] + dx * frac
            by = start[1] + dy * frac
            pxp = bx + px * off * side
            pyp = by + py * off * side
            yaw = math.atan2(uy, ux) + rng.uniform(-0.5, 0.5)
            pipe_info = (pxp, pyp, 2.55, rng.uniform(0.5, 0.6), yaw)
            for t in (-1.5, 0.0, 1.5):
                circles.append(Circle(pxp + math.cos(yaw) * t, pyp + math.sin(yaw) * t,
                                      1.15, SEABED_Z - 0.8, SEABED_Z, "pipe"))

        # Scenery rocks may sit close to the sight line; the visibility check
        # below allows partial occlusion (2 of 3 target-board samples).
        for _ in range(rng.randint(10, 24)):
            for _try in range(80):
                x = rng.uniform(AREA[0] + 3, AREA[1] - 3)
                y = rng.uniform(AREA[2] + 3, AREA[3] - 3)
                if math.hypot(x - start[0], y - start[1]) < 3.0:
                    continue
                if math.hypot(x - goal[0], y - goal[1]) < 3.0:
                    continue
                if math.hypot(x - DOCK_X, y - DOCK_Y) < 4.5:
                    continue
                if any(math.hypot(x - c.x, y - c.y) < c.r + 0.9 for c in circles):
                    continue
                add_rock(x, y, rng.uniform(2.6, 4.8))
                break

        grasses = []
        for _ in range(rng.randint(8, 16)):
            for _try in range(50):
                x = rng.uniform(AREA[0] + 3, AREA[1] - 3)
                y = rng.uniform(AREA[2] + 3, AREA[3] - 3)
                if math.hypot(x - goal[0], y - goal[1]) < 3.5:
                    continue
                if any(math.hypot(x - c.x, y - c.y) < c.r + 1.0 for c in circles):
                    continue
                grasses.append((x, y, rng.uniform(1.2, 2.0), rng.uniform(-math.pi, math.pi)))
                break

        if any(math.hypot(start[0] - c.x, start[1] - c.y) < c.r + 1.0 for c in circles):
            bump("start_clear")
            continue

        # Require at least 2 of the 3 target-board samples to be visible. This
        # keeps the charging station recognizable at frame 0 while allowing
        # rocks to partially occlude the board, varying the scene difficulty.
        board_samples = (
            (BOARD_X, BOARD_Y - 0.9),
            (BOARD_X, BOARD_Y),
            (BOARD_X, BOARD_Y + 0.9),
        )
        los_circles = [c for c in circles if c.kind != "dock"]
        target_visible = sum(
            1 for p in board_samples if line_of_sight(start, p, los_circles, extra=0.1)
        )
        if target_visible < 2:
            bump("los")
            continue

        raw = astar(start, goal, circles)
        if raw is None:
            bump("astar")
            continue
        simple = simplify(raw, circles)
        dense = densify(simple)
        if len(dense) < 30:
            bump("short")
            continue
        if path_clearance(dense, circles) < 0.45:
            bump("clearance")
            continue
        if turning_angle(dense) < 0.15:
            bump("turn")
            continue
        bump("ok")
        break
    else:
        raise RuntimeError(f"could not generate a valid episode for seed {seed}: {counts}")

    # Face the target board at the initial frame.
    start_yaw = math.atan2(board[1] - start[1], board[0] - start[0])
    jerlov = rng.uniform(0.10, 0.18)
    sun_az = rng.uniform(-180.0, 180.0)
    sun_el = rng.uniform(35.0, 80.0)

    xml = scene_header(seed, jerlov, sun_az, sun_el)
    xml += f'''
\t<include file="scenes/robots/bluerov2_usim_nav.scn">
\t\t<arg name="vehicle_name" value="bluerov2"/>
\t\t<arg name="position" value="{start[0]:.3f} {start[1]:.3f} {DEPTH:.3f}"/>
\t\t<arg name="orientation" value="0.000 0.000 {start_yaw:.3f}"/>
\t</include>
'''
    for i, (x, y, scale, yaw) in enumerate(rocks):
        xml += rock_block(i, x, y, scale, yaw)
    for i, (x, y, scale, yaw) in enumerate(containers):
        xml += container_block(i, x, y, scale, yaw)
    if pipe_info is not None:
        xml += pipe_block(*pipe_info)
    for i, (x, y, scale, yaw) in enumerate(grasses):
        xml += grass_block(i, x, y, scale, yaw)
    xml += dock_block()
    xml += dock_light()
    xml += "\n</scenario>\n"

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "scene.scn").write_text(xml)

    with (out_dir / "path.csv").open("w") as f:
        f.write("x,y,z,yaw\n")
        for x, y, z, yaw in dense:
            f.write(f"{x:.4f},{y:.4f},{z:.4f},{yaw:.6f}\n")

    with (out_dir / "obstacles.csv").open("w") as f:
        f.write("x,y,radius,zmin,zmax,type\n")
        for c in circles:
            f.write(f"{c.x:.4f},{c.y:.4f},{c.r:.4f},{c.zmin:.4f},{c.zmax:.4f},{c.kind}\n")

    with (out_dir / "episode.cfg").open("w") as f:
        f.write("task=Go to the charge station\n")
        f.write(f"split={split}\n")
        f.write(f"episode_name={out_dir.name}\n")
        f.write(f"seed={seed}\n")
        f.write(f"episode_index={episode_index}\n")
        f.write(f"start_x={start[0]:.4f}\nstart_y={start[1]:.4f}\nstart_z={DEPTH:.4f}\nstart_yaw={start_yaw:.6f}\n")
        f.write(f"goal_x={GOAL_X:.4f}\ngoal_y={GOAL_Y:.4f}\ngoal_z={DEPTH:.4f}\ngoal_yaw={GOAL_YAW:.6f}\n")
        f.write(f"depth={DEPTH}\nsuccess_radius={success_radius:.3f}\nsuccess_yaw={success_yaw:.3f}\n")
        f.write(f"target_visible_samples={target_visible}\nmax_time=75.0\n")

    with _PREVIEW_LOCK:
        make_preview(out_dir / "preview.png", start, goal, circles, dense, rocks, containers,
                     pipe_info)
    return {
        "start": (start[0], start[1], start_yaw),
        "path_length": sum(math.dist(dense[i][:2], dense[i + 1][:2]) for i in range(len(dense) - 1)),
        "turning": turning_angle(dense),
        "clearance": path_clearance(dense, circles),
        "rocks": len(rocks),
        "target_visible": target_visible,
    }


def make_preview(path: Path, start, goal, circles, dense, rocks, containers, pipe_info):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 7))
    for c in circles:
        ax.add_patch(plt.Circle((c.x, c.y), c.r, color="0.55", alpha=0.6))
    for x, y, s, _ in rocks:
        ax.plot(x, y, "k.", ms=2)
    for x, y, s, _ in containers:
        ax.plot(x, y, "s", color="tab:brown", ms=5)
    if pipe_info is not None:
        ax.plot(pipe_info[0], pipe_info[1], "^", color="tab:orange", ms=7)
    px = [p[0] for p in dense]
    py = [p[1] for p in dense]
    ax.plot(px, py, "c-", lw=2, label="A* path")
    ax.plot(start[0], start[1], "go", ms=8, label="start")
    ax.plot(goal[0], goal[1], "r*", ms=14, label="goal")
    ax.plot(DOCK_X, DOCK_Y, "ys", ms=10, label="dock")
    ax.set_xlim(AREA[0], AREA[1])
    ax.set_ylim(AREA[2], AREA[3])
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    ax.set_title("USIM UUV charge-station episode")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    project_data = Path(__file__).resolve().parent.parent / "data"
    make_target_texture(project_data / "textures" / "dock_target.png")
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--split", default="train", choices=["train", "test", "debug"])
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--success-radius", type=float, default=0.30)
    parser.add_argument("--success-yaw", type=float, default=0.10)
    parser.add_argument("--start-frac", type=float, default=0.0,
                        help="0=close start (9-12 m), 1=far start (11-15 m); target stays visible")
    parser.add_argument("--depth", type=float, default=None, help="cruise depth (NED z, metres)")
    args = parser.parse_args()
    if args.depth is not None:
        global DEPTH
        DEPTH = args.depth
    info = build_episode(Path(args.out), args.seed, args.split, args.episode_index,
                         success_radius=args.success_radius, success_yaw=args.success_yaw,
                         start_frac=args.start_frac)
    print(f"[generate] {args.out}: {info}")


if __name__ == "__main__":
    main()
