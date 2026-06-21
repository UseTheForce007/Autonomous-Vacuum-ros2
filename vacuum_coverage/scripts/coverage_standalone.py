#!/usr/bin/env python3
"""
Standalone boustrophedon coverage planner with A* obstacle-aware transitions.

Strategy:
  - Room decomposition uses 8-connectivity on the ORIGINAL grid (merges rooms
    separated by 1px diagonal walls into connected components).
  - Intra-room sweep & A* pathfinding use the INFLATED grid (robot-radius
    dilation of obstacles) so the robot's body stays clear of walls.
  - Inter-room connectors also run A* on the ORIGINAL grid so narrow passages
    / doorways remain passable (the inflated grid may close them).
  - ALL produced waypoints and all transitions are validated against the
    ORIGINAL grid — any point on an obstacle cell is flagged.
"""

import argparse
import heapq
import math
import os

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


def load_pgm(pgm_path):
    with open(pgm_path, 'rb') as f:
        f.readline()
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        data = f.read()
    # 0 = free, 1 = occupied (black pixels < 50)
    grid = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            if data[y * w + x] < 50:
                grid[y][x] = 1
    return grid, w, h


def load_map_yaml(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    return pgm, meta['resolution'], meta['origin']


def inflate_grid(grid, r):
    """Morphological dilation of obstacles by radius r (Chebyshev disk)."""
    if r == 0:
        return [row[:] for row in grid]
    h, w = len(grid), len(grid[0])
    out = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            if grid[y][x]:
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if dx * dx + dy * dy <= r * r:
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < h and 0 <= nx < w:
                                out[ny][nx] = 1
    return out


def find_rooms(grid, min_cells=100):
    """Connected-component labelling using 8-connectivity (no corner-cutting)."""
    h, w = len(grid), len(grid[0])
    seen = [[False] * w for _ in range(h)]
    rooms = []
    for y in range(h):
        for x in range(w):
            if grid[y][x] or seen[y][x]:
                continue
            stack = [(x, y)]
            comp = set()
            while stack:
                cx, cy = stack.pop()
                if not (0 <= cx < w and 0 <= cy < h):
                    continue
                if seen[cy][cx] or grid[cy][cx]:
                    continue
                seen[cy][cx] = True
                comp.add((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    stack.append((cx + dx, cy + dy))
            if len(comp) >= min_cells:
                rooms.append(comp)
    rooms.sort(key=len, reverse=True)
    return rooms


def astar_no_cc(grid, start, goal):
    """
    A* on the original grid with pure 8-connectivity (no corner-cutting check).

    This is used for inter-room connectors because the inflated grid may
    close the single-pixel diagonal doorway.  Every path is validated
    against the original grid afterwards.
    """
    h, w = len(grid), len(grid[0])
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return None
    if grid[sy][sx] or grid[gy][gx]:
        return None
    open_set = [(0.0, sx, sy)]
    came_from = {}
    g = {start: 0.0}
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while open_set:
        _, cx, cy = heapq.heappop(open_set)
        if (cx, cy) == goal:
            path = []
            while (cx, cy) in came_from:
                path.append((cx, cy))
                cx, cy = came_from[(cx, cy)]
            path.append(start)
            path.reverse()
            return path
        for dx, dy in dirs:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h) or grid[ny][nx]:
                continue
            nb = (nx, ny)
            cost = g[(cx, cy)] + math.hypot(dx, dy)
            if nb not in g or cost < g[nb]:
                came_from[nb] = (cx, cy)
                g[nb] = cost
                heapq.heappush(open_set,
                               (cost + math.hypot(gx - nx, gy - ny), nx, ny))
    return None


def astar_cc(grid, start, goal):
    """
    A* with corner-cutting check: reject diagonal moves where BOTH
    axis-aligned neighbours are obstacles.  Used on the INFLATED grid
    for intra-room sweep transitions.
    """
    h, w = len(grid), len(grid[0])
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return None
    if grid[sy][sx] or grid[gy][gx]:
        return None
    open_set = [(0.0, sx, sy)]
    came_from = {}
    g = {start: 0.0}
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while open_set:
        _, cx, cy = heapq.heappop(open_set)
        if (cx, cy) == goal:
            path = []
            while (cx, cy) in came_from:
                path.append((cx, cy))
                cx, cy = came_from[(cx, cy)]
            path.append(start)
            path.reverse()
            return path
        for dx, dy in dirs:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h) or grid[ny][nx]:
                continue
            # Corner-cutting: reject diagonal when both orthogonal
            # neighbours are obstacles (would clip a convex corner).
            if abs(dx) == 1 and abs(dy) == 1 and (grid[cy][nx] and grid[ny][cx]):
                continue
            nb = (nx, ny)
            cost = g[(cx, cy)] + math.hypot(dx, dy)
            if nb not in g or cost < g[nb]:
                came_from[nb] = (cx, cy)
                g[nb] = cost
                heapq.heappush(open_set,
                               (cost + math.hypot(gx - nx, gy - ny), nx, ny))
    return None


def pixel_to_world(px, py, map_h, origin, resolution):
    ox, oy, _ = origin
    wx = px * resolution + ox
    wy = (map_h - 1 - py) * resolution + oy
    return (round(wx, 3), round(wy, 3))


def cover_room(grid, cells, stride):
    """
    Boustrophedon coverage within a single connected component.

    ``grid`` is the INFLATED grid for safe pathfinding.  ``cells`` are
    the free cells of this room (from the inflated-grid decomposition).

    Returns list of (x, y) pixel-coordinate waypoints.
    """
    if not cells:
        return []
    xs = [x for x, _ in cells]
    ys = [y for _, y in cells]
    min_y, max_y = min(ys), max(ys)
    min_x, max_x = min(xs), max(xs)
    room = set(cells)
    horiz = (max_x - min_x) >= (max_y - min_y)
    wp = []

    if horiz:
        rows = list(range(min_y, max_y + 1, stride))
        # Pre-compute free cells per row
        row_data = []
        for y in rows:
            row_data.append(
                sorted([x for x in range(min_x, max_x + 1)
                        if (x, y) in room and not grid[y][x]])
            )
        for ri, y in enumerate(rows):
            here = row_data[ri]
            if not here:
                continue
            sweep = here if ri % 2 == 0 else list(reversed(here))
            for px in sweep:
                if wp and abs(px - wp[-1][0]) > 1:
                    p = astar_cc(grid, wp[-1], (px, y))
                    if p:
                        wp.extend(p[1:])
                    else:
                        wp.append((px, y))
                else:
                    wp.append((px, y))
            # Transition to next row
            if ri < len(rows) - 1:
                nxt = row_data[ri + 1]
                if nxt:
                    nx = nxt[0] if (ri + 1) % 2 == 0 else nxt[-1]
                    p = astar_cc(grid, wp[-1], (nx, rows[ri + 1]))
                    if p:
                        wp.extend(p[1:])
    else:
        cols = list(range(min_x, max_x + 1, stride))
        col_data = []
        for x in cols:
            col_data.append(
                sorted([y for y in range(min_y, max_y + 1)
                        if (x, y) in room and not grid[y][x]])
            )
        for ci, x in enumerate(cols):
            here = col_data[ci]
            if not here:
                continue
            sweep = here if ci % 2 == 0 else list(reversed(here))
            for py in sweep:
                if wp and abs(py - wp[-1][1]) > 1:
                    p = astar_cc(grid, wp[-1], (x, py))
                    if p:
                        wp.extend(p[1:])
                    else:
                        wp.append((x, py))
                else:
                    wp.append((x, py))
            if ci < len(cols) - 1:
                nxt = col_data[ci + 1]
                if nxt:
                    ny = nxt[0] if (ci + 1) % 2 == 0 else nxt[-1]
                    p = astar_cc(grid, wp[-1], (cols[ci + 1], ny))
                    if p:
                        wp.extend(p[1:])
    return wp


def validate(path, occ_grid):
    """
    Check every waypoint and every transition against the OCCUPANCY grid.

    Returns list of (index, x, y, kind) violations.
    """
    h, w = len(occ_grid), len(occ_grid[0])
    bad = []
    for i, (px, py) in enumerate(path):
        if not (0 <= px < w and 0 <= py < h):
            bad.append((i, px, py, 'OOB'))
        elif occ_grid[py][px]:
            bad.append((i, px, py, 'OBS'))
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        steps = max(abs(x2 - x1), abs(y2 - y1))
        if steps <= 1:
            continue
        for t in range(1, steps):
            x = x1 + (x2 - x1) * t // steps
            y = y1 + (y2 - y1) * t // steps
            if not (0 <= x < w and 0 <= y < h):
                continue
            if occ_grid[y][x]:
                bad.append((i, x, y, 'TRANS'))
    return bad


def main():
    ap = argparse.ArgumentParser(
        description='Boustrophedon coverage planner with A* transitions')
    ap.add_argument('--map-yaml', default='maps/map_2.0.yaml')
    ap.add_argument('--spacing', type=float, default=0.3,
                    help='Sweep row spacing in metres (default 0.3)')
    ap.add_argument('--robot-radius', type=float, default=0.2,
                    help='Robot radius in metres (default 0.2)')
    ap.add_argument('--output', default='coverage_path.yaml')
    ap.add_argument('--min-room-area', type=float, default=0.5)
    ap.add_argument('--save-plot', default=None,
                    help='Save coverage overlay to this PNG file')
    args = ap.parse_args()

    pgm_path, resolution, origin = load_map_yaml(args.map_yaml)
    occ_grid, w, h = load_pgm(pgm_path)          # original occupancy
    rpx = max(1, int(args.robot_radius / resolution))
    spx = max(1, int(args.spacing / resolution))
    min_px = int(args.min_room_area / (resolution * resolution))

    print(f'Map: {w}x{h}  res={resolution}m  '
          f'inflation={rpx}px  stride={spx}px')

    # --- Inflated grid for intra-room safety ---
    inf_grid = inflate_grid(occ_grid, rpx)
    free_before = sum(row.count(0) for row in occ_grid)
    free_after = sum(row.count(0) for row in inf_grid)
    print(f'Free cells: {free_before} → {free_after} (inflated)')

    # --- Room decomposition on ORIGINAL grid (noCC, 8-conn) ---
    # This keeps rooms connected even through 1-pixel diagonal passages.
    orig_rooms = find_rooms(occ_grid, min_px)
    print(f'\nConnected components (original grid, 8-conn):')

    # --- Sub-room decomposition on INFLATED grid ---
    # These are the "actual rooms" the robot can traverse safely.
    inf_rooms = find_rooms(inf_grid, min_px)
    print(f'Inflated components: {len(inf_rooms)}')

    # For each original room, find which inflated sub-rooms it contains
    # and generate coverage for each sub-room, then connect them.
    full_path = []
    total_violations = 0
    total_covered = set()

    # Only process the largest original room — smaller components are
    # outside-area disconnected by genuine walls.
    interior = orig_rooms[0] if orig_rooms else set()
    xs = [x for x, _ in interior]
    ys = [y for _, y in interior]
    area_m2 = len(interior) * resolution * resolution
    print(f'  Interior room: {len(interior)}c {area_m2:.1f}m²  '
          f'bbox=[{min(xs)}-{max(xs)}],[{min(ys)}-{max(ys)}]')

    overlapping = []
    for ir_idx, ir in enumerate(inf_rooms):
        overlap = len(interior & ir)
        if overlap > 0:
            overlapping.append((ir_idx, ir, overlap))
    overlapping.sort(key=lambda t: -t[2])

    print(f'  Contains {len(overlapping)} inflated sub-room(s)')

    sub_paths = []
    for ir_idx, ir, ov in overlapping:
        area_ir = len(ir) * resolution * resolution
        wp = cover_room(inf_grid, list(ir), spx)
        v = validate(wp, occ_grid)
        total_violations += len(v)
        total_covered.update(wp)
        status = f'{len(v)} viols' if v else 'OK'
        print(f'    Inflated sub-room {ir_idx}: {len(ir)}c  '
              f'{area_ir:.1f}m² → {len(wp)}wp  {status}')
        sub_paths.append(wp)

    # Connect sub-rooms using A* on the ORIGINAL grid
    # (so the diagonal doorway is passable).
    merged = []
    for i, sp in enumerate(sub_paths):
        if not sp:
            continue
        if merged:
            c = astar_no_cc(occ_grid, merged[-1], sp[0])
            if c:
                vc = validate(c, occ_grid)
                total_violations += len(vc)
                print(f'      Connector: {len(c)} steps  {len(vc)} viols')
                merged.extend(c[1:])
            else:
                print(f'      Connector: A* FAILED')
        merged.extend(sp)
    full_path.extend(merged)

    # --- Summary ---
    plen = sum(
        math.hypot((full_path[i][0] - full_path[i - 1][0]) * resolution,
                   (full_path[i][1] - full_path[i - 1][1]) * resolution)
        for i in range(1, len(full_path))
    )
    total_free = sum(row.count(0) for row in occ_grid)
    cov_pct = len(total_covered) / total_free * 100 if total_free else 0
    v = validate(full_path, occ_grid)

    print(f'\n{"=" * 50}')
    print(f'Coverage summary')
    print(f'{"=" * 50}')
    print(f'Total waypoints      : {len(full_path)}')
    print(f'Unique cells covered : {len(total_covered)} / {total_free} free '
          f'({cov_pct:.1f}%)')
    print(f'Path length          : {plen:.1f}m')
    print(f'Obstacle crossings   : {len(v)}')

    # --- Save ---
    wpath = [pixel_to_world(px, py, h, origin, resolution)
             for px, py in full_path]
    data = {
        'coverage_path': [{'x': p[0], 'y': p[1]} for p in wpath],
        'num_poses': len(wpath),
        'path_length_m': round(plen, 2),
        'coverage_pct': round(cov_pct, 1),
        'obstacle_crossings': len(v),
    }
    with open(args.output, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)
    print(f'\nSaved {len(wpath)} world-coordinate poses to {args.output}')

    # --- Plot ---
    if args.save_plot:
        ox, oy, _ = origin
        # Align pixel (0, h-1) center → (ox, oy) and pixel (0, 0) center
        # → (ox, oy + (h-1)*res).  origin='upper' keeps PGM row 0 at top.
        extent = [ox - 0.5 * resolution, ox + (w - 0.5) * resolution,
                  oy - 0.5 * resolution, oy + (h - 0.5) * resolution]
        _, ax = plt.subplots(figsize=(10, 7))
        img = np.array(Image.open(pgm_path).convert('L'))
        ax.imshow(img, cmap='gray', origin='upper', extent=extent)

        pts = [(wx, wy) for wx, wy in wpath]
        xs, ys = zip(*pts) if pts else ([], [])
        ax.plot(xs, ys, color='lime', linewidth=0.3, alpha=0.7,
                label=f'Coverage ({len(pts)} wp)')

        # Overlay obstacle crossings if any
        for reason, px, py, kind in v:
            wx = px * resolution + ox
            wy = (h - 1 - py) * resolution + oy
            m = {'WP': 'o', 'TRANS': 'x'}.get(kind, 'x')
            ax.plot(wx, wy, color='red', marker=m, markersize=3)

        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title(f'Coverage path — {len(pts)} waypoints, '
                     f'{plen:.0f}m, {len(v)} crossings')
        ax.legend(fontsize=8)
        plt.tight_layout()
        plt.savefig(args.save_plot, dpi=200)
        print(f'Saved overlay plot to {args.save_plot}')


if __name__ == '__main__':
    main()
