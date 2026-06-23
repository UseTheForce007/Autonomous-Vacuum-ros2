#!/usr/bin/env python3
"""
Offline grid-based boustrophedon coverage path planner.

Works directly on the PGM occupancy grid using connected components,
so internal walls naturally separate rooms.
"""

import argparse
import math
import os
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def load_map(map_yaml_path):
    with open(map_yaml_path) as f:
        meta = yaml.safe_load(f)
    res = meta['resolution']
    origin = meta['origin']
    pgm_rel = meta['image']
    pgm_path = os.path.join(os.path.dirname(map_yaml_path), pgm_rel)
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f'Cannot load {pgm_path}')
    return img, res, origin, pgm_path


def compute_connected_components(map_img, free_thresh=200):
    free_mask = (map_img >= free_thresh).astype(np.uint8) * 255
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    free_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, close_kernel)
    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        free_mask, connectivity=8)
    return free_mask, labels, stats, centroids


def compute_sweep_angle(component_mask):
    ys, xs = np.where(component_mask > 0)
    if len(xs) < 4:
        return 0.0
    points = np.column_stack([xs, ys]).astype(np.float64)
    mean = points.mean(axis=0)
    centered = points - mean
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    principal = eigenvectors[:, 1]
    return math.atan2(principal[1], principal[0])


def pixel_to_map(px, py, map_h, origin, resolution):
    ox, oy = origin
    return (ox + px * resolution, oy + (map_h - py) * resolution)


def map_to_pixel(wx, wy, map_h, origin, resolution):
    ox, oy = origin
    px = (wx - ox) / resolution
    py = map_h - (wy - oy) / resolution
    return px, py


def grid_boustrophedon(component_mask, map_h, origin, resolution, spacing, sweep_angle):
    ox, oy = origin
    ys, xs = np.where(component_mask > 0)
    if len(xs) == 0:
        return []

    dx_perp = math.cos(sweep_angle + math.pi / 2)
    dy_perp = math.sin(sweep_angle + math.pi / 2)
    dx_sweep = math.cos(sweep_angle)
    dy_sweep = math.sin(sweep_angle)

    perp_proj = xs * dx_perp + ys * dy_perp
    sweep_proj = xs * dx_sweep + ys * dy_sweep

    min_perp = perp_proj.min()
    max_perp = perp_proj.max()

    spacing_px = spacing / resolution
    num_rows = max(1, int(math.ceil((max_perp - min_perp) / spacing_px)))

    row_assignments = np.round((perp_proj - min_perp) / spacing_px).astype(np.int32)
    row_assignments = np.clip(row_assignments, 0, num_rows - 1)

    waypoints = []

    for row_idx in range(num_rows):
        perp_center_px = min_perp + (row_idx + 0.5) * spacing_px

        row_mask = (row_assignments == row_idx)
        sw_vals = sweep_proj[row_mask]
        px_vals = xs[row_mask].astype(np.float64)
        py_vals = ys[row_mask].astype(np.float64)

        if len(sw_vals) < 2:
            continue

        sort_order = np.argsort(sw_vals)
        sw_sorted = sw_vals[sort_order]
        x_sorted = px_vals[sort_order]
        y_sorted = py_vals[sort_order]

        gap_threshold_px = 2.0
        gaps = np.diff(sw_sorted) > gap_threshold_px
        split_indices = np.where(gaps)[0] + 1

        segments = []
        start = 0
        for idx in split_indices:
            seg_x = x_sorted[start:idx]
            seg_y = y_sorted[start:idx]
            if len(seg_x) >= 2:
                segments.append((seg_x, seg_y, sw_sorted[start:idx]))
            start = idx
        seg_x = x_sorted[start:]
        seg_y = y_sorted[start:]
        if len(seg_x) >= 2:
            segments.append((seg_x, seg_y, sw_sorted[start:]))

        if not segments:
            continue

        # Odd rows: reverse segment order (process right→left)
        # Keep segment coords in sweep_proj ascending (natural pixel order)
        # but swap which endpoint is added first via first/last
        if row_idx % 2 == 1:
            segments.reverse()

        for seg_idx, (seg_xs, seg_ys, seg_sw) in enumerate(segments):
            n = len(seg_xs)
            if row_idx % 2 == 0:
                first, last = 0, n - 1
            else:
                first, last = n - 1, 0

            for idx in (first, last):
                sw_val = float(seg_sw[idx])
                dx_px = sw_val * dx_sweep + perp_center_px * dx_perp
                dy_px = sw_val * dy_sweep + perp_center_px * dy_perp
                wx, wy = pixel_to_map(dx_px, dy_px, map_h, origin, resolution)
                if not waypoints or math.hypot(wx - waypoints[-1][0],
                                               wy - waypoints[-1][1]) > resolution * 0.5:
                    waypoints.append((wx, wy))

            if seg_idx < len(segments) - 1:
                next_seg = segments[seg_idx + 1]
                next_sw = float(next_seg[2][0])
                dx_px = next_sw * dx_sweep + perp_center_px * dx_perp
                dy_px = next_sw * dy_sweep + perp_center_px * dy_perp
                wx, wy = pixel_to_map(dx_px, dy_px, map_h, origin, resolution)
                if math.hypot(wx - waypoints[-1][0], wy - waypoints[-1][1]) > resolution:
                    waypoints.append((wx, wy))

    return waypoints


def decimate_path(path, min_dist):
    if len(path) < 2:
        return path
    result = [path[0]]
    for pt in path[1:]:
        dist = math.hypot(pt[0] - result[-1][0], pt[1] - result[-1][1])
        if dist >= min_dist:
            result.append(pt)
    return result


def path_length(path):
    return sum(math.hypot(path[i][0] - path[i - 1][0],
                          path[i][1] - path[i - 1][1])
               for i in range(1, len(path)))


def visualize(map_img, map_extent, all_paths, output_path=None):
    fig, ax = plt.subplots(figsize=(14, 11))

    ax.imshow(map_img, cmap='gray', extent=map_extent,
              origin='lower', alpha=0.7)

    colors = plt.cm.tab10.colors
    for comp_id, (path, area_m2) in enumerate(all_paths):
        if not path:
            continue
        color = colors[comp_id % len(colors)]
        px, py = zip(*path)
        ax.plot(px, py, '-', color=color, linewidth=1.5, alpha=0.8,
                label=f'Room {comp_id + 1}: {len(path)} poses ({area_m2:.0f}m²)')
        ax.plot(path[0][0], path[0][1], 'o', color=color,
                markersize=8, zorder=5)

        arrow_step = max(1, len(path) // 25)
        for i in range(0, len(path) - arrow_step, arrow_step):
            dx = path[i + arrow_step][0] - path[i][0]
            dy = path[i + arrow_step][1] - path[i][1]
            length = math.hypot(dx, dy)
            if length > 0.05:
                ax.annotate(
                    '', xy=path[i + arrow_step], xytext=path[i],
                    arrowprops=dict(arrowstyle='->', color=color,
                                    lw=1.0, alpha=0.5),
                )

    ax.legend(loc='upper right', fontsize=9)
    ax.set_aspect('equal')
    total_len = sum(path_length(p) for p, _ in all_paths)
    total_poses = sum(len(p) for p, _ in all_paths)
    ax.set_title(
        f'Grid Boustrophedon Coverage — {total_poses} poses, {total_len:.1f}m')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved plot to {output_path}')
    plt.show()


def save_paths_to_yaml(all_paths, output_path):
    rooms = []
    for path, area_m2 in all_paths:
        room_data = {
            'area_m2': round(area_m2, 2),
            'coverage_path': [
                {'x': float(p[0]), 'y': float(p[1])} for p in path
            ]
        }
        rooms.append(room_data)
    data = {'rooms': rooms, 'num_rooms': len(rooms)}
    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)
    print(f'Saved {len(rooms)} rooms ({sum(len(p) for p, _ in all_paths)} poses) to {output_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Grid-based boustrophedon coverage path planner')
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--spacing', type=float, default=0.3)
    parser.add_argument('--min-room-area', type=float, default=1.0)
    parser.add_argument('--output', default='coverage_path.yaml')
    parser.add_argument('--save-plot')
    parser.add_argument('--no-display', action='store_true')
    args = parser.parse_args()

    print('Loading map...')
    map_img, resolution, (ox, oy, _), pgm_path = load_map(args.map_yaml)
    h, w = map_img.shape
    map_extent = [ox, ox + w * resolution, oy, oy + h * resolution]
    print(f'  Map: {w}x{h}, {resolution*100:.1f}cm/px, origin=({ox:.3f}, {oy:.3f})')

    print('Computing connected components on free space...')
    free_mask, labels, stats, centroids = compute_connected_components(map_img)

    free_area_px = np.sum(free_mask > 0)
    print(f'  Free space: {free_area_px} px ({free_area_px * resolution * resolution:.1f}m²)')

    min_area_px = args.min_room_area / (resolution * resolution)
    all_paths = []
    total_waypoints = 0

    for label_id in range(1, stats.shape[0]):
        area_px = stats[label_id, cv2.CC_STAT_AREA]
        if area_px < min_area_px:
            continue

        area_m2 = area_px * resolution * resolution
        comp_mask = (labels == label_id).astype(np.uint8)
        angle = compute_sweep_angle(comp_mask)

        print(f'Room {label_id}: area={area_m2:.1f}m², sweep={math.degrees(angle):.0f}°')

        path = grid_boustrophedon(comp_mask, h, (ox, oy),
                                   resolution, args.spacing, angle)

        total_waypoints += len(path)

        path = decimate_path(path, args.spacing * 0.5)

        print(f'  -> {len(path)} waypoints ({path_length(path):.1f}m)')
        all_paths.append((path, area_m2))

    total_poses = sum(len(p) for p, _ in all_paths)
    print(f'Total: {len(all_paths)} rooms, {total_poses} waypoints')

    save_paths_to_yaml(all_paths, args.output)

    if not args.no_display or args.save_plot:
        visualize(map_img, map_extent, all_paths, args.save_plot)

    print('Done.')


if __name__ == '__main__':
    main()
