#!/usr/bin/env python3
"""Check mapped occupied cells against bundled-world geometry at default spawn.

Ground truth is used only for this read-only assertion. Map/world axes are
assumed to coincide at the default spawn; this is not a localization input.
"""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import yaml

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    world = Path(__file__).resolve().parents[1] / 'src/smartcar_gazebo/worlds/smartcar_navigation.sdf'
    models = []
    config = world.parents[2] / 'smartcar_navigation/config/nav2_params.yaml'
    threshold = yaml.safe_load(config.read_text())['global_costmap']['global_costmap']['ros__parameters']['lethal_cost_threshold']
    for model in ET.parse(world).getroot().findall('.//model'):
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        geometry = model.find('link/collision/geometry')
        if geometry is None:
            continue
        if geometry.find('box') is not None:
            models.append(('box', pose, [float(v) for v in geometry.findtext('box/size').split()]))
        elif geometry.find('cylinder') is not None:
            models.append(('cylinder', pose, float(geometry.findtext('cylinder/radius'))))
    rclpy.init()
    node = rclpy.create_node('verify_map_geometry')
    maps = []
    node.create_subscription(OccupancyGrid, '/map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic() + 10.0
        while not maps and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not maps:
            raise RuntimeError('No map received')
        grid = maps[-1]
        distances = []
        uncertain_distances = []
        for i, value in enumerate(grid.data):
            if value < 50:
                continue
            x = grid.info.origin.position.x + (i % grid.info.width + 0.5) * grid.info.resolution
            y = grid.info.origin.position.y + (i // grid.info.width + 0.5) * grid.info.resolution
            candidates = []
            for kind, pose, size in models:
                dx, dy = x-pose[0], y-pose[1]
                c, s = math.cos(pose[5]), math.sin(pose[5])
                u, v = c*dx+s*dy, -s*dx+c*dy
                if kind == 'box':
                    candidates.append(math.hypot(max(abs(u)-size[0]/2, 0), max(abs(v)-size[1]/2, 0)))
                else:
                    candidates.append(max(math.hypot(u, v)-size, 0))
            if value >= threshold:
                distances.append(min(candidates))
            else:
                uncertain_distances.append(min(candidates))
        if not distances:
            raise RuntimeError('Map has no occupied cells')
        distances.sort()
        summary = {'occupied_cells': len(distances),
                   'occupied_threshold': threshold,
                   'uncertain_50_to_threshold_cells': len(uncertain_distances),
                   'uncertain_cells_over_10cm': sum(v > 0.1 for v in uncertain_distances),
                   'mean_distance_m': sum(distances)/len(distances),
                   'p95_distance_m': distances[int(0.95*len(distances))],
                   'max_distance_m': distances[-1],
                   'fraction_over_10cm': sum(v > 0.1 for v in distances)/len(distances)}
        summary['passed'] = summary['p95_distance_m'] <= 0.08 and summary['fraction_over_10cm'] <= 0.05
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary))
        return 0 if summary['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
