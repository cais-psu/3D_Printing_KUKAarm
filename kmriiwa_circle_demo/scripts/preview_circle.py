#!/usr/bin/env python3
"""Export an offline geometric example. Never imports ROS or sends commands."""
import argparse
import csv
import json
from pathlib import Path

from kmriiwa_circle_demo.bed_preview import load_bed_preview
from kmriiwa_circle_demo.geometry import Pose, generate_circle_plan, path_length


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--radius', type=float, default=.02)
    parser.add_argument('--layers', type=int, default=3)
    parser.add_argument('--layer-step', type=float, default=.005)
    parser.add_argument('--samples', type=int, default=60)
    parser.add_argument('--plane', choices=('xy', 'xz', 'yz'), default='xy')
    parser.add_argument('--bed', type=Path, help='preview the measured footprint and circles from this JSON file')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    bed = load_bed_preview(args.bed) if args.bed else None
    plan = bed.path if bed else generate_circle_plan(
        Pose((0., 0., 0.), (0., 0., 0., 1.)), radius=args.radius, layers=args.layers,
        layer_step=args.layer_step, samples_per_circle=args.samples, plane=args.plane)
    args.output.mkdir(parents=True, exist_ok=True)
    poses = (plan.start_tcp,) + plan.tcp_waypoints
    with (args.output / 'offline_path.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(('index', 'x_m', 'y_m', 'z_m'))
        writer.writerows((i,) + p.position for i, p in enumerate(poses))
    report = {'offline_only': True, 'initial_pose_is_synthetic': True,
              'radius_m': args.radius, 'layers': args.layers, 'layer_step_m': args.layer_step,
              'geometric_targets': len(plan.tcp_waypoints), 'path_length_m': path_length(poses),
              'minimum_xyz_m': [min(p.position[i] for p in poses) for i in range(3)],
              'maximum_xyz_m': [max(p.position[i] for p in poses) for i in range(3)],
              'plane': args.plane, 'layer_normal': plan.plane_normal}
    if bed:
        report.update(bed.summary(), radius_m=bed.radius, layers=bed.layers, layer_step_m=bed.layer_step,
                      plane='estimated bed surface')
    (args.output / 'offline_summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if bed:
        write_bed_svg(args.output / 'offline_preview.svg', bed)
        print(json.dumps(report, indent=2))
        return 0
    # Isometric SVG uses only standard-library geometry, no plotting dependencies.
    projected = [(p.position[0] - .5 * p.position[1], .5 * p.position[1] + p.position[2]) for p in poses]
    xs, ys = [p[0] for p in projected], [p[1] for p in projected]
    scale = 430 / max(max(xs)-min(xs), max(ys)-min(ys), .001)
    coords = ' '.join('%.2f,%.2f' % (50 + (x-min(xs))*scale, 480-(y-min(ys))*scale) for x, y in projected)
    (args.output / 'offline_preview.svg').write_text(
        '<svg xmlns="http://www.w3.org/2000/svg" width="600" height="550" viewBox="0 0 600 550">'
        '<rect width="600" height="550" fill="white"/>'
        '<text x="24" y="28" font-family="sans-serif" font-size="18">Air-circle geometry (synthetic start; metres)</text>'
        '<polyline points="' + coords + '" fill="none" stroke="#2479ba" stroke-width="2"/>'
        '<text x="24" y="530" font-family="sans-serif" font-size="14">No robot / ROS / heater / extruder commands</text></svg>')
    print(json.dumps(report, indent=2))
    return 0


def write_bed_svg(output, bed):
    """Top-down view with World +X up and +Y left, matching the bed labels."""
    center_x, center_y, _ = bed.surface.position
    extent = max(max(abs(x - center_x), abs(y - center_y)) for x, y, _ in bed.corners)
    scale = 202 / extent

    def project(point):
        x, y, _ = point
        return 300 - (y - center_y) * scale, 284 - (x - center_x) * scale

    def coords(points):
        return ' '.join('%.2f,%.2f' % project(point) for point in points)

    circle = (bed.path.start_tcp,) + bed.path.tcp_waypoints[:bed.samples_per_circle]
    first_x, first_y = project(circle[0].position)
    output.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="600" height="550" viewBox="0 0 600 550">
  <rect width="600" height="550" fill="white"/>
  <g font-family="sans-serif" fill="#192e3e">
    <text x="26" y="30" font-size="20">Bed-centered circle preview</text>
    <text x="26" y="52" font-size="13">Top view · World +X up (back), +Y left</text>
    <polygon points="{coords(bed.corners)}" fill="#eaf3f8" stroke="#658194" stroke-width="2"/>
    <polyline points="{coords(p.position for p in circle)}" fill="none" stroke="#1678ac" stroke-width="3"/>
    <circle cx="{first_x:.2f}" cy="{first_y:.2f}" r="4" fill="#e27629"/>
    <text x="{first_x + 10:.2f}" y="{first_y - 3:.2f}" font-size="13">Start</text>
    <text x="26" y="502" font-size="14">{bed.layers} layers · {2000 * bed.radius:g} mm diameter · {1000 * bed.clearance:g} mm estimated clearance above bed</text>
    <text x="26" y="526" font-size="13">CAD nozzle offset and provisional bed height. Preview only.</text>
  </g>
</svg>
''')


if __name__ == '__main__':
    raise SystemExit(main())
