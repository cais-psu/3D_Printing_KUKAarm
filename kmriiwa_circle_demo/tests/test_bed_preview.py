import json
from pathlib import Path
import tempfile
import unittest

from kmriiwa_circle_demo.bed_preview import load_bed_preview
from kmriiwa_circle_demo.geometry import compose_pose, inverse_pose


CONFIG = Path(__file__).resolve().parents[1] / 'config/bed_preview.json'


class BedPreviewTests(unittest.TestCase):
    def test_measured_footprint_and_three_parallel_closed_circles(self):
        bed = load_bed_preview(CONFIG)
        self.assertAlmostEqual(bed.surface.position[0], -.339525)
        self.assertAlmostEqual(bed.surface.position[1], -.425)
        self.assertAlmostEqual(bed.edge_clearance, .12107, places=5)
        bed_from_world = inverse_pose(bed.surface)
        points = [compose_pose(bed_from_world, pose).position
                  for pose in (bed.path.start_tcp,) + bed.path.tcp_waypoints]
        self.assertEqual(len(points), 183)
        for layer in range(3):
            ring = points[layer * 61:(layer + 1) * 61]
            self.assertEqual(ring[0], ring[-1])
            for x, y, z in ring:
                self.assertAlmostEqual(x*x + y*y, .02**2)
                self.assertAlmostEqual(z, .02 + .005*layer)
        ee_from_tcp = compose_pose(inverse_pose(bed.start_ee), bed.path.start_tcp)
        for axis, expected in zip(ee_from_tcp.position, (-.060, 0., .1557)):
            self.assertAlmostEqual(axis, expected)

    def test_reject_circle_outside_edge_margin(self):
        data = json.loads(CONFIG.read_text())
        data['radius_mm'] = 120
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'bed.json'
            config.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'edge margin'):
                load_bed_preview(config)
