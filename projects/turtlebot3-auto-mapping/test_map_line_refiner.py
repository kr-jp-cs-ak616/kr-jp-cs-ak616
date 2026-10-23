"""직선화의 단절 보존·좌표 변환·반복 관측 조건을 검증한다."""
import math
import unittest
import xml.etree.ElementTree as ET
from map_line_refiner import LineRefiner, simplify, point_distance, svg_text


class LineRefinerTests(unittest.TestCase):
    def grid(self):
        data = [0]*1200
        for x in range(5, 55):
            data[10*60+x] = 100
        return data

    def process(self, refiner, data, stamp=1, origin=(0., 0., 0.)):
        return refiner.process(data, 60, 20, .05, origin, stamp)

    def test_straight_wall_reduces_to_two_long_faces(self):
        result = self.process(LineRefiner(observations=1), self.grid())
        self.assertEqual(len(result['lines']), 2)
        self.assertTrue(all(abs(math.dist(a, b)-2.5) <= result['tolerance_m'] for a, b in result['lines']))

    def test_same_stamp_does_not_count_as_new_observation(self):
        refiner = LineRefiner()
        for _ in range(4):
            self.assertEqual(self.process(refiner, self.grid(), 1)['lines'], [])
        self.assertEqual(self.process(refiner, self.grid(), 2)['lines'], [])
        self.assertEqual(len(self.process(refiner, self.grid(), 3)['lines']), 2)

    def test_unknown_only_boundaries_are_not_invented(self):
        data = [-1]*1200
        for x in range(5, 55):
            data[10*60+x] = 100
        self.assertEqual(self.process(LineRefiner(observations=1), data)['lines'], [])

    def test_door_gap_is_not_bridged(self):
        data = self.grid()
        for x in range(25, 35):
            data[10*60+x] = 0
        result = self.process(LineRefiner(observations=1), data)
        for a, b in result['lines']:
            self.assertFalse(min(a[0], b[0]) < 1.25 and max(a[0], b[0]) > 1.75)
        self.assertEqual(len(result['lines']), 4)

    def test_origin_rotation_transforms_endpoints(self):
        refiner = LineRefiner(observations=1)
        base = self.process(refiner, self.grid())['lines']
        rotated = self.process(refiner, self.grid(), 2, (2., 3., math.pi/2))['lines']
        self.assertEqual(len(base), len(rotated))
        for line, transformed in zip(base, rotated):
            for (x, y), (rx, ry) in zip(line, transformed):
                self.assertAlmostEqual(rx, 2-y)
                self.assertAlmostEqual(ry, 3+x)

    def test_disappearance_and_time_reset_restart_stability(self):
        refiner = LineRefiner(observations=2)
        self.process(refiner, self.grid(), 1)
        self.assertTrue(self.process(refiner, self.grid(), 2)['lines'])
        self.process(refiner, [0]*1200, 3)
        self.assertFalse(self.process(refiner, self.grid(), 4)['lines'])
        self.assertTrue(self.process(refiner, self.grid(), 5)['lines'])
        self.assertFalse(self.process(refiner, self.grid(), 0)['lines'])

    def test_staircase_is_simplified_with_bounded_error(self):
        points = [(i*.05, .05*(i//10)) for i in range(50)]
        result = simplify(points, .075)
        self.assertLess(len(result), len(points)//4)
        for point in points:
            self.assertLessEqual(min(point_distance(point, a, b) for a, b in zip(result, result[1:])), .075+1e-9)

    def test_svg_is_vector_and_empty_map_clears_lines(self):
        result = self.process(LineRefiner(observations=1), self.grid())
        root = ET.fromstring(svg_text(result['lines'], 'map & test'))
        self.assertEqual(len(root.findall('{http://www.w3.org/2000/svg}g/{http://www.w3.org/2000/svg}line')), 2)
        self.assertEqual(self.process(LineRefiner(observations=1), [0]*1200)['lines'], [])


if __name__ == '__main__':
    unittest.main()
