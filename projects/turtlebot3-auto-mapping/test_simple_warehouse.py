"""단순 창고의 외부 자산 독립성과 출발 여유·통로 구조를 검사한다."""
import unittest
import xml.etree.ElementTree as ET
from prepare_simple_warehouse import BOXES, SPAWN, build_world


class SimpleWarehouseTests(unittest.TestCase):
    def test_world_has_only_matching_primitive_visuals_and_collisions(self):
        world = ET.fromstring(build_world()).find('world')
        self.assertEqual(len(world.findall('model')), 13)
        self.assertEqual(list(world.iter('uri')), [])
        self.assertEqual(list(world.iter('mesh')), [])
        for model in world.findall('model'):
            self.assertEqual(model.findtext('static'), 'true')
            self.assertEqual(model.findtext('link/visual/geometry/box/size'),
                             model.findtext('link/collision/geometry/box/size'))

    def test_spawn_has_footprint_clearance_and_door_is_open(self):
        x, y = SPAWN[:2]
        for name, bx, by, bz, sx, sy, sz, color in BOXES:
            if name == 'floor':
                continue
            self.assertTrue(abs(x-bx) > sx/2+.3 or abs(y-by) > sy/2+.3, name)
            # 출입구 중심에서도 반경 0.3m에 해당하는 통과 여유를 확인한다.
            self.assertTrue(abs(2.5-bx) > sx/2+.3 or abs(by) > sy/2+.3, name)
        self.assertAlmostEqual(next(b for b in BOXES if b[0]=='partition_north')[2]-1.2, .6)


if __name__ == '__main__':
    unittest.main()
