"""소형 창고의 월드·모델·메시 참조 검사. 외부 다운로드나 Gazebo 실행은 하지 않는다."""
import hashlib
from pathlib import Path
from tempfile import TemporaryDirectory
from contextlib import contextmanager
import unittest
import xml.etree.ElementTree as ET
from prepare_small_warehouse import inspect_assets, create_roofless_world, apply_vm_lite


@contextmanager
def workspace_temp():
    base = Path(__file__).resolve().parent
    with TemporaryDirectory(prefix='.warehouse_test_', dir=base) as folder:
        # 자동 정리 전에 임시 폴더가 작업공간 안인지 확인한다.
        Path(folder).resolve().relative_to(base)
        yield folder


class WarehouseAssetsTests(unittest.TestCase):
    def test_vm_lite_keeps_structure_and_physics_step_but_removes_props(self):
        with workspace_temp() as folder:
            path = Path(folder)/'lite.world'
            models = ['WallB_01', 'GroundB_01', 'ShelfE_01', 'ClutteringA_01', 'Bucket_01']
            path.write_text('<sdf><world name="default"><physics><max_step_size>0.001</max_step_size></physics>'
                '<light name="light"><cast_shadows>1</cast_shadows></light>'+''.join(
                '<model name="'+name+'"><include><uri>model://aws_robomaker_warehouse_'+name+
                '</uri></include></model>' for name in models)+'</world></sdf>')
            self.assertEqual(apply_vm_lite(path), ['ClutteringA_01', 'Bucket_01'])
            world = ET.parse(path).getroot().find('world')
            self.assertEqual([n.get('name') for n in world.findall('model')], models[:3])
            self.assertEqual(world.findtext('physics/max_step_size'), '0.001')
            self.assertEqual(world.findtext('physics/real_time_update_rate'), '500')
            self.assertEqual(world.findtext('scene/shadows'), 'false')
            self.assertEqual(world.findtext('light/cast_shadows'), 'false')

    def test_roof_removed_and_other_world_elements_preserved(self):
        with workspace_temp() as folder:
            original = Path(folder)/'original.world'
            original.write_text('<sdf version="1.6"><world name="default">'
                '<model name="roof"><include><uri>model://aws_robomaker_warehouse_RoofB_01</uri></include></model>'
                '<model name="wall"><pose>1 2 0 0 0 0</pose></model><light name="ceiling_light"/>'
                '<physics name="physics"/></world></sdf>')
            before = original.read_bytes()
            target = Path(folder)/'generated/no_roof.world'
            self.assertEqual(create_roofless_world(original, target), ['roof'])
            world = ET.parse(target).getroot().find('world')
            self.assertEqual([m.get('name') for m in world.findall('model')], ['wall'])
            self.assertEqual(world.findtext('model/pose'), '1 2 0 0 0 0')
            self.assertIsNotNone(world.find('light'))
            self.assertIsNotNone(world.find('physics'))
            self.assertEqual(original.read_bytes(), before)

    def test_roof_conversion_rejects_overwrite_or_unexpected_world(self):
        with workspace_temp() as folder:
            world = Path(folder)/'world.world'
            world.write_text('<sdf><world name="default"/></sdf>')
            with self.assertRaisesRegex(ValueError, '덮어쓸'):
                create_roofless_world(world, world)
            with self.assertRaisesRegex(ValueError, '지붕'):
                create_roofless_world(world, Path(folder)/'out.world')

    def build_assets(self, root):
        world = root/'worlds/small_warehouse/small_warehouse.world'
        world.parent.mkdir(parents=True)
        world.write_text('<sdf version="1.6"><world name="default"><include><uri>model://rack</uri></include></world></sdf>')
        rack = root/'models/rack'
        (rack/'meshes').mkdir(parents=True)
        (rack/'meshes/rack.dae').write_text('<COLLADA/>')
        (rack/'model.config').write_text('<model><sdf version="1.6">model.sdf</sdf></model>')
        (rack/'model.sdf').write_text('<sdf version="1.6"><model name="rack"><link name="body"><visual name="v"><geometry><mesh><uri>file://models/rack/meshes/rack.dae</uri></mesh></geometry></visual></link></model></sdf>')
        return world, rack

    def test_world_model_and_root_relative_file_uris_are_resolved(self):
        with workspace_temp() as folder:
            root = Path(folder)/'assets with spaces'
            world, _ = self.build_assets(root)
            original = world.read_bytes()
            report = inspect_assets(root)
            self.assertEqual(report['sdf_files_checked'], 2)
            self.assertEqual(report['model_names'], ['rack'])
            self.assertEqual(report['world_sha256'], hashlib.sha256(original).hexdigest())
            self.assertEqual(world.read_bytes(), original)
            self.assertEqual(report['spawn'], [0., 0., .01, 0.])

    def test_missing_mesh_is_reported_before_launch(self):
        with workspace_temp() as folder:
            root = Path(folder)
            _, rack = self.build_assets(root)
            (rack/'meshes/rack.dae').unlink()
            with self.assertRaisesRegex(ValueError, 'rack.dae'):
                inspect_assets(root)

    def test_missing_model_config_is_reported(self):
        with workspace_temp() as folder:
            root = Path(folder)
            _, rack = self.build_assets(root)
            (rack/'model.config').unlink()
            with self.assertRaisesRegex(ValueError, 'model.config'):
                inspect_assets(root)

    def test_model_scheme_mesh_paths_are_supported(self):
        with workspace_temp() as folder:
            root = Path(folder)
            _, rack = self.build_assets(root)
            path = rack/'model.sdf'
            path.write_text(path.read_text().replace('file://models/', 'model://'))
            self.assertEqual(inspect_assets(root)['model_names'], ['rack'])

    def test_reference_outside_asset_root_is_rejected(self):
        with workspace_temp() as folder:
            root = Path(folder)
            _, rack = self.build_assets(root)
            path = rack/'model.sdf'
            path.write_text(path.read_text().replace('file://models/rack/meshes/rack.dae', 'file://../../../../outside.dae'))
            with self.assertRaisesRegex(ValueError, '밖'):
                inspect_assets(root)

    def test_wrong_branch_layout_is_rejected(self):
        with workspace_temp() as folder:
            with self.assertRaisesRegex(ValueError, 'ros2'):
                inspect_assets(Path(folder))

    def test_malformed_sdf_does_not_pass(self):
        with workspace_temp() as folder:
            root = Path(folder)
            _, rack = self.build_assets(root)
            (rack/'model.sdf').write_text('<sdf>')
            with self.assertRaises(ET.ParseError):
                inspect_assets(root)


if __name__ == '__main__':
    unittest.main()
