"""외부 자산 없이 상자 형상으로 8×6m 지붕 없는 창고 월드를 만든다."""
from pathlib import Path
import xml.etree.ElementTree as ET


# 이름, 중심 x/y/z, 폭/깊이/높이, 색상. 바닥 상면은 z=0이다.
BOXES = [
    ('floor', 0, 0, -.05, 8.2, 6.2, .1, '.65 .65 .65 1'),
    ('north_wall', 0, 3, .75, 8.2, .1, 1.5, '.8 .8 .75 1'),
    ('south_wall', 0, -3, .75, 8.2, .1, 1.5, '.8 .8 .75 1'),
    ('west_wall', -4, 0, .75, .1, 6, 1.5, '.8 .8 .75 1'),
    ('east_wall', 4, 0, .75, .1, 6, 1.5, '.8 .8 .75 1'),
    ('rack_a', -1.8, -.6, .6, 1.6, .6, 1.2, '.2 .35 .65 1'),
    ('rack_b', .8, -.6, .6, 1.6, .6, 1.2, '.2 .35 .65 1'),
    ('rack_c', -1.8, 1.6, .6, 1.6, .6, 1.2, '.2 .35 .65 1'),
    ('rack_d', .8, 1.6, .6, 1.6, .6, 1.2, '.2 .35 .65 1'),
    # 오른쪽 적재실 출입구는 y=-0.6..0.6의 폭 1.2m이다.
    ('partition_south', 2.5, -1.8, .75, .1, 2.4, 1.5, '.8 .8 .75 1'),
    ('partition_north', 2.5, 1.8, .75, .1, 2.4, 1.5, '.8 .8 .75 1'),
    ('crate_a', 3.4, 1.8, .3, .5, .6, .6, '.65 .4 .15 1'),
    ('crate_b', 3.4, -1.8, .3, .5, .6, .6, '.65 .4 .15 1'),
]
SPAWN = (-3., -2., .01, 0.)


def build_world():
    root = ET.Element('sdf', version='1.6')
    world = ET.SubElement(root, 'world', name='simple_warehouse')
    ET.SubElement(world, 'gravity').text = '0 0 -9.8'
    physics = ET.SubElement(world, 'physics', name='default_physics', type='ode', default='1')
    for tag, text in [('max_step_size', '.001'), ('real_time_update_rate', '1000'),
                      ('real_time_factor', '1')]:
        ET.SubElement(physics, tag).text = text
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '.6 .6 .6 1'
    ET.SubElement(scene, 'background').text = '.75 .8 .85 1'
    ET.SubElement(scene, 'shadows').text = 'false'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '.8 .8 .8 1'
    ET.SubElement(light, 'specular').text = '.1 .1 .1 1'
    ET.SubElement(light, 'direction').text = '-.5 .1 -1'
    ET.SubElement(light, 'cast_shadows').text = 'false'
    for name, x, y, z, sx, sy, sz, color in BOXES:
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 0'
        link = ET.SubElement(model, 'link', name='body')
        for kind in ('collision', 'visual'):
            item = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = f'{sx} {sy} {sz}'
            if kind == 'visual':
                material = ET.SubElement(item, 'material')
                ET.SubElement(material, 'ambient').text = color
                ET.SubElement(material, 'diffuse').text = color
                ET.SubElement(item, 'cast_shadows').text = 'false'
    gui = ET.SubElement(world, 'gui')
    camera = ET.SubElement(gui, 'camera', name='overview')
    ET.SubElement(camera, 'pose').text = '0 -8 9 0 .85 1.57'
    ET.indent(root)
    return ET.tostring(root, encoding='unicode', xml_declaration=True)


def main():
    target = Path.home()/'ros2_practice_ws/warehouse_worlds/simple_warehouse.world'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(build_world(), encoding='utf-8')
    print('단순 창고 생성:', target)
    print('외부 모델·메시·텍스처 없음 / 지붕 없음 / 정적 상자 13개')
    print('출발 위치 x=-3.0, y=-2.0 / 적재실 출입구 폭 1.2m')


if __name__ == '__main__':
    main()
