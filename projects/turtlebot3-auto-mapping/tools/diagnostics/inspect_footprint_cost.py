"""Nav2 로봇 윤곽 경계의 로컬 비용 지도 셀을 읽기만 한다. 이동 명령 없음."""
import math
import time
from collections import Counter
from types import SimpleNamespace as NS

import rclpy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from rclpy.qos import qos_profile_sensor_data


def inspect(grid, polygon, raw=False):
    if grid.header.frame_id != polygon.header.frame_id:
        return f'좌표계 불일치: 지도={grid.header.frame_id}, 윤곽={polygon.header.frame_id}'
    info = grid.info
    q = info.origin.orientation
    angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(angle), math.sin(angle)
    cells = set()
    points = list(polygon.polygon.points)
    if not points or info.resolution <= 0:
        return '지도 또는 윤곽 데이터가 비어 있습니다.'
    for p, end in zip(points, points[1:]+points[:1]):
        steps = max(1, math.ceil(math.hypot(end.x-p.x, end.y-p.y)/(info.resolution/5)))
        for k in range(steps+1):
            x = p.x+(end.x-p.x)*k/steps-info.origin.position.x
            y = p.y+(end.y-p.y)*k/steps-info.origin.position.y
            cells.add((math.floor((c*x+s*y)/info.resolution),
                       math.floor((-s*x+c*y)/info.resolution)))
    values = Counter()
    categories = Counter()
    for x, y in cells:
        if not 0 <= x < info.width or not 0 <= y < info.height:
            categories['지도밖'] += 1
            continue
        value = grid.data[y*info.width+x]
        values[value] += 1
        if raw:
            category = ('미탐색255' if value == 255 else '치명장애물254' if value == 254
                        else '내접영역253' if value == 253 else '빈공간0' if value == 0
                        else '일반비용1~252')
        else:
            category = ('미탐색' if value < 0 else '치명장애물100' if value == 100
                        else '내접영역99' if value == 99 else '빈공간0' if value == 0
                        else '일반비용1~98')
        categories[category] += 1
    return f'윤곽 경계 셀 분류: {dict(categories)}\n경계 셀 값: {dict(sorted(values.items()))}'


def stamp(header):
    return header.stamp.sec+header.stamp.nanosec/1e9


def raw_grid(message):
    meta = message.metadata
    return NS(header=message.header, data=message.data,
              info=NS(origin=meta.origin, resolution=meta.resolution,
                      width=meta.size_x, height=meta.size_y))


def rotated(polygon, degrees):
    """현재 원형 윤곽의 중심을 기준으로 회전한다. 비대칭 몸체에는 사용하지 않는다."""
    points = polygon.polygon.points
    cx = sum(p.x for p in points)/len(points)
    cy = sum(p.y for p in points)/len(points)
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    new = [NS(x=cx+c*(p.x-cx)-s*(p.y-cy), y=cy+s*(p.x-cx)+c*(p.y-cy)) for p in points]
    return NS(header=polygon.header, polygon=NS(points=new))


def main():
    rclpy.init()
    node = rclpy.create_node('footprint_cost_inspector')
    received = {}
    def map_callback(msg):
        received['map'] = msg
    def footprint_callback(msg):
        received['footprint'] = msg
    def raw_callback(msg):
        received['raw'] = msg
    subscriptions = [
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', map_callback, qos_profile_sensor_data),
        node.create_subscription(PolygonStamped, '/local_costmap/published_footprint', footprint_callback, qos_profile_sensor_data),
        node.create_subscription(Costmap, '/local_costmap/costmap_raw', raw_callback, qos_profile_sensor_data),
    ]
    print('최대 15초 관측합니다. Gazebo 재생, Nav2 실행, 탐색 정지 상태로 두세요.', flush=True)
    end = time.monotonic()+15
    ready = None
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(received) == 3:
                ready = ready or time.monotonic()
                if time.monotonic()-ready >= 3:
                    break
        if len(received) != 3:
            print('수신 상태:', sorted(received))
            return
        grid, footprint = received['map'], received['footprint']
        print(f'지도 시각: {stamp(grid.header):.3f}, 윤곽 시각: {stamp(footprint.header):.3f}')
        if abs(stamp(grid.header)-stamp(footprint.header)) > 2:
            print('시각 차이가 큽니다. 아래 기하 결과보다 시간 동기 문제를 먼저 확인하세요.')
        print(inspect(grid, footprint))
        raw = raw_grid(received['raw'])
        print(f'원본 비용 지도 시각: {stamp(raw.header):.3f}')
        if abs(stamp(raw.header)-stamp(footprint.header)) > 2:
            print('원본 지도와 윤곽 시각 차이가 큽니다. 먼저 시간 상태를 확인하세요.')
        for degrees in range(0, 91, 5):
            print(f'--- 원본 지도 / 반시계 {degrees}도 ---')
            print(inspect(raw, rotated(footprint, degrees), raw=True))
        print('현재 원형 윤곽 중심 기준의 5도 간격 근사 검사입니다. 실제 회전 명령이나 안전 보장이 아닙니다.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
