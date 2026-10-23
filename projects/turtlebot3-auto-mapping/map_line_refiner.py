"""반복 관측된 지도 경계를 제한 오차 안에서 직선으로 단순화해 별도 표시한다."""
import json
import math
from pathlib import Path
from xml.sax.saxutils import escape


def point_distance(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    t = max(0., min(1., ((point[0]-start[0])*dx+(point[1]-start[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(point[0]-start[0]-t*dx, point[1]-start[1]-t*dy)


def simplify(points, tolerance):
    """반복형 RDP. 입력점에서 근사 선분까지의 거리로 허용 오차를 제한한다."""
    if len(points) < 3:
        return points[:]
    keep = {0, len(points)-1}
    pending = [(0, len(points)-1)]
    while pending:
        first, last = pending.pop()
        if last-first < 2:
            continue
        distance, index = max((point_distance(points[i], points[first], points[last]), i)
                              for i in range(first+1, last))
        if distance > tolerance:
            keep.add(index)
            pending.extend(((first, index), (index, last)))
    return [points[i] for i in sorted(keep)]


def trace_edges(edges):
    """격자 꼭짓점 그래프에서 분기점과 단절을 넘어가지 않는 체인을 만든다."""
    adjacent = {}
    remaining = set()
    for a, b in edges:
        adjacent.setdefault(a, set()).add(b)
        adjacent.setdefault(b, set()).add(a)
        remaining.add(tuple(sorted((a, b))))
    def walk(a, b):
        chain = [a, b]
        remaining.remove(tuple(sorted((a, b))))
        while len(adjacent[b]) == 2 and b != chain[0]:
            next_point = next(p for p in adjacent[b] if p != a)
            edge = tuple(sorted((b, next_point)))
            if edge not in remaining:
                break
            remaining.remove(edge)
            chain.append(next_point)
            a, b = b, next_point
        return chain
    chains = []
    for a in sorted(adjacent):
        if len(adjacent[a]) != 2:
            for b in sorted(adjacent[a]):
                if tuple(sorted((a, b))) in remaining:
                    chains.append(walk(a, b))
    while remaining:
        a, b = min(remaining)
        chain = walk(a, b)
        # 닫힌 경계를 먼 꼭짓점에서 나눠 폐곡선의 시작·끝이 같아지는 문제를 피한다.
        if chain[0] == chain[-1]:
            split = max(range(1, len(chain)-1), key=lambda i:
                        math.dist(chain[0], chain[i]))
            chains.extend((chain[:split+1], chain[split:]))
        else:
            chains.append(chain)
    return chains


class LineRefiner:
    def __init__(self, tolerance=.075, min_length=.35, observations=3,
                 occupied=65, free=25):
        if not (0 < tolerance <= .2 and min_length > 0 and observations >= 1 and
                0 <= free < occupied <= 100):
            raise ValueError('직선화 오차·길이·반복 횟수·점유 기준을 확인하세요.')
        self.tolerance, self.min_length = tolerance, min_length
        self.observations, self.occupied, self.free = observations, occupied, free
        self.streak, self.signature, self.last_stamp = {}, None, None

    def process(self, data, width, height, resolution, origin=(0., 0., 0.), stamp=None):
        if width <= 0 or height <= 0 or len(data) != width*height or resolution <= 0:
            raise ValueError('지도 크기 또는 해상도가 올바르지 않습니다.')
        ox, oy, yaw = origin
        if not all(math.isfinite(v) for v in (resolution, ox, oy, yaw)):
            raise ValueError('지도 좌표에 유한하지 않은 값이 있습니다.')
        c, s = math.cos(yaw), math.sin(yaw)
        def world(x, y):
            x, y = x*resolution, y*resolution
            return (ox+c*x-s*y, oy+s*x+c*y)
        signature = (round(resolution, 9), round(yaw, 6))
        if signature != self.signature:
            self.streak, self.last_stamp, self.signature = {}, None, signature
        elif stamp is not None and self.last_stamp is not None and stamp < self.last_stamp:
            self.streak, self.last_stamp = {}, None
        occupied = [i for i, value in enumerate(data) if value >= self.occupied]
        keys = {}
        for i in occupied:
            x, y = world(i % width+.5, i//width+.5)
            keys[i] = (round(x/resolution, 4), round(y/resolution, 4))
        if stamp is None or stamp != self.last_stamp:
            self.streak = {key: min(self.observations, self.streak.get(key, 0)+1)
                           for key in keys.values()}
            self.last_stamp = stamp
        edges = []
        stable_cells = 0
        for i in occupied:
            if self.streak.get(keys[i], 0) < self.observations:
                continue
            stable_cells += 1
            x, y = i % width, i//width
            sides = [(-1, 0, (x, y), (x, y+1)), (1, 0, (x+1, y), (x+1, y+1)),
                     (0, -1, (x, y), (x+1, y)), (0, 1, (x, y+1), (x+1, y+1))]
            for dx, dy, a, b in sides:
                nx, ny = x+dx, y+dy
                if 0 <= nx < width and 0 <= ny < height and 0 <= data[ny*width+nx] <= self.free:
                    edges.append((a, b))
        lines = []
        for chain in trace_edges(edges):
            points = [world(*point) for point in chain]
            reduced = simplify(points, self.tolerance)
            lines.extend((a, b) for a, b in zip(reduced, reduced[1:])
                         if math.dist(a, b) >= self.min_length)
        return dict(lines=lines, stable_cells=stable_cells, boundary_edges=len(edges),
                    tolerance_m=self.tolerance, min_length_m=self.min_length,
                    required_observations=self.observations,
                    note='반복 관측된 장애물/빈 공간 경계의 벡터 근사입니다. SLAM 보정·확정 판정·주행용 지도가 아닙니다.')


def svg_text(lines, frame='map'):
    points = [point for line in lines for point in line]
    if not points:
        return '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1 1"/>\n'
    x0, x1 = min(p[0] for p in points)-.2, max(p[0] for p in points)+.2
    y0, y1 = min(p[1] for p in points)-.2, max(p[1] for p in points)+.2
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0} {-y1} {x1-x0} {y1-y0}">',
             '<title>'+escape(frame)+' frame: wall boundary lines, metres</title>',
             f'<rect x="{x0}" y="{-y1}" width="{x1-x0}" height="{y1-y0}" fill="white"/>',
             '<g stroke="#164f88" stroke-width="0.025" fill="none">']
    for a, b in lines:
        parts.append(f'<line x1="{a[0]}" y1="{-a[1]}" x2="{b[0]}" y2="{-b[1]}"/>')
    return '\n'.join(parts+['</g></svg>\n'])


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point

    class RefinerNode(Node):
        def __init__(self):
            super().__init__('map_line_refiner')
            def param(name, default):
                return self.declare_parameter(name, default).value
            self.refiner = LineRefiner(param('tolerance', .075), param('min_length', .35),
                                       param('observations', 3), param('occupied_threshold', 65),
                                       param('free_threshold', 25))
            self.pending = None
            self.frame = None
            self.exports = Path(param('output_dir', str(Path.home()/'ros2_practice_ws/map_refined'))).expanduser()
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(Marker, '/map_lines', qos)
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if param('map_transient_local', False)
                else DurabilityPolicy.VOLATILE)
            self.subscription = self.create_subscription(OccupancyGrid, '/map', self.receive, map_qos)
            self.timer = self.create_timer(2., self.update)
            self.get_logger().info('직선 지도 준비: /map 수신 → /map_lines 표시, 원본 /map 유지')

        def receive(self, msg):
            self.pending = msg

        def update(self):
            if self.pending is None:
                return
            msg, self.pending = self.pending, None
            try:
                if msg.header.frame_id != self.frame:
                    self.refiner.streak, self.refiner.last_stamp = {}, None
                    self.frame = msg.header.frame_id
                q = msg.info.origin.orientation
                norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                if norm < 1e-6 or abs(q.x/norm) > 1e-5 or abs(q.y/norm) > 1e-5:
                    raise ValueError('평면 지도의 원점 quaternion이 필요합니다.')
                yaw = 2*math.atan2(q.z/norm, q.w/norm)
                p = msg.info.origin.position
                result = self.refiner.process(msg.data, msg.info.width, msg.info.height,
                    msg.info.resolution, (p.x, p.y, yaw),
                    msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)
                marker = Marker()
                marker.header = msg.header
                marker.ns, marker.id = 'refined_boundary', 0
                marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
                marker.pose.orientation.w = 1.
                marker.scale.x = .025
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = .05, .35, 1., 1.
                for line in result['lines']:
                    for x, y in line:
                        point = Point()
                        point.x, point.y, point.z = float(x), float(y), .04
                        marker.points.append(point)
                self.publisher.publish(marker)
                result.update(frame=msg.header.frame_id, stamp=dict(sec=msg.header.stamp.sec,
                              nanosec=msg.header.stamp.nanosec), resolution=msg.info.resolution)
                self.exports.mkdir(parents=True, exist_ok=True)
                (self.exports/'map_lines.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
                (self.exports/'map_lines.svg').write_text(svg_text(result['lines'], msg.header.frame_id), encoding='utf-8')
                self.get_logger().info(f"직선 {len(result['lines'])}개 / 반복 관측 장애물 셀 {result['stable_cells']}개")
            except (ValueError, OSError) as error:
                self.get_logger().error('직선화 처리 실패: '+str(error))

    rclpy.init()
    node = RefinerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
