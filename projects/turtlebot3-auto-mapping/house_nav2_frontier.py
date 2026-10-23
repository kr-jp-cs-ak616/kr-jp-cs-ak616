"""지도 미탐색 경계의 좌표를 Nav2 액션 목표로 전송한다. cmd_vel은 발행하지 않는다."""
import json
import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from prepare_house_nav2_frontier import WAFFLE_PI_FOOTPRINT


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def transform(point, pose, inverse=False):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    if inverse:
        x, y = point[0]-pose[0], point[1]-pose[1]
        return (c*x+s*y, -s*x+c*y)
    return (pose[0]+c*point[0]-s*point[1], pose[1]+s*point[0]+c*point[1])


def footprint_bounds(text, padding=.01):
    points = json.loads(text)
    if points == []:
        return None
    if len(points) != 4 or not 0 <= padding <= .1 or any(
            len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('goal_footprint는 사각형 꼭짓점 4개, goal_padding은 0~0.1m가 필요합니다.')
    xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
    ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
    if not xmin < 0 < xmax or not ymin < 0 < ymax or {tuple(p) for p in points} != {
            (xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)}:
        raise ValueError('goal_footprint는 원점을 포함하는 축 정렬 사각형이어야 합니다.')
    return xmin-padding, xmax+padding, ymin-padding, ymax+padding


class FrontierMap:
    def __init__(self, data, width, height, resolution, origin=(0., 0., 0.), clearance=.30,
                 footprint=None):
        if len(data) != width*height or not 0 < len(data) <= 250000 or resolution <= 0:
            raise ValueError('지도 크기·해상도가 지원 범위를 벗어났습니다.')
        self.data, self.width, self.height = list(data), width, height
        self.resolution, self.origin, self.clearance = resolution, tuple(origin), clearance
        self.free = bytearray(0 <= v <= 64 for v in data)
        self.disk_cache = {}
        self.footprint, self.body_cache = footprint, {}

    def body_clear(self, pose):
        """회전된 사각 몸체와 지도 셀의 겹침을 분리축 검사로 판정한다."""
        if self.footprint is None:
            i = self.cell(pose)
            return i is not None and self.disk_clear(i)
        xmin, xmax, ymin, ymax = self.footprint
        hx, hy = (xmax-xmin)/2, (ymax-ymin)/2
        mx, my = (xmin+xmax)/2, (ymin+ymax)/2
        a = pose[2]-self.origin[2]
        c, s = math.cos(a), math.sin(a)
        px, py = transform(pose[:2], self.origin, inverse=True)
        cx, cy = px+c*mx-s*my, py+s*mx+c*my
        rx, ry = abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy
        half = self.resolution/2
        cell_projection = half*(abs(c)+abs(s))
        for ny in range(math.floor((cy-ry)/self.resolution), math.floor((cy+ry)/self.resolution)+1):
            for nx in range(math.floor((cx-rx)/self.resolution), math.floor((cx+rx)/self.resolution)+1):
                dx, dy = (nx+.5)*self.resolution-cx, (ny+.5)*self.resolution-cy
                if (abs(dx) > rx+half or abs(dy) > ry+half
                        or abs(c*dx+s*dy) > hx+cell_projection
                        or abs(-s*dx+c*dy) > hy+cell_projection):
                    continue
                if not (0 <= nx < self.width and 0 <= ny < self.height) or not self.free[ny*self.width+nx]:
                    return False
        return True

    def goal_angle(self, i, angles):
        for angle in dict.fromkeys(round(wrap(a), 8) for a in angles):
            key = i, angle
            if key not in self.body_cache:
                self.body_cache[key] = self.body_clear((*self.world(i), angle))
            if self.body_cache[key]:
                return angle
        return None

    def cell(self, p):
        x, y = transform(p, self.origin, inverse=True)
        x, y = math.floor(x/self.resolution), math.floor(y/self.resolution)
        return y*self.width+x if 0 <= x < self.width and 0 <= y < self.height else None

    def world(self, i):
        return transform(((i % self.width+.5)*self.resolution,
                          (i//self.width+.5)*self.resolution), self.origin)

    def neighbors(self, i, diagonal=False):
        x, y = i % self.width, i//self.width
        steps = ((-1, 0), (1, 0), (0, -1), (0, 1))
        if diagonal:
            steps += ((-1, -1), (1, -1), (-1, 1), (1, 1))
        return [ny*self.width+nx for dx, dy in steps
                if 0 <= (nx := x+dx) < self.width and 0 <= (ny := y+dy) < self.height]

    def disk_clear(self, i):
        if i in self.disk_cache:
            return self.disk_cache[i]
        x, y = i % self.width, i//self.width
        distance = self.clearance+math.sqrt(.5)*self.resolution
        n = math.ceil(distance/self.resolution)
        clear = True
        for dy in range(-n, n+1):
            for dx in range(-n, n+1):
                if math.hypot(dx, dy)*self.resolution > distance:
                    continue
                nx, ny = x+dx, y+dy
                if not (0 <= nx < self.width and 0 <= ny < self.height) or not self.free[ny*self.width+nx]:
                    clear = False
                    break
            if not clear:
                break
        self.disk_cache[i] = clear
        return clear

    def signature(self, p):
        i = self.cell(p)
        if i is None:
            return None
        x, y, n = i % self.width, i//self.width, math.ceil(.6/self.resolution)
        counts = [0, 0, 0]
        for ny in range(max(0, y-n), min(self.height, y+n+1)):
            for nx in range(max(0, x-n), min(self.width, x+n+1)):
                value = self.data[ny*self.width+nx]
                counts[0 if value == -1 else 1 if value <= 64 else 2] += 1
        return tuple(counts)

    def visible_edge(self, start, end):
        """목표와 경계 사이가 알려진 빈 공간인지 검사한다. 경로 통과 판정은 Nav2 담당."""
        a, b = self.world(start), self.world(end)
        steps = max(1, math.ceil(math.dist(a, b)/(self.resolution*.4)))
        for n in range(steps+1):
            i = self.cell((a[0]+(b[0]-a[0])*n/steps, a[1]+(b[1]-a[1])*n/steps))
            if i is None or not self.free[i]:
                return False
        return True

    def information_gain(self, i, sensor_range=3.5, heading=None):
        """장애물에서 멈추는 가상 광선의 미탐색 면적 추정. 실제 통행 가능성을 뜻하지 않는다."""
        x, y = i % self.width+.5, i//self.width+.5
        unknown = set()
        rays = 96 if heading is None else 48
        for ray in range(rays):
            a = ray*2*math.pi/rays if heading is None else heading-self.origin[2]+(ray/(rays-1)-.5)*2*math.pi/3
            c, s = math.cos(a), math.sin(a)
            for step in range(1, math.ceil(sensor_range/self.resolution)+1):
                nx, ny = math.floor(x+c*step), math.floor(y+s*step)
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    v = self.data[ny*self.width+nx]
                    if v >= 65:
                        break
                    if v == -1:
                        unknown.add((nx, ny))
                else:
                    unknown.add((nx, ny))
        return len(unknown)*self.resolution**2

    def select(self, pose, excluded=(), now=0., preferred=None):
        start = self.cell(pose[:2])
        if start is None or not self.free[start]:
            return dict(state='start_unobserved', goals=[], frontiers=0, area=0.)
        costs, parents, queue, edges = {start: 0}, {}, deque([start]), set()
        while queue:
            i = queue.popleft()
            neighbors = self.neighbors(i)
            if len(neighbors) < 4 or any(self.data[j] == -1 for j in neighbors):
                edges.add(i)
            for j in neighbors:
                if self.free[j] and j not in costs:
                    costs[j] = costs[i]+1
                    parents[j] = i
                    queue.append(j)
        groups = []
        while edges:
            seed = edges.pop()
            group, queue = [seed], deque([seed])
            while queue:
                for j in self.neighbors(queue.popleft(), True):
                    if j in edges:
                        edges.remove(j)
                        group.append(j)
                        queue.append(j)
            if len(group) >= 2 and len(group)*self.resolution >= .10:
                groups.append(group)
        goals, used, gains = [], set(), {}
        offsets = {(round(length*math.cos(a)/self.resolution), round(length*math.sin(a)/self.resolution))
                   for length in (.20, .30, .40, .60, .90) for a in (k*math.pi/8 for k in range(16))}
        for group in sorted(groups, key=len, reverse=True)[:30]:
            choices = []
            for edge in group[::max(1, len(group)//30)]:
                x, y = edge % self.width, edge//self.width
                for dx, dy in offsets:
                    nx, ny = x+dx, y+dy
                    if not (0 <= nx < self.width and 0 <= ny < self.height):
                        continue
                    i = ny*self.width+nx
                    if i in used or i not in costs or math.dist(self.world(i), pose[:2]) < .18:
                        continue
                    if not self.visible_edge(i, edge):
                        continue
                    p = self.world(i)
                    if any(math.dist(p, r['point']) < .4
                           and (tuple(r['signature']) == self.signature(p) or now-r['time'] < 60)
                           and (r['success'] or now-r['time'] < 120) for r in excluded):
                        continue
                    ex, ey = self.world(edge)
                    direction = math.atan2(ey-p[1], ex-p[0])
                    previous = self.world(parents.get(i, start))
                    approach = math.atan2(p[1]-previous[1], p[0]-previous[0])
                    angle = self.goal_angle(i, (approach, pose[2], direction,
                                                 approach+math.pi, direction+math.pi/2, direction-math.pi/2))
                    if angle is None:
                        continue
                    score = costs[i]*self.resolution/(1+math.sqrt(len(group)*self.resolution))
                    choices.append((score, i, angle, (ex, ey)))
            # 같은 후보의 중복 평가를 줄인다
            unique = {}
            for choice in sorted(choices):
                unique.setdefault(choice[1], choice)
            ranked = []
            # 가까운 후보와 경계에 더 가까이 접근하는 후보를 함께 평가한다.
            shortlist = list(unique.values())[:4]
            shortlist += sorted(unique.values(), key=lambda q: math.dist(self.world(q[1]), q[3]))[:4]
            if preferred is not None:
                shortlist += sorted(unique.values(), key=lambda q: math.dist(q[3], preferred))[:4]
            shortlist = list(dict.fromkeys(shortlist))
            for _, i, angle, edge_point in shortlist:
                if i not in gains:
                    gains[i] = self.information_gain(i)
                gain = gains[i]
                if gain <= 0:
                    continue
                point = self.world(i)
                direction = math.atan2(edge_point[1]-point[1], edge_point[0]-point[0])
                frontier_gain = self.information_gain(i, heading=direction)
                if frontier_gain <= 0:
                    continue
                distance = costs[i]*self.resolution
                stand_off = math.dist(point, edge_point)
                score = (1+distance)/(frontier_gain+.05)+1.2*stand_off
                if preferred is not None:
                    score += 2*math.dist(edge_point, preferred)
                ranked.append((score, i, angle, edge_point, gain, distance, frontier_gain, stand_off))
            for score, i, angle, edge_point, gain, distance, frontier_gain, stand_off in sorted(ranked)[:2]:
                if i not in used:
                    used.add(i)
                    goals.append(dict(point=(*self.world(i), angle), score=score,
                                      frontier=edge_point, expected_gain_m2=gain, distance_m=distance,
                                      frontier_gain_m2=frontier_gain, stand_off_m=stand_off,
                                      signature=self.signature(self.world(i))))
        for goal in goals:
            edge = self.cell(goal['frontier'])
            unknown = [j for j in self.neighbors(edge) if self.data[j] == -1]
            goal['interest'] = self.world(unknown[0]) if unknown else goal['frontier']
        return dict(state='ready' if goals else 'no_frontiers' if not groups else 'no_candidates',
                    goals=sorted(goals, key=lambda g: g['score'])[:20], frontiers=len(groups),
                    area=len(costs)*self.resolution**2,
                    body_model='rectangle' if self.footprint else 'circle',
                    start_body_clear=self.body_clear(pose), reachable_cells=len(costs))

    def wall_checks(self, pose, checked=()):
        """경계 탐색 후 알려진 막힘 면의 접근 가능한 앞쪽을 유한하게 재관측한다."""
        start = self.cell(pose)
        if start is None or not self.free[start]:
            return dict(state='inspection_start_unobserved', goals=[], pending=0)
        reachable, queue, walls = {start: 0}, deque([start]), {}
        while queue:
            i = queue.popleft()
            for j in self.neighbors(i):
                if self.free[j] and j not in reachable:
                    reachable[j] = reachable[i]+1
                    queue.append(j)
                elif self.data[j] >= 65:
                    walls.setdefault(j, i)
        # 벽 전체를 같은 목표로 반복 방문하지 않도록 1m 구간별 대표점을 고른다.
        bins = {}
        for wall, edge in sorted(walls.items(), key=lambda pair: math.dist(self.world(pair[0]), pose[:2])):
            point = self.world(wall)
            if any(math.dist(point, p) < .8 for p in checked):
                continue
            key = (math.floor((wall % self.width)*self.resolution),
                   math.floor((wall//self.width)*self.resolution))
            bins.setdefault(key, (wall, edge))
        goals = []
        for wall, edge in bins.values():
            wall_point, edge_point = self.world(wall), self.world(edge)
            choices = []
            for radius in (.35, .50, .70):
                for k in range(16):
                    a = k*math.pi/8
                    i = self.cell((wall_point[0]+radius*math.cos(a), wall_point[1]+radius*math.sin(a)))
                    if i is None or i not in reachable or not self.visible_edge(i, edge):
                        continue
                    p = self.world(i)
                    direction = math.atan2(wall_point[1]-p[1], wall_point[0]-p[0])
                    angle = self.goal_angle(i, (pose[2], direction, direction+math.pi/2, direction-math.pi/2))
                    if angle is not None:
                        choices.append((reachable[i], i, angle))
            if choices:
                distance, i, angle = min(choices)
                goals.append(dict(point=(*self.world(i), angle), score=distance*self.resolution,
                                  frontier=edge_point, inspection_point=wall_point, kind='wall_check',
                                  signature=self.signature(self.world(i))))
        return dict(state='inspection_ready' if goals else 'inspection_blocked' if bins else 'inspection_done',
                    goals=sorted(goals, key=lambda g: g['score']), pending=len(bins))

    def survey(self, pose, excluded=(), now=0., preferred=None, checked=(), inspect=False):
        report = self.select(pose, excluded, now, preferred)
        if inspect and report['state'] in ('no_frontiers', 'no_candidates'):
            inspection = self.wall_checks(pose, checked)
            if inspection['state'] == 'inspection_done' and report['frontiers']:
                inspection['state'] = 'inspection_exhausted'
            inspection.update(frontiers=report['frontiers'], area=report['area'],
                              exploration_state=report['state'])
            return inspection
        return report


def single_target(report):
    goals = report.get('goals', [])
    return goals[:1]


class Completion:
    def __init__(self, hold=60.):
        self.hold, self.since, self.last_stamp, self.samples = hold, None, None, 0
        self.area = 0.

    def update(self, report, now, stamp):
        if report['state'] != 'no_frontiers' or report['area'] < .5:
            self.since, self.samples = None, 0
            return False
        if self.since is None or abs(report['area']-self.area) > max(.1, report['area']*.02):
            self.since, self.samples = now, 0
        self.area = report['area']
        if stamp != self.last_stamp:
            self.last_stamp, self.samples = stamp, self.samples+1
        return now-self.since >= self.hold and self.samples >= 3


def valid_nav_path(poses, goal, tolerance=.20):
    return bool(poses) and all(math.isfinite(x) and math.isfinite(y) for x, y in poses) and math.dist(poses[-1], goal[:2]) <= tolerance


def arrival_yaw(poses, fallback):
    if not poses:
        return fallback
    end = poses[-1]
    for point in reversed(poses[:-1]):
        if math.dist(point, end) >= .10:
            return math.atan2(end[1]-point[1], end[0]-point[0])
    return fallback


def goal_visuals(goals, current, stage=None, paused=False, done=False, home=False, preferred=None):
    """ROS 메시지와 분리한 RViz 표시 정의. 후보와 전송된 목표를 구분한다."""
    items = []
    if preferred is not None:
        items.append(dict(kind='sphere', ns='preferred_region', id=0, point=preferred,
                          color=(1., .2, .7, .9), scale=(.22, .22, .22), z=.2, text=''))
        items.append(dict(kind='text', ns='preferred_label', id=0, point=preferred,
                          color=(1., .4, .8, 1.), scale=(0., 0., .18), z=.45,
                          text='CHECK WALL' if current and current.get('kind') == 'wall_check' else 'EXPLORE HERE'))
    for n, goal in enumerate(goals[:20]):
        p = goal['point']
        items.append(dict(kind='sphere', ns='candidates', id=n, point=p, color=(0., .8, 1., .8),
                          scale=(.14, .14, .14), z=.10, text=''))
        items.append(dict(kind='text', ns='candidate_labels', id=n, point=p, color=(.6, .95, 1., 1.),
                          scale=(0., 0., .13), z=.25,
                          text=f'C{n+1} gain={goal.get("frontier_gain_m2", goal["expected_gain_m2"]):.1f}m2' if 'expected_gain_m2' in goal else f'C{n+1}'))
        if 'frontier' in goal:
            items.append(dict(kind='sphere', ns='frontier_edges', id=n, point=(*goal['frontier'], 0.),
                              color=(1., .85, .1, 1.), scale=(.10, .10, .10), z=.12, text=''))
    if current is not None:
        p = current['point']
        if paused:
            label, color = 'PAUSED', (1., .2, .2, 1.)
        elif done:
            label, color = 'DONE', (.6, .6, .6, 1.)
        elif stage in ('nav_send', 'nav_result'):
            label, color = ('HOME' if home else 'GOAL'), ((.8, .3, 1., 1.) if home else (.15, 1., .15, 1.))
        elif stage in ('plan_send', 'plan_result'):
            label, color = 'CHECK', (1., .65, .05, 1.)
        else:
            label, color = 'LAST GOAL', (.6, .6, .6, 1.)
        items.append(dict(kind='arrow', ns='selected', id=0, point=p, color=color,
                          scale=(.55, .12, .12), z=.15, text=''))
        items.append(dict(kind='text', ns='selected_label', id=0, point=p, color=color,
                          scale=(0., 0., .20), z=.50,
                          text=f'{label} x={p[0]:.2f} y={p[1]:.2f}'))
    return items


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock as ClockMessage
    from std_msgs.msg import Bool
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener

    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    class Explorer(Node):
        def __init__(self):
            super().__init__('house_nav2_frontier')
            if not self.get_parameter('use_sim_time').value:
                raise ValueError('시뮬레이션 시험은 --ros-args -p use_sim_time:=true 로 실행하세요.')
            for key, value in [('goal_clearance', .30), ('completion_hold', 60.), ('goal_timeout', 600.),
                               ('base_frame', 'base_footprint'), ('clock_timeout', 2.0),
                               ('goal_footprint', WAFFLE_PI_FOOTPRINT), ('goal_padding', .01),
                               ('inspect_blocked', True)]:
                self.declare_parameter(key, value)
            self.clearance = float(self.get_parameter('goal_clearance').value)
            if not .28 <= self.clearance <= .6:
                raise ValueError('goal_clearance는 0.28~0.60m 범위여야 합니다.')
            self.footprint = footprint_bounds(self.get_parameter('goal_footprint').value,
                                              float(self.get_parameter('goal_padding').value))
            self.wait_reason = None
            self.inspect_blocked = bool(self.get_parameter('inspect_blocked').value)
            self.inspection_requested = False
            self.inspection_history = []
            self.complete = Completion(float(self.get_parameter('completion_hold').value))
            self.blocked_wait = Completion(20.)
            if self.complete.hold < 15:
                raise ValueError('completion_hold는 15초 이상이어야 합니다.')
            self.base = self.get_parameter('base_frame').value
            self.goal_timeout = float(self.get_parameter('goal_timeout').value)
            self.clock_timeout = float(self.get_parameter('clock_timeout').value)
            if self.goal_timeout <= 0 or self.clock_timeout < 1:
                raise ValueError('goal_timeout은 양수, clock_timeout은 1초 이상이어야 합니다.')
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
            visual_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.goal_pub = self.create_publisher(PoseStamped, '/house_frontier/goal', visual_qos)
            self.candidates_pub = self.create_publisher(PoseArray, '/house_frontier/candidates', visual_qos)
            self.markers_pub = self.create_publisher(MarkerArray, '/house_frontier/markers', visual_qos)
            self.path_pub = self.create_publisher(NavPath, '/house_frontier/path', visual_qos)
            self.create_subscription(NavPath, '/plan', self.on_plan, 1)
            self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(ClockMessage, '/clock', self.on_clock, qos_profile_sensor_data)
            self.create_subscription(Bool, '/house_frontier/resume', self.on_resume, 10)
            self.create_subscription(Bool, '/house_frontier/return_home', self.on_home, 10)
            self.create_subscription(PointStamped, '/clicked_point', self.on_preferred, 10)
            self.create_subscription(Bool, '/house_frontier/clear_preference', self.on_clear_preference, 10)
            self.map = self.scan = None
            self.clock_value = None
            self.clock_wall = self.map_wall = self.scan_wall = 0.
            self.paused = False
            self.home_requested = False
            self.done = False
            self.origin = None
            self.map_from_odom = None
            self.current_pose = None
            self.operation = None
            self.cancel_reason = None
            self.cancel_future = None
            self.current_goal = None
            self.current_home = False
            self.preferred_odom = None
            self.auto_interest_odom = None
            self.latest_feedback = None
            self.active_path = None
            self.events_path = Path.home()/'ros2_practice_ws'/'diagnostics'/f'frontier_events_{time.time_ns()}.jsonl'
            self.events_path.parent.mkdir(parents=True, exist_ok=True)
            self.excluded = deque(maxlen=300)
            self.queue = []
            self.selection = None
            self.selection_stamp = None
            self.last_report = None
            self.selection_started = 0.
            self.next_select = 0.
            self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='nav2_frontiers')
            self.last_warning = self.last_save = self.last_feedback = -math.inf
            self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.create_timer(1., self.publish_visuals, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info('Nav2 자동 목표 탐색 준비: /map, /scan, /clock, TF, Nav2 액션을 기다립니다.')
            self.get_logger().info('RViz 표시: Add → By topic → /house_frontier/markers → MarkerArray')
            self.get_logger().info('후보 몸체 검사: '+('방향별 사각 윤곽' if self.footprint else '원형 여유'))
            self.get_logger().info('탐색 사건 로그: '+str(self.events_path))
            self.write_event('started', footprint=self.footprint)
            self.clear_path()

        def clear_path(self):
            message = NavPath()
            message.header.frame_id = 'map'
            self.path_pub.publish(message)

        def on_plan(self, message):
            if (self.operation is not None and self.operation['stage'].startswith('nav')
                    and self.current_goal is not None and not self.cancel_reason and not self.paused
                    and message.header.frame_id == 'map'
                    and valid_nav_path([(p.pose.position.x, p.pose.position.y) for p in message.poses],
                                       self.current_goal['point'])):
                self.path_pub.publish(message)

        def write_event(self, event, **details):
            try:
                with self.events_path.open('a', encoding='utf-8') as stream:
                    stream.write(json.dumps(dict(wall_time=time.time(), clock=self.clock_value,
                                                 event=event, pose=self.current_pose, **details), ensure_ascii=False)+'\n')
            except OSError as error:
                if time.monotonic()-getattr(self, 'last_event_error', -math.inf) > 30:
                    self.last_event_error = time.monotonic()
                    self.get_logger().warn('사건 로그 기록 실패: '+str(error))

        def reset_preference_plan(self):
            self.queue = []
            self.selection = None
            self.next_select = 0.
            self.request_cancel('관심 미탐색 영역 변경: 현재 목표 취소 후 재선택')

        def on_preferred(self, m):
            if m.header.frame_id != 'map' or self.map_from_odom is None or self.done or self.home_requested:
                self.get_logger().warn('관심 영역은 TF 준비 후 매핑 중 RViz Fixed Frame=map에서 지정하세요.')
                return
            point = (m.point.x, m.point.y)
            if not all(math.isfinite(v) for v in point):
                return
            self.preferred_odom = transform(point, self.map_from_odom, inverse=True)
            self.reset_preference_plan()
            self.write_event('preferred_region', point=point)
            self.get_logger().info(f'관심 영역 x={point[0]:.2f}, y={point[1]:.2f}: 인접한 안전한 미탐색 경계를 우선 탐색')

        def on_clear_preference(self, m):
            if m.data:
                self.preferred_odom = None
                self.reset_preference_plan()
                self.write_event('preference_cleared')

        def on_map(self, m):
            self.map, self.map_wall = m, time.monotonic()

        def on_scan(self, m):
            self.scan, self.scan_wall = m, time.monotonic()

        def on_clock(self, m):
            value = m.clock.sec+m.clock.nanosec/1e9
            if self.clock_value is not None and value < self.clock_value:
                self.pause('시뮬레이션 시간 초기화: 탐색기를 다시 실행하세요')
            if value != self.clock_value:
                self.clock_value, self.clock_wall = value, time.monotonic()

        def on_resume(self, m):
            if not m.data:
                self.pause('사용자 일시정지 요청')
            elif not self.done and self.operation is None:
                self.paused, self.cancel_reason = False, None
                self.queue = []
                self.next_select = 0.
                self.get_logger().info('자동 탐색 재개: 입력 상태 재확인')

        def on_home(self, m):
            if m.data and not self.done:
                self.home_requested = True
                self.queue = []
                self.request_cancel('출발점 복귀 전 현재 목표 취소')

        def request_cancel(self, reason):
            if self.operation is not None:
                self.clear_path()
                self.cancel_reason = reason
                handle = self.operation.get('handle')
                if handle is not None and self.cancel_future is None:
                    self.cancel_future = handle.cancel_goal_async()

        def pause(self, reason):
            if not self.paused:
                self.get_logger().warn(reason+' — 자동 목표 일시정지 및 진행 중 목표 취소 요청')
                self.write_event('paused', reason=reason)
            self.paused = True
            self.queue = []
            self.request_cancel(reason)

        def readiness(self):
            wall, now = time.monotonic(), self.clock_value
            if now is None or wall-self.clock_wall > self.clock_timeout:
                return '시뮬레이션 시간 갱신 없음: Gazebo 재생 및 /clock 통신 확인 필요'
            for name, m, received, limit in (('지도', self.map, self.map_wall, 5.),
                                             ('라이다', self.scan, self.scan_wall, .7)):
                if m is None:
                    return name+' 수신 대기'
                age = now-(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
                if not -.3 <= age <= limit or wall-received > limit:
                    return f'{name} 시각/수신 지연: stamp차이={age:+.3f}초, 수신공백={wall-received:.3f}초'
            if self.map.header.frame_id != 'map':
                return '현재 구현은 /map의 frame_id=map 구성을 사용합니다'
            if not self.nav.server_is_ready() or not self.planner.server_is_ready():
                return 'Nav2 /navigate_to_pose 및 /compute_path_to_pose 준비 대기'
            try:
                t = self.buffer.lookup_transform('map', self.base, Time())
                age = now-(t.header.stamp.sec+t.header.stamp.nanosec/1e9)
                if not -.3 <= age <= .7:
                    return f'로봇 TF 시각 지연: {age:+.3f}초'
                self.current_pose = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
                mt = self.buffer.lookup_transform('map', 'odom', Time())
                self.map_from_odom = (mt.transform.translation.x, mt.transform.translation.y, yaw(mt.transform.rotation))
                if self.origin is None:
                    ot = self.buffer.lookup_transform('odom', self.base, Time())
                    self.origin = (ot.transform.translation.x, ot.transform.translation.y, yaw(ot.transform.rotation))
            except Exception as exc:
                return 'TF 준비 대기: '+str(exc).split('\n')[0]
            return None

        def pose_message(self, point):
            m = PoseStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y = float(point[0]), float(point[1])
            m.pose.orientation.z, m.pose.orientation.w = math.sin(point[2]/2), math.cos(point[2]/2)
            return m

        def publish_visuals(self):
            stage = None if self.operation is None else self.operation['stage']
            visible_goal = self.current_goal if stage and not self.paused and not self.done else None
            array = PoseArray()
            array.header = self.pose_message((0, 0, 0)).header
            array.poses = [] if visible_goal is None else [self.pose_message(visible_goal['point']).pose]
            self.candidates_pub.publish(array)
            if self.current_goal is not None and stage in ('nav_send', 'nav_result') and not self.paused:
                self.goal_pub.publish(self.pose_message(self.current_goal['point']))
            markers = MarkerArray()
            clear = Marker()
            clear.header.frame_id = 'map'
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            interest = self.preferred_odom if self.preferred_odom is not None else self.auto_interest_odom
            preferred = transform(interest, self.map_from_odom) if interest is not None and self.map_from_odom is not None else None
            for spec in goal_visuals([], visible_goal, stage, self.paused, self.done, self.current_home, preferred):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns, marker.id = spec['ns'], spec['id']
                marker.action = Marker.ADD
                marker.type = {'sphere': Marker.SPHERE, 'text': Marker.TEXT_VIEW_FACING, 'arrow': Marker.ARROW}[spec['kind']]
                p = spec['point']
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(p[0]), float(p[1]), spec['z']
                angle = p[2] if len(p) > 2 else 0.
                marker.pose.orientation.z, marker.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
                marker.scale.x, marker.scale.y, marker.scale.z = spec['scale']
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec['color']
                marker.text = spec['text']
                marker.lifetime.sec = 3
                markers.markers.append(marker)
            self.markers_pub.publish(markers)

        def remember(self, success):
            if self.current_goal is not None and not self.current_home:
                if self.current_goal.get('kind') == 'wall_check':
                    self.inspection_history.append(dict(point=self.current_goal['inspection_odom'],
                                                        success=success, time=time.monotonic()))
                self.queue = [g for g in self.queue if math.dist(g['point'][:2], self.current_goal['point'][:2]) >= .4]
                self.excluded.append(dict(point=self.current_goal['odom_point'],
                                          signature=self.current_goal['signature'],
                                          success=success, time=time.monotonic()))

        def feedback(self, m):
            self.latest_feedback = dict(distance_remaining=m.feedback.distance_remaining,
                                        recoveries=m.feedback.number_of_recoveries,
                                        received_wall=time.time())
            if time.monotonic()-self.last_feedback > 5:
                self.last_feedback = time.monotonic()
                self.get_logger().info(f'Nav2 이동 중: 남은 거리={m.feedback.distance_remaining:.2f}m, '
                                       f'복구 횟수={m.feedback.number_of_recoveries}')
                self.write_event('feedback', **self.latest_feedback)

        def begin_plan(self, goal, home=False):
            if self.operation is not None:
                raise RuntimeError('기존 액션의 종료를 확인하기 전에는 다음 목표를 보낼 수 없습니다.')
            self.clear_path()
            self.current_goal, self.current_home = goal, home
            self.auto_interest_odom = None if home else goal.get('interest_odom', goal.get('inspection_odom'))
            self.latest_feedback, self.active_path = None, None
            self.write_event('plan_requested', goal=goal, home=home)
            request = ComputePathToPose.Goal()
            request.goal = self.pose_message(goal['point'])
            request.use_start = False
            self.operation = dict(stage='plan_send', future=self.planner.send_goal_async(request),
                                  started=time.monotonic())
            purpose = '복귀' if home else '막힘 재관측' if goal.get('kind') == 'wall_check' else '미탐색'
            self.get_logger().info(f"{purpose} 목표 경로 확인: "
                                   f"x={goal['point'][0]:.2f}, y={goal['point'][1]:.2f}")

        def poll_operation(self):
            op = self.operation
            if op is None:
                return
            if self.cancel_reason is not None:
                self.request_cancel(self.cancel_reason)
            if not op['future'].done():
                limit = self.goal_timeout if op['stage'].startswith('nav') else 20.
                if time.monotonic()-op['started'] > limit and not self.paused:
                    self.pause('Nav2 액션 응답 제한 시간 초과')
                return
            response = op['future'].result()
            if op['stage'].endswith('send'):
                if not response.accepted:
                    self.write_event('action_rejected', stage=op['stage'])
                    self.operation = None
                    self.clear_path()
                    self.cancel_reason, self.cancel_future = None, None
                    self.queue = []
                    self.next_select = time.monotonic()+5
                    self.get_logger().warn('Nav2 액션 목표 접수 거절: lifecycle active 상태와 서버 준비를 확인하세요.')
                    if self.current_home:
                        self.pause('Nav2가 복귀 경로 또는 복귀 목표를 거절했습니다')
                    return
                op['handle'], op['future'] = response, response.get_result_async()
                op['stage'] = 'plan_result' if op['stage'] == 'plan_send' else 'nav_result'
                if self.cancel_reason is not None:
                    self.request_cancel(self.cancel_reason)
                return
            self.operation = None
            self.clear_path()
            self.write_event('action_result', stage=op['stage'], status=response.status,
                             canceled_by=self.cancel_reason)
            if self.cancel_reason is not None:
                self.cancel_reason, self.cancel_future = None, None
                return
            if op['stage'] == 'plan_result':
                path = response.result.path
                poses = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
                if (response.status != GoalStatus.STATUS_SUCCEEDED or path.header.frame_id != 'map'
                        or not valid_nav_path(poses, self.current_goal['point'])):
                    self.remember(False)
                    self.get_logger().warn('Nav2 경로가 없거나 목표에 도달하지 않음: 다음 후보 선택')
                    if self.current_home:
                        self.pause('출발점까지 Nav2 경로를 확보하지 못했습니다')
                    return
                request = NavigateToPose.Goal()
                if not self.current_home:
                    # 360도 라이다이므로 도착 후 경계 쪽으로 다시 돌 필요 없이 경로 진입 방향을 우선한다.
                    point = self.current_goal['point']
                    incoming = arrival_yaw(poses, point[2])
                    m, origin = self.map, self.map.info.origin
                    grid = FrontierMap(m.data, m.info.width, m.info.height, m.info.resolution,
                                       (origin.position.x, origin.position.y, yaw(origin.orientation)),
                                       self.clearance, self.footprint)
                    if grid.body_clear((*point[:2], incoming)):
                        self.current_goal['point'] = (*point[:2], incoming)
                        self.current_goal['orientation_source'] = 'path_tangent'
                        self.write_event('arrival_orientation', yaw=incoming)
                self.active_path = dict(frame=path.header.frame_id,
                                        stamp=path.header.stamp.sec+path.header.stamp.nanosec/1e9,
                                        points=poses)
                self.path_pub.publish(path)
                request.pose = self.pose_message(self.current_goal['point'])
                self.goal_pub.publish(request.pose)
                self.operation = dict(stage='nav_send', future=self.nav.send_goal_async(request, feedback_callback=self.feedback),
                                      started=time.monotonic())
                self.get_logger().info('Nav2 Goal 전송: 경로 주행과 장애물 회피는 Nav2가 수행합니다.')
                return
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                self.remember(True)
                self.queue = []
                self.next_select = time.monotonic()+2
                self.get_logger().info('출발점 복귀 완료' if self.current_home else
                                       '막힘 재관측 지점 도착: 다음 구간 확인' if self.current_goal.get('kind') == 'wall_check' else
                                       '미탐색 목표 도착: 지도 갱신 후 다음 목표 선택')
                if self.current_home:
                    self.done = True
            elif response.status == GoalStatus.STATUS_CANCELED:
                self.pause('Nav2 목표가 외부에서 취소됨')
            else:
                self.remember(False)
                self.get_logger().warn('Nav2 주행 실패: 해당 목표를 잠시 제외하고 다음 후보 선택')
                if self.current_home:
                    self.pause('출발점 복귀 실패')

        def select_goals(self):
            m, o = self.map, self.map.info.origin
            grid = FrontierMap(m.data, m.info.width, m.info.height, m.info.resolution,
                               (o.position.x, o.position.y, yaw(o.orientation)), self.clearance, self.footprint)
            self.selection_stamp = m.header.stamp.sec+m.header.stamp.nanosec/1e9
            excluded = [dict(r, point=transform(r['point'], self.map_from_odom)) for r in self.excluded]
            pose = self.current_pose
            preferred = transform(self.preferred_odom, self.map_from_odom) if self.preferred_odom is not None else None
            self.selection_started = time.monotonic()
            checked = [transform(r['point'], self.map_from_odom) for r in self.inspection_history]
            self.selection = self.pool.submit(grid.survey, pose, excluded, time.monotonic(), preferred,
                                              checked, self.inspection_requested)

        def save(self):
            if time.monotonic()-self.last_save < 5:
                return
            self.last_save = time.monotonic()
            payload = dict(clock=self.clock_value, paused=self.paused, done=self.done,
                           pose=self.current_pose, origin_odom=self.origin, goal=self.current_goal,
                           home_requested=self.home_requested, operation=None if self.operation is None else self.operation['stage'],
                           report=self.last_report, excluded=list(self.excluded))
            payload['cancel_reason'] = self.cancel_reason
            payload['wait_reason'] = self.wait_reason
            payload['goal_footprint_bounds'] = self.footprint
            payload['feedback'] = self.latest_feedback
            payload['prechecked_path'] = self.active_path
            payload['events_log'] = str(self.events_path)
            payload['preferred_odom'] = self.preferred_odom
            payload['auto_interest_odom'] = self.auto_interest_odom
            payload['inspection_requested'] = self.inspection_requested
            payload['inspection_history'] = self.inspection_history
            payload['map_from_odom'] = self.map_from_odom
            if self.map is not None:
                m, o = self.map, self.map.info.origin
                payload['map'] = dict(data=list(m.data), width=m.info.width, height=m.info.height,
                                      resolution=m.info.resolution, origin=(o.position.x, o.position.y, yaw(o.orientation)),
                                      stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
            if self.scan is not None:
                payload['scan_stamp'] = self.scan.header.stamp.sec+self.scan.header.stamp.nanosec/1e9
            path = Path.home()/'ros2_practice_ws'/'house_nav2_frontier_state.json'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')

        def tick(self):
            try:
                problem = self.readiness()
                if problem != self.wait_reason:
                    self.write_event('readiness_changed', reason=problem)
                self.wait_reason = problem
                if problem and self.operation is not None:
                    self.pause(problem)
                self.poll_operation()
                if self.done:
                    self.save()
                    return
                if problem:
                    if self.operation is not None:
                        self.pause(problem)
                    elif time.monotonic()-self.last_warning > 5:
                        self.last_warning = time.monotonic()
                        self.get_logger().warn(problem)
                    self.save()
                    return
                if self.paused or self.operation is not None:
                    self.save()
                    return
                if self.home_requested:
                    point = transform(self.origin[:2], self.map_from_odom)
                    goal = dict(point=(*point, wrap(self.origin[2]+self.map_from_odom[2])))
                    self.begin_plan(goal, home=True)
                    return
                if self.selection is not None:
                    if not self.selection.done():
                        if time.monotonic()-self.selection_started > 15:
                            self.pause('미탐색 후보 계산 15초 초과: 지도 처리 부하 확인 필요')
                        return
                    report = self.selection.result()
                    self.selection = None
                    self.last_report = report
                    self.write_event('selection', state=report['state'], frontiers=report['frontiers'],
                                     candidates=len(report['goals']), best=report['goals'][:3])
                    self.queue = single_target(report)
                    for goal in self.queue:
                        goal['odom_point'] = transform(goal['point'][:2], self.map_from_odom, inverse=True)
                        if 'interest' in goal:
                            goal['interest_odom'] = transform(goal['interest'], self.map_from_odom, inverse=True)
                        if 'inspection_point' in goal:
                            goal['inspection_odom'] = transform(goal['inspection_point'], self.map_from_odom, inverse=True)
                    array = PoseArray()
                    array.header = self.pose_message((0, 0, 0)).header
                    array.poses = []
                    self.candidates_pub.publish(array)
                    self.get_logger().info(f"경계={report['frontiers']}개, Nav2 확인 후보={len(self.queue)}개, 상태={report['state']}")
                    if report['state'] == 'no_candidates':
                        self.get_logger().warn('미탐색 경계는 있으나 몸체가 들어갈 관측 후보 없음: '
                                               f"시작 몸체 여유={report.get('start_body_clear')}, "
                                               f"연결 셀={report.get('reachable_cells')} / 지도 갱신 후 재계산")
                    ordinary = dict(report, state=report.get('exploration_state', report['state']))
                    exploration_complete = self.complete.update(ordinary, self.clock_value, self.selection_stamp)
                    waiting = dict(state='no_frontiers' if report['state'] == 'no_candidates' else 'ready',
                                   area=report['area'])
                    blocked_stable = self.blocked_wait.update(waiting, self.clock_value, self.selection_stamp)
                    if report['state'] == 'inspection_done' and exploration_complete:
                        self.home_requested = True
                        failed = sum(not r['success'] for r in self.inspection_history)
                        self.write_event('inspection_finished', checked=len(self.inspection_history), failed=failed)
                        if failed:
                            self.get_logger().warn(f'막힘 재관측 시도 종료: 도달 실패 {failed}곳은 미확인으로 기록하고 복귀')
                        else:
                            self.get_logger().info('막힌 면의 접근 가능한 관측 지점 확인 완료: 출발점 복귀 요청')
                    elif report['state'] == 'inspection_blocked':
                        self.get_logger().warn('막힘 재관측 지점에 안전한 후보 없음: 완료로 판단하지 않고 대기')
                    elif report['state'] == 'inspection_exhausted':
                        self.get_logger().warn('막힘 재관측 시도 후에도 미탐색 경계에 안전한 경로 없음: 완료로 판단하지 않음')
                    elif exploration_complete and not self.inspection_requested:
                        if self.inspect_blocked:
                            self.inspection_requested = True
                            self.get_logger().info('미탐색 경계가 안정적으로 사라짐: 문 이외의 막힌 면을 순차 재관측')
                        else:
                            self.home_requested = True
                            self.get_logger().info('미탐색 경계 확인 종료: 출발점 복귀 요청')
                    elif blocked_stable and self.inspect_blocked:
                        self.inspection_requested = True
                        self.get_logger().info('후보 부족 상태 지속: 막힌 면의 접근 가능한 앞쪽을 재관측')
                    if not self.queue:
                        self.next_select = time.monotonic()+5
                    self.save()
                    return
                if self.queue:
                    self.begin_plan(self.queue.pop(0))
                    return
                if time.monotonic() >= self.next_select:
                    self.select_goals()
                    self.next_select = time.monotonic()+5
            except Exception as exc:
                self.pause('자동 목표 처리 실패: '+str(exc))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Explorer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.paused = True
            node.request_cancel('탐색기 종료')
            end = time.monotonic()+2
            while node.operation is not None and time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
                try:
                    node.poll_operation()
                except Exception as exc:
                    node.get_logger().warn('종료 중 Nav2 취소 응답 처리 실패: '+str(exc))
                    break
            if node.operation is not None:
                node.get_logger().warn('Nav2 목표 취소 완료를 확인하지 못했습니다. Gazebo를 일시정지하고 Nav2 상태를 확인하세요.')
            node.pool.shutdown(wait=False, cancel_futures=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

