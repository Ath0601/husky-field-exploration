#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException


class CoverageExplorer(Node):
    def __init__(self):
        super().__init__('coverage_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cell_size', 1.0)
        self.declare_parameter('visit_radius', 1.0)
        self.declare_parameter('min_unvisited_region', 8)
        self.declare_parameter('candidate_count', 8)
        self.declare_parameter('candidate_radius', 0.7)
        self.declare_parameter('goal_obstacle_radius', 0.45)
        self.declare_parameter('min_goal_separation', 0.8)
        self.declare_parameter('failed_region_radius', 1.5)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('planner_check_timeout', 5.0)
        self.declare_parameter('planning_period', 2.0)
        self.declare_parameter('max_known_cells', 200000)
        self.declare_parameter('planner_id', 'GridBased')

        self.map_msg = None
        self.visited = set()
        self.failed_goals = []
        self.visited_goals = []
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.last_goal = None
        self.decision_active = False
        self.plan_active = False
        self.plan_token = 0
        self.plan_started = None
        self.current_candidate = None
        self.current_candidates = []
        self.candidate_index = 0
        self.goal_yaw = 0.0
        self.last_mark_pose = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            qos
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(
            float(self.get_parameter('planning_period').value),
            self.exploration_cycle
        )
        self.watchdog = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info('Grid-based coverage explorer started.')
        self.get_logger().info('No frontier detection and no information-gain scoring are used.')

    # ------------------------------------------------------------
    # MAP / POSE
    # ------------------------------------------------------------
    def map_callback(self, msg):
        if sum(v >= 0 for v in msg.data) < 100:
            return
        self.map_msg = msg
        pose = self.robot_pose()
        if pose is not None:
            self.mark_visited(pose[0], pose[1])

    def robot_pose(self):
        if self.map_msg is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_msg.header.frame_id,
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            return t.transform.translation.x, t.transform.translation.y
        except TransformException:
            return None

    def world_to_map_cell(self, x, y):
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        if 0 <= r < info.height and 0 <= c < info.width:
            return r, c
        return None

    def map_cell_to_world(self, r, c):
        info = self.map_msg.info
        return (
            info.origin.position.x + (c + 0.5) * info.resolution,
            info.origin.position.y + (r + 0.5) * info.resolution
        )

    def mark_visited(self, x, y):
        if self.map_msg is None:
            return
        cell = self.world_to_map_cell(x, y)
        if cell is None:
            return
        info = self.map_msg.info
        radius = max(1, int(float(self.get_parameter('visit_radius').value) / info.resolution))
        r0, c0 = cell
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr * dr + dc * dc > radius * radius:
                    continue
                r, c = r0 + dr, c0 + dc
                if 0 <= r < info.height and 0 <= c < info.width:
                    if self.map_msg.data[r * info.width + c] >= 0:
                        self.visited.add((r, c))
        self.last_mark_pose = (x, y)

    # ------------------------------------------------------------
    # MAP CELL TESTS
    # ------------------------------------------------------------
    def value(self, r, c):
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        if not (0 <= r < info.height and 0 <= c < info.width):
            return None
        return self.map_msg.data[r * info.width + c]

    def free(self, r, c):
        v = self.value(r, c)
        return v is not None and 0 <= v < 50

    def goal_safe(self, x, y):
        cell = self.world_to_map_cell(x, y)
        if cell is None:
            return False
        r0, c0 = cell
        radius_m = float(self.get_parameter('goal_obstacle_radius').value)
        radius = max(1, int(math.ceil(radius_m / self.map_msg.info.resolution)))
        info = self.map_msg.info
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr * dr + dc * dc > radius * radius:
                    continue
                r, c = r0 + dr, c0 + dc
                if not (0 <= r < info.height and 0 <= c < info.width):
                    return False
                v = self.map_msg.data[r * info.width + c]
                if v >= 50:
                    return False
        return True

    def failed_region(self, x, y):
        radius = float(self.get_parameter('failed_region_radius').value)
        return any(math.hypot(x - gx, y - gy) < radius for gx, gy in self.failed_goals)

    def used_goal(self, x, y):
        sep = float(self.get_parameter('min_goal_separation').value)
        if self.failed_region(x, y):
            return True
        return any(math.hypot(x - gx, y - gy) < sep for gx, gy in self.visited_goals)

    # ------------------------------------------------------------
    # COVERAGE REGIONS
    # ------------------------------------------------------------
    def coverage_regions(self):
        if self.map_msg is None:
            return []

        info = self.map_msg.info
        cell_size = float(self.get_parameter('cell_size').value)
        block = max(1, int(round(cell_size / info.resolution)))
        min_region = int(self.get_parameter('min_unvisited_region').value)
        regions = {}

        # Work only over known free cells. Unknown cells are not considered
        # unexplored targets; they become eligible automatically when mapping
        # reveals them as free space.
        for r in range(info.height):
            base = r * info.width
            for c in range(info.width):
                if self.map_msg.data[base + c] < 0:
                    continue
                if not self.free(r, c):
                    continue
                if (r, c) in self.visited:
                    continue
                key = (r // block, c // block)
                regions.setdefault(key, []).append((r, c))

        result = []
        for key, cells in regions.items():
            if len(cells) >= min_region:
                result.append(cells)
        return result

    def region_candidates(self, cells, robot_pose):
        if not cells:
            return []
        info = self.map_msg.info
        rx, ry = robot_pose

        # Choose the geometric center of the unvisited region, then nearby
        # free cells. This is deliberately coverage-oriented rather than
        # frontier/information-gain-oriented.
        ar = sum(r for r, _ in cells) / len(cells)
        ac = sum(c for _, c in cells) / len(cells)
        center = sorted(cells, key=lambda p: (p[0] - ar) ** 2 + (p[1] - ac) ** 2)

        selected = center[:max(1, int(self.get_parameter('candidate_count').value))]
        candidates = []
        for r, c in selected:
            x, y = self.map_cell_to_world(r, c)
            if not self.goal_safe(x, y):
                continue
            if self.used_goal(x, y):
                continue
            d = math.hypot(x - rx, y - ry)
            candidates.append((d, x, y))

        # If the exact central cells are blocked/used, search the region for
        # the nearest usable free cells.
        if not candidates:
            step = max(1, len(center) // 40)
            for r, c in center[::step]:
                x, y = self.map_cell_to_world(r, c)
                if not self.goal_safe(x, y) or self.used_goal(x, y):
                    continue
                d = math.hypot(x - rx, y - ry)
                candidates.append((d, x, y))
                if len(candidates) >= int(self.get_parameter('candidate_count').value):
                    break

        candidates.sort(key=lambda q: q[0])
        return [(x, y) for _, x, y in candidates]

    def choose_region_candidates(self, robot_pose):
        regions = self.coverage_regions()
        if not regions:
            return []

        rx, ry = robot_pose
        ranked = []
        for cells in regions:
            candidates = self.region_candidates(cells, robot_pose)
            if not candidates:
                continue
            cx = sum(self.map_cell_to_world(r, c)[0] for r, c in cells) / len(cells)
            cy = sum(self.map_cell_to_world(r, c)[1] for r, c in cells) / len(cells)
            d = math.hypot(cx - rx, cy - ry)
            ranked.append((d, len(cells), candidates))

        ranked.sort(key=lambda q: q[0])
        # Try the nearest few regions. No information-gain score is involved.
        out = []
        for _, _, candidates in ranked[:6]:
            out.extend(candidates)
        return out

    # ------------------------------------------------------------
    # NAV2
    # ------------------------------------------------------------
    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.map_msg.header.frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def path_length(self, path):
        if not path.poses:
            return float('inf')
        total = 0.0
        prev = None
        for ps in path.poses:
            p = (ps.pose.position.x, ps.pose.position.y)
            if prev is not None:
                total += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
        return total

    def start_path_check(self, candidates):
        if not candidates:
            self.decision_active = False
            self.get_logger().warn('No reachable coverage candidate found in known free space.')
            return
        self.current_candidates = candidates
        self.candidate_index = 0
        self.plan_active = True
        self.check_next_candidate()

    def check_next_candidate(self):
        if not self.plan_active:
            return
        if self.candidate_index >= len(self.current_candidates):
            self.plan_active = False
            self.decision_active = False
            self.get_logger().warn('No candidate in the current coverage regions has a valid Nav2 path.')
            return

        x, y = self.current_candidates[self.candidate_index]
        self.candidate_index += 1
        robot = self.robot_pose()
        if robot is None:
            self.check_next_candidate()
            return

        yaw = math.atan2(y - robot[1], x - robot[0])
        goal = ComputePathToPose.Goal()
        goal.goal = self.make_pose(x, y, yaw)
        goal.use_start = False
        goal.planner_id = str(self.get_parameter('planner_id').value)

        self.plan_token += 1
        token = self.plan_token
        self.plan_started = time.monotonic()
        self.current_candidate = (x, y, yaw)
        self.get_logger().info(
            f'Checking coverage goal {self.candidate_index}/{len(self.current_candidates)}: '
            f'({x:.2f},{y:.2f})'
        )
        future = self.path_client.send_goal_async(goal)
        future.add_done_callback(lambda f, t=token: self.path_goal_response(f, t))

    def path_goal_response(self, future, token):
        if not self.plan_active or token != self.plan_token:
            return
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Planner request failed: {e}')
            self.check_next_candidate()
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 planner rejected the path request.')
            self.check_next_candidate()
            return
        future2 = handle.get_result_async()
        future2.add_done_callback(lambda f, t=token: self.path_result(f, t))

    def path_result(self, future, token):
        if not self.plan_active or token != self.plan_token:
            return
        try:
            result = future.result()
            path = result.result.path
            x, y, yaw = self.current_candidate
            if result.status == GoalStatus.STATUS_SUCCEEDED and path.poses:
                length = self.path_length(path)
                self.get_logger().info(
                    f'VALID coverage path: ({x:.2f},{y:.2f}) length={length:.2f}m'
                )
                self.plan_active = False
                self.decision_active = False
                self.goal_yaw = yaw
                self.send_goal(x, y, yaw)
                return
            self.get_logger().warn(f'Coverage candidate rejected by Nav2: status={result.status}')
        except Exception as e:
            self.get_logger().warn(f'Planner result error: {e}')
        self.check_next_candidate()

    def send_goal(self,x,y,yaw=0.0):
        goal=NavigateToPose.Goal()
        goal.pose=PoseStamped()
        goal.pose.header.frame_id=self.map_msg.header.frame_id
        goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=x
        goal.pose.pose.position.y=y
        goal.pose.pose.position.z=0.0
        goal.pose.pose.orientation.x=0.0
        goal.pose.pose.orientation.y=0.0
        goal.pose.pose.orientation.z=0.0
        goal.pose.pose.orientation.w=1.0
        self.goal_active=True
        self.last_goal=(x,y)
        self.goal_sent_time=time.monotonic()
        self.get_logger().info(f'Sending coverage goal: ({x:.2f},{y:.2f})')
        future=self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Navigation action failed: {e}')
            self.fail_goal()
            return
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected coverage goal.')
            self.fail_goal()
            return
        self.get_logger().info('Coverage goal accepted.')
        future2 = self.goal_handle.get_result_async()
        future2.add_done_callback(self.goal_result)

    def goal_result(self, future):
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                pose = self.robot_pose()
                if pose is not None:
                    self.mark_visited(pose[0], pose[1])
                elif self.last_goal is not None:
                    self.mark_visited(self.last_goal[0], self.last_goal[1])
                if self.last_goal is not None:
                    self.visited_goals.append(self.last_goal)
                self.get_logger().info('Coverage goal reached.')
            else:
                self.get_logger().warn(f'Coverage navigation ended with status {result.status}.')
                self.fail_goal()
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self.fail_goal()
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.decision_active = False

    def fail_goal(self):
        if self.last_goal is not None:
            x, y = self.last_goal
            radius = float(self.get_parameter('failed_region_radius').value)
            if not any(math.hypot(x - gx, y - gy) < radius for gx, gy in self.failed_goals):
                self.failed_goals.append((x, y))
                self.get_logger().warn(
                    f'Blacklisting failed region around ({x:.2f},{y:.2f}), radius={radius:.1f}m'
                )
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.decision_active = False

    # ------------------------------------------------------------
    # COVERAGE STATUS
    # ------------------------------------------------------------
    def coverage_stats(self):
        if self.map_msg is None:
            return 0, 0, 0.0
        known_free = 0
        visited_free = 0
        for r, value in enumerate(self.map_msg.data):
            if 0 <= value < 50:
                known_free += 1
                if r // self.map_msg.info.width in ():
                    pass
        if known_free:
            visited_free = sum(
                1 for r, c in self.visited
                if 0 <= r < self.map_msg.info.height
                and 0 <= c < self.map_msg.info.width
                and 0 <= self.map_msg.data[r * self.map_msg.info.width + c] < 50
            )
        percent = 100.0 * visited_free / known_free if known_free else 0.0
        return known_free, visited_free, percent

    # ------------------------------------------------------------
    # MAIN EXPLORATION LOOP
    # ------------------------------------------------------------
    def exploration_cycle(self):
        if self.goal_active or self.decision_active or self.map_msg is None:
            return
        if not self.nav_client.server_is_ready() or not self.path_client.server_is_ready():
            return

        pose = self.robot_pose()
        if pose is None:
            return
        self.mark_visited(pose[0], pose[1])

        candidates = self.choose_region_candidates(pose)
        known, visited, percent = self.coverage_stats()
        self.get_logger().info(
            f'Coverage: {percent:.1f}% of known free cells | known={known} visited={visited}'
        )

        if not candidates:
            self.get_logger().info('No unvisited known-free coverage region is currently reachable.')
            return

        self.decision_active = True
        self.start_path_check(candidates)

    def watchdog_callback(self):
        if self.goal_active and self.goal_sent_time is not None:
            if time.monotonic() - self.goal_sent_time > float(self.get_parameter('goal_timeout').value):
                self.get_logger().warn('Coverage goal timed out. Canceling and blacklisting.')
                if self.goal_handle:
                    future = self.goal_handle.cancel_goal_async()
                    future.add_done_callback(self.cancel_callback)
                else:
                    self.fail_goal()

        if self.plan_active and self.plan_started is not None:
            if time.monotonic() - self.plan_started > float(self.get_parameter('planner_check_timeout').value):
                self.get_logger().warn('Planner check timed out; trying next coverage candidate.')
                self.plan_token += 1
                self.check_next_candidate()

    def cancel_callback(self, future):
        try:
            if future.result().goals_canceling:
                self.get_logger().info('Navigation cancellation accepted.')
        except Exception as e:
            self.get_logger().error(f'Cancellation error: {e}')
        self.fail_goal()


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
