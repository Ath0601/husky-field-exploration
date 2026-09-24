#!/usr/bin/env python3
import math,time
from collections import deque
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer,TransformListener,TransformException

class InformationGainExplorer(Node):
    def __init__(self):
        super().__init__('information_gain_explorer')
        self.declare_parameter('map_topic','/map')
        self.declare_parameter('min_frontier_size',10)
        self.declare_parameter('candidate_offset_cells',4)
        self.declare_parameter('candidate_count_per_frontier',5)
        self.declare_parameter('max_planner_candidates',20)
        self.declare_parameter('sensor_radius',4.0)
        self.declare_parameter('distance_weight',12.0)
        self.declare_parameter('min_goal_separation',0.75)
        self.declare_parameter('goal_clearance_cells',5)
        self.declare_parameter('min_path_clearance',0.3)
        self.declare_parameter('goal_timeout',45.0)
        self.declare_parameter('planning_period',2.0)
        self.declare_parameter('planner_id','GridBased')
        self.map_msg=None
        self.goal_active=False
        self.goal_handle=None
        self.goal_sent_time=None
        self.last_goal=None
        self.failed_goals=[]
        self.visited_goals=[]
        self.decision_active=False
        self.planner_check_active=False
        self.planner_check_queue=[]
        self.planner_check_index=0
        self.planner_check_started=None
        qos=QoSProfile(depth=1)
        qos.reliability=ReliabilityPolicy.RELIABLE
        qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub=self.create_subscription(OccupancyGrid,self.get_parameter('map_topic').value,self.map_callback,qos)
        self.nav_client=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.path_client=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.timer=self.create_timer(self.get_parameter('planning_period').value,self.exploration_cycle)
        self.timeout_timer=self.create_timer(1.0,self.check_goal_timeout)
        self.get_logger().info('Information-gain frontier explorer started.')
        self.get_logger().info('Using Nav2 path reachability and path-clearance checking.')

    def map_callback(self,msg):
        if sum(1 for v in msg.data if v>=0)>=100:
            self.map_msg=msg

    def robot_pose(self):
        if self.map_msg is None:return None
        try:
            t=self.tf_buffer.lookup_transform(self.map_msg.header.frame_id,'base_link',rclpy.time.Time(),timeout=Duration(seconds=0.2))
            return t.transform.translation.x,t.transform.translation.y
        except TransformException:return None

    def cell_to_world(self,r,c):
        i=self.map_msg.info
        return i.origin.position.x+(c+0.5)*i.resolution,i.origin.position.y+(r+0.5)*i.resolution

    def world_to_cell(self,x,y):
        i=self.map_msg.info
        c=int((x-i.origin.position.x)/i.resolution)
        r=int((y-i.origin.position.y)/i.resolution)
        return (r,c) if 0<=r<i.height and 0<=c<i.width else None

    def cell_value(self,r,c):
        if self.map_msg is None:return None
        i=self.map_msg.info
        if not(0<=r<i.height and 0<=c<i.width):return None
        return self.map_msg.data[r*i.width+c]

    def free(self,r,c):
        v=self.cell_value(r,c)
        return v is not None and 0<=v<50

    def frontier(self,r,c):
        if not self.free(r,c):return False
        i=self.map_msg.info
        for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
            rr,cc=r+dr,c+dc
            if 0<=rr<i.height and 0<=cc<i.width and self.map_msg.data[rr*i.width+cc]==-1:return True
        return False

    def get_frontiers(self):
        i=self.map_msg.info
        cells={(r,c) for r in range(i.height) for c in range(i.width) if self.frontier(r,c)}
        minimum=int(self.get_parameter('min_frontier_size').value)
        components=[]
        while cells:
            seed=cells.pop()
            q=deque([seed]);comp=[seed]
            while q:
                r,c=q.popleft()
                for dr,dc in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                    n=(r+dr,c+dc)
                    if n in cells:
                        cells.remove(n);q.append(n);comp.append(n)
            if len(comp)>=minimum:components.append(comp)
        return components

    def goal_is_safe(self,x,y):
        cell=self.world_to_cell(x,y)
        if cell is None:return False
        r,c=cell
        radius=int(self.get_parameter('goal_clearance_cells').value)
        i=self.map_msg.info
        for dr in range(-radius,radius+1):
            for dc in range(-radius,radius+1):
                if dr*dr+dc*dc>radius*radius:continue
                rr,cc=r+dr,c+dc
                if not(0<=rr<i.height and 0<=cc<i.width):return False
                v=self.map_msg.data[rr*i.width+cc]
                if v>=50:return False
        return True

    def candidate_points(self,component):
        i=self.map_msg.info
        offset=int(self.get_parameter('candidate_offset_cells').value)
        count=max(1,int(self.get_parameter('candidate_count_per_frontier').value))
        component=sorted(component)
        if len(component)>count:
            idx=[int(k*(len(component)-1)/(count-1)) for k in range(count)] if count>1 else [len(component)//2]
            selected=[component[k] for k in idx]
        else:selected=component
        candidates=[]
        for r,c in selected:
            for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
                rr,cc=r+dr,c+dc
                if not(0<=rr<i.height and 0<=cc<i.width):continue
                if self.map_msg.data[rr*i.width+cc]!=-1:continue
                gr,gc=r-dr*offset,c-dc*offset
                if not self.free(gr,gc):continue
                x,y=self.cell_to_world(gr,gc)
                if self.goal_is_safe(x,y):candidates.append((x,y))
        return self.unique_points(candidates)

    def unique_points(self,points):
        out=[]
        for x,y in points:
            if not any(math.hypot(x-a,y-b)<0.15 for a,b in out):out.append((x,y))
        return out

    def information_gain(self,x,y):
        cell=self.world_to_cell(x,y)
        if cell is None:return 0
        r0,c0=cell
        i=self.map_msg.info
        radius=int(self.get_parameter('sensor_radius').value/i.resolution)
        gain=0;step=2
        for dr in range(-radius,radius+1,step):
            for dc in range(-radius,radius+1,step):
                if dr*dr+dc*dc>radius*radius:continue
                r,c=r0+dr,c0+dc
                if 0<=r<i.height and 0<=c<i.width and self.map_msg.data[r*i.width+c]==-1:gain+=step*step
        return gain

    def already_used(self,x,y):
        s=float(self.get_parameter('min_goal_separation').value)
        return any(math.hypot(x-gx,y-gy)<s for gx,gy in self.failed_goals+self.visited_goals)

    def rank_candidates(self,pose,components):
        rx,ry=pose;w=float(self.get_parameter('distance_weight').value)
        out=[]
        for comp in components:
            for x,y in self.candidate_points(comp):
                if self.already_used(x,y):continue
                gain=self.information_gain(x,y)
                if gain<=0:continue
                d=math.hypot(x-rx,y-ry)
                out.append((gain-w*d,gain,d,x,y))
        out.sort(reverse=True)
        return out[:int(self.get_parameter('max_planner_candidates').value)]

    def path_clearance(self,path):
        if self.map_msg is None or not path.poses:return 0.0
        res=self.map_msg.info.resolution
        required=float(self.get_parameter('min_path_clearance').value)
        radius=max(1,int(math.ceil(required/res)))
        i=self.map_msg.info
        minimum=float('inf')
        for pose in path.poses:
            x=pose.pose.position.x;y=pose.pose.position.y
            cell=self.world_to_cell(x,y)
            if cell is None:return 0.0
            r0,c0=cell
            local_min=float('inf')
            for dr in range(-radius,radius+1):
                for dc in range(-radius,radius+1):
                    if dr*dr+dc*dc>radius*radius:continue
                    r,c=r0+dr,c0+dc
                    if 0<=r<i.height and 0<=c<i.width and self.map_msg.data[r*i.width+c]>=50:
                        d=math.hypot(dr,dc)*res
                        local_min=min(local_min,d)
            minimum=min(minimum,local_min)
            if minimum<required:return minimum
        return minimum if minimum!=float('inf') else required

    def start_planner_check(self,candidates):
        if not candidates:
            self.decision_active=False
            return
        if not self.path_client.server_is_ready():
            self.decision_active=False
            return
        self.planner_check_queue=list(candidates)
        self.planner_check_index=0
        self.planner_check_active=True
        self.planner_check_started=None
        self.check_next_candidate()

    def check_next_candidate(self):
        if not self.planner_check_active:return
        if self.planner_check_index>=len(self.planner_check_queue):
            self.planner_check_active=False
            self.decision_active=False
            self.get_logger().warn('No frontier candidate has a valid path with sufficient clearance.')
            return
        score,gain,distance,x,y=self.planner_check_queue[self.planner_check_index]
        self.planner_check_index+=1
        goal=ComputePathToPose.Goal()
        goal.goal=PoseStamped()
        goal.goal.header.frame_id=self.map_msg.header.frame_id
        goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=x
        goal.goal.pose.position.y=y
        goal.goal.pose.position.z=0.0
        goal.goal.pose.orientation.w=1.0
        goal.use_start=False
        goal.planner_id=self.get_parameter('planner_id').value
        self.planner_check_started=time.monotonic()
        self.get_logger().info(f'Checking path: ({x:.2f},{y:.2f}) gain={gain:.0f} dist={distance:.2f}')
        future=self.path_client.send_goal_async(goal)
        future.add_done_callback(self.planner_goal_response)

    def planner_goal_response(self,future):
        if not self.planner_check_active:return
        try:h=future.result()
        except Exception as e:
            self.get_logger().warn(f'Planner request failed: {e}')
            self.check_next_candidate();return
        if not h.accepted:
            self.get_logger().warn('Planner feasibility request rejected.')
            self.check_next_candidate();return
        f=h.get_result_async()
        f.add_done_callback(self.planner_result)

    def planner_result(self,future):
        if not self.planner_check_active:return
        try:
            result=future.result()
            status=result.status
            path=result.result.path
            score,gain,distance,x,y=self.planner_check_queue[self.planner_check_index-1]
            if status==GoalStatus.STATUS_SUCCEEDED and path.poses:
                clearance=self.path_clearance(path)
                self.get_logger().info(f'Path result: poses={len(path.poses)} clearance={clearance:.2f}m')
                if clearance>=float(self.get_parameter('min_path_clearance').value):
                    self.get_logger().info(f'Path accepted: ({x:.2f},{y:.2f})')
                    self.planner_check_active=False
                    self.decision_active=False
                    self.send_goal(x,y)
                    return
                self.get_logger().warn(f'Path rejected: insufficient clearance at ({x:.2f},{y:.2f})')
            else:
                self.get_logger().warn(f'Path rejected: status={status}, poses={len(path.poses)}')
        except Exception as e:
            self.get_logger().warn(f'Planner result error: {e}')
        self.check_next_candidate()

    def exploration_cycle(self):
        if self.goal_active or self.decision_active or self.map_msg is None:return
        if not self.nav_client.server_is_ready() or not self.path_client.server_is_ready():return
        pose=self.robot_pose()
        if pose is None:return
        components=self.get_frontiers()
        if not components:
            self.get_logger().info('No unexplored frontier detected. Exploration complete.')
            return
        candidates=self.rank_candidates(pose,components)
        if not candidates:
            self.get_logger().info('No information-gain candidates available.')
            return
        self.decision_active=True
        self.start_planner_check(candidates)

    def send_goal(self,x,y):
        goal=NavigateToPose.Goal()
        goal.pose=PoseStamped()
        goal.pose.header.frame_id=self.map_msg.header.frame_id
        goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=x
        goal.pose.pose.position.y=y
        goal.pose.pose.position.z=0.0
        goal.pose.pose.orientation.w=1.0
        self.goal_active=True
        self.last_goal=(x,y)
        self.goal_sent_time=time.monotonic()
        self.get_logger().info(f'Sending reachable/clear viewpoint: ({x:.2f},{y:.2f})')
        f=self.nav_client.send_goal_async(goal,feedback_callback=self.feedback_callback)
        f.add_done_callback(self.goal_response)

    def goal_response(self,future):
        try:self.goal_handle=future.result()
        except Exception as e:
            self.get_logger().error(f'Navigation action failed: {e}')
            self.fail_goal();return
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected exploration viewpoint.')
            self.fail_goal();return
        self.get_logger().info('Exploration viewpoint accepted.')
        f=self.goal_handle.get_result_async()
        f.add_done_callback(self.goal_result)

    def goal_result(self,future):
        try:
            result=future.result()
            if result.status==GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Exploration viewpoint reached.')
                if self.last_goal:self.visited_goals.append(self.last_goal)
            else:
                self.get_logger().warn(f'Navigation ended with status {result.status}.')
                self.fail_goal()
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self.fail_goal()
        self.goal_active=False
        self.goal_handle=None
        self.goal_sent_time=None
        self.decision_active=False

    def fail_goal(self):
        if self.last_goal:
            x,y=self.last_goal
            s=float(self.get_parameter('min_goal_separation').value)
            if not any(math.hypot(x-gx,y-gy)<s for gx,gy in self.failed_goals):
                self.failed_goals.append((x,y))
                self.get_logger().warn(f'Blacklisting failed viewpoint ({x:.2f},{y:.2f}); failed={len(self.failed_goals)}')
        self.goal_active=False
        self.goal_handle=None
        self.goal_sent_time=None
        self.decision_active=False

    def check_goal_timeout(self):
        if not self.goal_active or self.goal_sent_time is None:return
        if time.monotonic()-self.goal_sent_time<float(self.get_parameter('goal_timeout').value):return
        self.get_logger().warn('Exploration viewpoint timed out. Canceling and blacklisting.')
        if self.goal_handle:
            f=self.goal_handle.cancel_goal_async()
            f.add_done_callback(self.cancel_callback)
        else:self.fail_goal()

    def cancel_callback(self,future):
        try:
            if future.result().goals_canceling:self.get_logger().info('Navigation cancellation accepted.')
        except Exception as e:self.get_logger().error(f'Cancellation error: {e}')
        self.fail_goal()

    def feedback_callback(self,feedback):
        pass

def main(args=None):
    rclpy.init(args=args)
    node=InformationGainExplorer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
