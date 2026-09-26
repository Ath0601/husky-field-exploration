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
        self.declare_parameter('costmap_topic','/global_costmap/costmap')
        self.declare_parameter('min_frontier_size',8)
        self.declare_parameter('candidate_offset_cells',3)
        self.declare_parameter('candidate_count_per_frontier',8)
        self.declare_parameter('max_planner_candidates',24)
        self.declare_parameter('sensor_radius',4.0)
        self.declare_parameter('information_weight',5.0)
        self.declare_parameter('distance_weight',2.0)
        self.declare_parameter('cost_weight',0.8)
        self.declare_parameter('heading_weight',0.4)
        self.declare_parameter('min_goal_separation',0.8)
        self.declare_parameter('failed_region_radius',1.5)
        self.declare_parameter('goal_clearance_cells',3)
        self.declare_parameter('goal_timeout',60.0)
        self.declare_parameter('planner_check_timeout',4.0)
        self.declare_parameter('planning_period',2.0)
        self.declare_parameter('planner_id','GridBased')
        self.map_msg=None
        self.costmap_msg=None
        self.goal_active=False
        self.goal_handle=None
        self.goal_sent_time=None
        self.last_goal=None
        self.failed_goals=[]
        self.visited_goals=[]
        self.decision_active=False
        self.planner_check_active=False
        self.planner_queue=[]
        self.planner_index=0
        self.planner_started=None
        self.planner_token=0
        self.planner_results=[]
        map_qos=QoSProfile(depth=1)
        map_qos.reliability=ReliabilityPolicy.RELIABLE
        map_qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
        cost_qos=QoSProfile(depth=1)
        cost_qos.reliability=ReliabilityPolicy.RELIABLE
        cost_qos.durability=DurabilityPolicy.VOLATILE
        self.map_sub=self.create_subscription(OccupancyGrid,self.get_parameter('map_topic').value,self.map_callback,map_qos)
        self.costmap_sub=self.create_subscription(OccupancyGrid,self.get_parameter('costmap_topic').value,self.costmap_callback,cost_qos)
        self.nav_client=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.path_client=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.timer=self.create_timer(float(self.get_parameter('planning_period').value),self.exploration_cycle)
        self.timeout_timer=self.create_timer(1.0,self.check_timers)
        self.get_logger().info('Costmap-aware information-gain frontier explorer started.')
        self.get_logger().info('No custom path-clearance rejection is used; Nav2 costmap decides path feasibility.')

    def map_callback(self,msg):
        if sum(v>=0 for v in msg.data)>=100:self.map_msg=msg

    def costmap_callback(self,msg):
        self.costmap_msg=msg

    def robot_pose(self):
        if self.map_msg is None:return None
        try:
            t=self.tf_buffer.lookup_transform(self.map_msg.header.frame_id,'base_link',rclpy.time.Time(),timeout=Duration(seconds=0.2))
            return t.transform.translation.x,t.transform.translation.y
        except TransformException:return None

    def cell_to_world(self,r,c):
        i=self.map_msg.info
        return i.origin.position.x+(c+0.5)*i.resolution,i.origin.position.y+(r+0.5)*i.resolution

    def world_to_cell(self,x,y,msg=None):
        m=msg or self.map_msg
        if m is None:return None
        i=m.info
        c=int((x-i.origin.position.x)/i.resolution)
        r=int((y-i.origin.position.y)/i.resolution)
        return (r,c) if 0<=r<i.height and 0<=c<i.width else None

    def cell_value(self,r,c,msg=None):
        m=msg or self.map_msg
        if m is None:return None
        i=m.info
        if not(0<=r<i.height and 0<=c<i.width):return None
        return m.data[r*i.width+c]

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
        out=[]
        while cells:
            seed=cells.pop()
            q=deque([seed]);comp=[seed]
            while q:
                r,c=q.popleft()
                for dr,dc in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                    n=(r+dr,c+dc)
                    if n in cells:
                        cells.remove(n);q.append(n);comp.append(n)
            if len(comp)>=minimum:out.append(comp)
        return out

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

    def candidate_points(self,comp):
        i=self.map_msg.info
        offset=int(self.get_parameter('candidate_offset_cells').value)
        limit=max(1,int(self.get_parameter('candidate_count_per_frontier').value))
        comp=sorted(comp)
        selected=[]
        if len(comp)<=limit:
            selected=comp
        else:
            for k in range(limit):
                idx=int(k*(len(comp)-1)/(limit-1)) if limit>1 else len(comp)//2
                selected.append(comp[idx])
        candidates=[]
        for r,c in selected:
            for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
                rr,cc=r+dr,c+dc
                if not(0<=rr<i.height and 0<=cc<i.width):continue
                if self.map_msg.data[rr*i.width+cc]!=-1:continue
                gr,gc=r-dr*offset,c-dc*offset
                if not self.free(gr,gc):continue
                x,y=self.cell_to_world(gr,gc)
                if self.goal_is_safe(x,y):
                    fx,fy=self.cell_to_world(r,c)
                    yaw=math.atan2(fy-y,fx-x)
                    candidates.append((x,y,yaw))
        return self.unique_points(candidates)

    def unique_points(self,points):
        out=[]
        for p in points:
            if not any(math.hypot(p[0]-q[0],p[1]-q[1])<0.35 for q in out):out.append(p)
        return out

    def information_gain(self,x,y):
        cell=self.world_to_cell(x,y)
        if cell is None:return 0
        r0,c0=cell
        i=self.map_msg.info
        radius=max(1,int(float(self.get_parameter('sensor_radius').value)/i.resolution))
        gain=0
        step=2
        for dr in range(-radius,radius+1,step):
            for dc in range(-radius,radius+1,step):
                if dr*dr+dc*dc>radius*radius:continue
                r,c=r0+dr,c0+dc
                if 0<=r<i.height and 0<=c<i.width and self.map_msg.data[r*i.width+c]==-1:gain+=1
        return gain

    def failed_region(self,x,y):
        radius=float(self.get_parameter('failed_region_radius').value)
        return any(math.hypot(x-gx,y-gy)<radius for gx,gy in self.failed_goals)

    def already_used(self,x,y):
        sep=float(self.get_parameter('min_goal_separation').value)
        return self.failed_region(x,y) or any(math.hypot(x-gx,y-gy)<sep for gx,gy in self.visited_goals)

    def candidate_heading_cost(self,rx,ry,x,y,yaw):
        desired=math.atan2(y-ry,x-rx)
        return abs(math.atan2(math.sin(yaw-desired),math.cos(yaw-desired)))

    def rank_initial_candidates(self,pose,components):
        rx,ry=pose
        iw=float(self.get_parameter('information_weight').value)
        dw=float(self.get_parameter('distance_weight').value)
        hw=float(self.get_parameter('heading_weight').value)
        out=[]
        for comp in components:
            for x,y,yaw in self.candidate_points(comp):
                if self.already_used(x,y):continue
                gain=self.information_gain(x,y)
                if gain<=0:continue
                d=math.hypot(x-rx,y-ry)
                h=self.candidate_heading_cost(rx,ry,x,y,yaw)
                pre=iw*gain-dw*d-hw*h
                out.append((pre,gain,d,h,x,y,yaw))
        out.sort(key=lambda v:v[0],reverse=True)
        return out[:int(self.get_parameter('max_planner_candidates').value)]

    def path_metrics(self,path):
        if not path.poses:return float('inf'),float('inf')
        length=0.0
        cost_sum=0.0
        cost_n=0
        prev=None
        cm=self.costmap_msg
        for ps in path.poses:
            x=ps.pose.position.x;y=ps.pose.position.y
            if prev is not None:length+=math.hypot(x-prev[0],y-prev[1])
            prev=(x,y)
            if cm is not None:
                cell=self.world_to_cell(x,y,cm)
                if cell is None:return float('inf'),float('inf')
                v=cm.data[cell[0]*cm.info.width+cell[1]]
                if v>=253:return float('inf'),float('inf')
                if v>=0:
                    cost_sum+=v
                    cost_n+=1
        return length,(cost_sum/cost_n if cost_n else 0.0)

    def make_pose(self,x,y,yaw):
        p=PoseStamped()
        p.header.frame_id=self.map_msg.header.frame_id
        p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=0.0
        p.pose.orientation.z=math.sin(yaw/2.0)
        p.pose.orientation.w=math.cos(yaw/2.0)
        return p

    def start_planner_check(self,candidates):
        if not candidates:
            self.decision_active=False
            self.get_logger().warn('No usable frontier candidates.')
            return
        self.planner_queue=list(candidates)
        self.planner_index=0
        self.planner_results=[]
        self.planner_check_active=True
        self.check_next_candidate()

    def check_next_candidate(self):
        if not self.planner_check_active:return
        if self.planner_index>=len(self.planner_queue):
            self.finish_planner_checks();return
        item=self.planner_queue[self.planner_index]
        self.planner_index+=1
        _,gain,distance,heading,x,y,yaw=item
        goal=ComputePathToPose.Goal()
        goal.goal=self.make_pose(x,y,yaw)
        goal.use_start=False
        goal.planner_id=str(self.get_parameter('planner_id').value)
        self.planner_token+=1
        token=self.planner_token
        self.planner_started=time.monotonic()
        self.get_logger().info(f'Checking candidate {self.planner_index}/{len(self.planner_queue)}: ({x:.2f},{y:.2f}) gain={gain} dist={distance:.2f}')
        future=self.path_client.send_goal_async(goal)
        future.add_done_callback(lambda f,t=token:self.planner_goal_response(f,t))

    def planner_goal_response(self,future,token):
        if not self.planner_check_active or token!=self.planner_token:return
        try:h=future.result()
        except Exception as e:
            self.get_logger().warn(f'Planner request failed: {e}')
            self.check_next_candidate();return
        if not h.accepted:
            self.get_logger().warn('Planner feasibility request rejected.')
            self.check_next_candidate();return
        future2=h.get_result_async()
        future2.add_done_callback(lambda f,t=token:self.planner_result(f,t))

    def planner_result(self,future,token):
        if not self.planner_check_active or token!=self.planner_token:return
        try:
            result=future.result()
            path=result.result.path
            item=self.planner_queue[self.planner_index-1]
            _,gain,distance,heading,x,y,yaw=item
            if result.status==GoalStatus.STATUS_SUCCEEDED and path.poses:
                length,cost=self.path_metrics(path)
                if math.isfinite(length) and math.isfinite(cost):
                    self.planner_results.append((gain,distance,heading,length,cost,x,y,yaw,len(path.poses)))
                    self.get_logger().info(f'  VALID path: length={length:.2f}m avg_cost={cost:.1f} poses={len(path.poses)}')
                else:self.get_logger().warn('  Rejected: path enters lethal/invalid global-costmap cells.')
            else:self.get_logger().warn(f'  Rejected by Nav2 planner: status={result.status}')
        except Exception as e:self.get_logger().warn(f'Planner result error: {e}')
        self.check_next_candidate()

    def finish_planner_checks(self):
        self.planner_check_active=False
        if not self.planner_results:
            self.decision_active=False
            self.get_logger().warn('No candidate produced a usable Nav2 path.')
            return
        iw=float(self.get_parameter('information_weight').value)
        dw=float(self.get_parameter('distance_weight').value)
        cw=float(self.get_parameter('cost_weight').value)
        hw=float(self.get_parameter('heading_weight').value)
        scored=[]
        for gain,distance,heading,length,cost,x,y,yaw,n in self.planner_results:
            score=iw*gain-dw*length-cw*cost-hw*heading
            scored.append((score,gain,distance,heading,length,cost,x,y,yaw,n))
        scored.sort(key=lambda v:v[0],reverse=True)
        score,gain,distance,heading,length,cost,x,y,yaw,n=scored[0]
        self.get_logger().info(f'BEST candidate: ({x:.2f},{y:.2f}) score={score:.1f} gain={gain} length={length:.2f} cost={cost:.1f}')
        self.decision_active=False
        self.send_goal(x,y,yaw)

    def exploration_cycle(self):
        if self.goal_active or self.decision_active or self.map_msg is None:return
        if not self.nav_client.server_is_ready() or not self.path_client.server_is_ready():return
        pose=self.robot_pose()
        if pose is None:return
        components=self.get_frontiers()
        if not components:
            self.get_logger().info('No unexplored frontier detected. Exploration complete.')
            return
        candidates=self.rank_initial_candidates(pose,components)
        if not candidates:
            self.get_logger().info('No useful frontier candidates available.')
            return
        self.decision_active=True
        self.start_planner_check(candidates)

    def send_goal(self,x,y,yaw):
        goal=NavigateToPose.Goal()
        goal.pose=self.make_pose(x,y,yaw)
        self.goal_active=True
        self.last_goal=(x,y)
        self.goal_sent_time=time.monotonic()
        self.get_logger().info(f'Sending exploration goal: ({x:.2f},{y:.2f}) yaw={yaw:.2f}')
        future=self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self,future):
        try:self.goal_handle=future.result()
        except Exception as e:
            self.get_logger().error(f'Navigation action failed: {e}')
            self.fail_goal();return
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected exploration goal.')
            self.fail_goal();return
        self.get_logger().info('Exploration goal accepted.')
        future2=self.goal_handle.get_result_async()
        future2.add_done_callback(self.goal_result)

    def goal_result(self,future):
        try:
            result=future.result()
            if result.status==GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Exploration goal reached.')
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
            radius=float(self.get_parameter('failed_region_radius').value)
            if not any(math.hypot(x-gx,y-gy)<radius for gx,gy in self.failed_goals):
                self.failed_goals.append((x,y))
                self.get_logger().warn(f'Blacklisting failed region around ({x:.2f},{y:.2f}); radius={radius:.1f}m')
        self.goal_active=False
        self.goal_handle=None
        self.goal_sent_time=None
        self.decision_active=False

    def check_timers(self):
        if self.goal_active and self.goal_sent_time is not None:
            if time.monotonic()-self.goal_sent_time>float(self.get_parameter('goal_timeout').value):
                self.get_logger().warn('Exploration goal timed out. Canceling and blacklisting.')
                if self.goal_handle:
                    future=self.goal_handle.cancel_goal_async()
                    future.add_done_callback(self.cancel_callback)
                else:self.fail_goal()
        if self.planner_check_active and self.planner_started is not None:
            if time.monotonic()-self.planner_started>float(self.get_parameter('planner_check_timeout').value):
                self.get_logger().warn('Planner check timed out; trying next candidate.')
                self.planner_token+=1
                self.check_next_candidate()

    def cancel_callback(self,future):
        try:
            if future.result().goals_canceling:self.get_logger().info('Navigation cancellation accepted.')
        except Exception as e:self.get_logger().error(f'Cancellation error: {e}')
        self.fail_goal()

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
