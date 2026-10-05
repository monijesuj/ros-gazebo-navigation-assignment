"""GPS corrected odometry, live occupancy updates, A* and waypoint control."""
import json
import math
import time
from pathlib import Path as FilePath
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from .planning import Grid, geodetic_to_xy, wrap_angle, clearance


class Navigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('autostart', True)
        self.cfg = json.loads(FilePath(self.get_parameter('mission_file').value).read_text())
        self.grid = Grid(self.cfg)
        self.goals = [geodetic_to_xy(*v, self.cfg['datum']) for v in self.cfg['waypoints_gps']] if self.cfg['waypoint_mode'] == 'gps' else self.cfg['waypoints_world']
        self.odom = None
        self.offset = list(self.cfg['start'])
        self.xy = tuple(self.offset)
        self.yaw = 0.
        self.seen = {}
        self.counts = {'odom':0, 'lidar':0, 'depth':0, 'gps':0}
        self.front_range = self.depth_range = float('inf')
        self.path = []
        self.target_index = 1
        self.goal_index = 0
        self.enabled = self.get_parameter('autostart').value
        self.state = 'WAITING_FOR_SENSORS'
        self.plan_time = -10.
        self.min_clearance = float('inf')
        self.travelled = 0.
        self.previous_xy = None
        self.started = None
        self.finished = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.path_pub = self.create_publisher(Path, '/planned_path', latched)
        self.trail_pub = self.create_publisher(Path, '/travelled_path', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.trail = Path(); self.trail.header.frame_id = 'map'
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth_image', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.create_service(SetBool, '/mission/enable', self.on_enable)
        self.create_timer(.05, self.control)
        self.create_timer(1., self.publish_map_status)
        self.get_logger().info(f'Loaded {len(self.goals)} {self.cfg["waypoint_mode"]} waypoints; waiting for odometry, lidar, depth and GNSS')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def touch(self, sensor):
        self.seen[sensor] = time.monotonic()
        self.counts[sensor] += 1

    def on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.cmd.publish(Twist())
        response.success = True
        response.message = 'Mission enabled' if self.enabled else 'Mission paused'
        return response

    def on_odom(self, msg):
        self.touch('odom'); self.odom = msg
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.xy = (msg.pose.pose.position.x+self.offset[0], msg.pose.pose.position.y+self.offset[1])
        tf = TransformStamped(); tf.header.stamp = msg.header.stamp; tf.header.frame_id = 'map'; tf.child_frame_id = 'odom'
        tf.transform.translation.x, tf.transform.translation.y = self.offset
        tf.transform.rotation.w = 1.; self.tf.sendTransform(tf)
        if self.previous_xy is not None and self.started is not None and self.finished is None:
            self.travelled += math.dist(self.xy, self.previous_xy)
            self.min_clearance = min(self.min_clearance, clearance(self.xy,self.cfg['obstacles'])-self.cfg['robot_radius'])
        self.previous_xy = self.xy

    def on_gps(self, msg):
        if msg.status.status < 0 or not all(math.isfinite(v) for v in (msg.latitude,msg.longitude)):
            return
        gx, gy = geodetic_to_xy(msg.latitude,msg.longitude,self.cfg['datum'])
        if self.odom is not None:
            px, py = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
            estimate = gx-px, gy-py
            # Reject gross outliers. GPS antenna has zero XY lever arm.
            if math.dist(estimate,self.offset) > 1.5:
                return
            self.offset = [.9*old+.1*new for old,new in zip(self.offset,estimate)]
        self.touch('gps')

    def on_scan(self, msg):
        if self.odom is None:
            return
        valid = [(msg.angle_min+i*msg.angle_increment,r) for i,r in enumerate(msg.ranges) if math.isfinite(r) and msg.range_min < r < msg.range_max]
        front = [r for a,r in valid if abs(a)<.40]
        self.front_range = min(front,default=float('inf'))
        for angle, r in valid[::3]:
            self.grid.observe((self.xy[0]+r*math.cos(self.yaw+angle), self.xy[1]+r*math.sin(self.yaw+angle)),self.now())
        self.touch('lidar')

    def on_depth(self, msg):
        if msg.encoding not in ('32FC1','16UC1'):
            return
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4') if msg.encoding=='32FC1' else np.dtype('>u2' if msg.is_bigendian else '<u2')
        try:
            image = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.step,dtype.itemsize))
        except (ValueError,TypeError):
            return
        # Central horizontal band avoids the floor while covering the forward corridor.
        roi = image[int(.30*msg.height):int(.58*msg.height),int(.20*msg.width):int(.80*msg.width)].astype(float)
        if msg.encoding=='16UC1': roi *= .001
        values = roi[np.isfinite(roi) & (roi>.15) & (roi<6.)]
        self.depth_range = float(np.percentile(values,5)) if len(values) else float('inf')
        self.touch('depth')

    def make_pose(self, xy):
        pose = PoseStamped(); pose.header.frame_id = 'map'; pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = map(float,xy); pose.pose.orientation.w = 1.
        return pose

    def replan(self):
        self.path = self.grid.plan(self.xy,self.goals[self.goal_index],self.now())
        self.target_index = 1
        self.plan_time = self.now()
        path = Path(); path.header.frame_id = 'map'; path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.make_pose(p) for p in self.path]; self.path_pub.publish(path)
        self.get_logger().info(f'A* path: {len(self.path)} vertices; goal {self.goal_index+1}')

    def control(self):
        cmd = Twist()
        fresh = all(time.monotonic()-self.seen.get(s,-1e9)<2.0 for s in ('odom','lidar','depth','gps'))
        if self.finished is not None:
            self.state = 'SUCCEEDED'
        elif not fresh:
            self.state = 'WAITING_FOR_SENSORS'
        elif not self.enabled:
            self.state = 'PAUSED'
        else:
            if self.started is None: self.started = self.now()
            if math.dist(self.xy,self.goals[self.goal_index]) < self.cfg['goal_tolerance']:
                self.goal_index += 1; self.path = []
                if self.goal_index == len(self.goals):
                    self.finished = self.now(); self.state = 'SUCCEEDED'
                    self.get_logger().info('Mission SUCCEEDED: final waypoint reached')
                    self.cmd.publish(cmd); self.publish_map_status(); return
            blocked = self.grid.inflated(self.now()) if self.now()-self.plan_time>1. else None
            unsafe = blocked is not None and self.path and not self.grid.line_free(self.xy,self.path[self.target_index],blocked)
            if (not self.path or unsafe) and self.now()-self.plan_time>1.:
                self.replan()
            if not self.path:
                self.state = 'NO_PATH'
            else:
                while self.target_index<len(self.path)-1 and math.dist(self.xy,self.path[self.target_index])<.18:
                    self.target_index += 1
                tx,ty = self.path[self.target_index]
                error = wrap_angle(math.atan2(ty-self.xy[1],tx-self.xy[0])-self.yaw)
                cmd.angular.z = max(-.7,min(.7,1.8*error))
                cmd.linear.x = min(self.cfg['max_speed'], .8*math.dist(self.xy,(tx,ty))) * max(0.,math.cos(error)) if abs(error)<.45 else 0.
                # At 0.32 m/s the stop distances exceed braking + one sensor period.
                if self.front_range<.70 or self.depth_range<.48:
                    cmd.linear.x = 0.; self.state = 'SAFETY_STOP'
                else:
                    self.state = 'NAVIGATING'
        self.cmd.publish(cmd)

    def publish_map_status(self):
        grid = OccupancyGrid(); grid.header.frame_id = 'map'; grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.grid.resolution; grid.info.width = self.grid.width; grid.info.height = self.grid.height
        grid.info.origin.position.x = self.grid.xmin; grid.info.origin.position.y = self.grid.ymin; grid.info.origin.orientation.w = 1.
        grid.data = (self.grid.occupied(self.now()).astype(np.int8)*100).flatten().tolist(); self.map_pub.publish(grid)
        if self.odom:
            self.trail.poses.append(self.make_pose(self.xy)); self.trail.header.stamp = grid.header.stamp
            self.trail_pub.publish(self.trail)
        goal = self.goals[min(self.goal_index,len(self.goals)-1)]
        status = {'state':self.state,'xy':self.xy,'goal_xy':goal,'goal_error_m':math.dist(self.xy,goal),'waypoints_reached':self.goal_index,'sensor_messages':self.counts,'elapsed_sim_s':None if self.started is None else (self.finished or self.now())-self.started,'travelled_m':self.travelled,'minimum_obstacle_clearance_m':None if not math.isfinite(self.min_clearance) else self.min_clearance,'lidar_front_m':self.front_range if math.isfinite(self.front_range) else None,'depth_front_m':self.depth_range if math.isfinite(self.depth_range) else None}
        msg = String(); msg.data = json.dumps(status); self.status_pub.publish(msg)


def main():
    rclpy.init(); node = Navigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd.publish(Twist()); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
