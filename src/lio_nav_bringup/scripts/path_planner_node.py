#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import heapq
import math

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Parameters
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('plan_unknown', True) # Implement "Unknown is Free" logic
        self.declare_parameter('lookahead_dist', 1.0)
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('goal_tolerance', 0.4) # Stop when within this distance of goal
        self.declare_parameter('cost_weight', 2.0)   # Weight for obstacle avoidance (0.0 = shortest path)
        self.declare_parameter('enable_movement', False) # If False, only plan path, don't move
        
        # State
        self.map_data = None
        self.map_info = None
        self.map_header = None
        self.robot_pose = None
        self.goal_pose = None
        self.current_path = []
        
        # QoS
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/costmap', self.map_callback, qos_reliable)
        self.create_subscription(Odometry, '/Odometry', self.odom_callback, qos_reliable)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, qos_reliable)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Direct to twist_mux
        
        # Timer
        freq = self.get_parameter('planning_frequency').value
        self.timer = self.create_timer(1.0/freq, self.planning_loop)
        
        control_freq = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(1.0/control_freq, self.control_loop)
        
        self.get_logger().info("Path Planner Node Started. Waiting for Map, Odom, and Goal...")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8)
        self.map_info = msg.info
        self.map_header = msg.header

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"New Goal Received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        # Trigger immediate replan
        self.planning_loop()

    def world_to_grid(self, wx, wy):
        if self.map_info is None: return None
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return (mx, my)
        return None

    def grid_to_world(self, mx, my):
        if self.map_info is None: return None
        wx = (mx * self.map_info.resolution) + self.map_info.origin.position.x + (self.map_info.resolution / 2.0)
        wy = (my * self.map_info.resolution) + self.map_info.origin.position.y + (self.map_info.resolution / 2.0)
        return (wx, wy)

    def is_valid(self, mx, my):
        # 0 = Free, 100 = Occupied, -1 = Unknown
        # User wants to treat Unknown (-1) as TRAVERSABLE.
        index = my * self.map_info.width + mx
        val = self.map_data[index]
        
        # Obstacle Check
        if val >= 50: # Occupied
            return False
            
        # If we want to strictly ONLY allow Free(0) and Unknown(-1):
        if val == 0 or val == -1:
            return True
            
        return val < 50 # Allow decayed inflation areas

    def planning_loop(self):
        if self.map_info is None or self.robot_pose is None or self.goal_pose is None:
            return

        # 1. Start and Goal in Grid
        start_grid = self.world_to_grid(self.robot_pose.position.x, self.robot_pose.position.y)
        goal_grid = self.world_to_grid(self.goal_pose.position.x, self.goal_pose.position.y)

        if start_grid is None:
            self.get_logger().warn("Robot is outside map bounds!", throttle_duration_sec=2.0)
            return
        if goal_grid is None:
            self.get_logger().warn("Goal is outside map bounds!", throttle_duration_sec=2.0)
            # Optional: Clamp goal to map edge? For now, just return.
            return
            
        # Check if Goal is reached (Pre-check)
        dist_to_goal = math.sqrt((self.robot_pose.position.x - self.goal_pose.position.x)**2 + 
                                 (self.robot_pose.position.y - self.goal_pose.position.y)**2)
        goal_tolerance = self.get_parameter('goal_tolerance').value
        
        if dist_to_goal < goal_tolerance:
             self.current_path = []
             self.stop_robot()
             return

        # 2. A* Algorithm
        path = self.run_astar(start_grid, goal_grid)

        # 3. Publish Path
        if path:
            path_msg = Path()
            path_msg.header.frame_id = self.map_header.frame_id
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for (gx, gy) in path:
                wx, wy = self.grid_to_world(gx, gy)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.orientation.w = 1.0 # No orientation for path points
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
            self.current_path = path_msg.poses
        else:
            self.get_logger().warn("No path found to goal!", throttle_duration_sec=2.0)
            self.current_path = [] # Clear path if failed

    def run_astar(self, start, goal):
        # start, goal are (x, y) tuples
        width = self.map_info.width
        height = self.map_info.height
        cost_weight = self.get_parameter('cost_weight').value
        
        # Priority Queue: (f_score, x, y)
        open_list = []
        heapq.heappush(open_list, (0, start[0], start[1]))
        
        came_from = {}
        g_score = {start: 0}
        
        # 8-Connectivity Neighbors
        neighbors = [
            (0,1), (0,-1), (1,0), (-1,0),
            (1,1), (1,-1), (-1,1), (-1,-1)
        ]
        
        # Optimization: Limit iterations to prevent freezing if path is huge/unreachable
        iterations = 0
        max_iterations = 20000 
        
        while open_list:
            iterations += 1
            if iterations > max_iterations:
                self.get_logger().warn("A* Search timed out (too many iterations)")
                return None
                
            current_f, cx, cy = heapq.heappop(open_list)
            current = (cx, cy)
            
            if current == goal:
                # Reconstruct Path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)
                
                # Bounds and Obstacle Check
                if 0 <= nx < width and 0 <= ny < height:
                    if self.is_valid(nx, ny):
                        # Distance Cost: 1.0 for straight, 1.414 for diagonal
                        dist_cost = math.sqrt(dx*dx + dy*dy)
                        
                        # --- Cost Weighting ---
                        idx = ny * width + nx
                        val = self.map_data[idx]
                        
                        penalty_cost = 0.0
                        if val > 0 and val < 50:
                            # val is 1~49. Higher is closer to obstacle.
                            # We want to penalize higher values.
                            # A normalized penalty: (val / 50.0) * cost_weight
                            penalty_cost = (val / 50.0) * cost_weight
                        
                        # Note: Unknown (-1) is treated as 0 penalty here (preferred over obstacle)
                        
                        move_cost = dist_cost + penalty_cost
                        # -----------------------
                        
                        tentative_g_score = g_score[current] + move_cost
                        
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            
                            # Heuristic (Euclidean)
                            h_score = math.sqrt((nx - goal[0])**2 + (ny - goal[1])**2)
                            f_score = tentative_g_score + h_score
                            
                            heapq.heappush(open_list, (f_score, nx, ny))
                            
        return None

    def get_robot_yaw(self):
        if self.robot_pose is None: return 0.0
        q = self.robot_pose.orientation
        # Yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.current_path or self.robot_pose is None:
            # Safety stop if no path or pose
            # self.stop_robot() # Optional: keep stopping?
            return

        # Check if movement is enabled
        if not self.get_parameter('enable_movement').value:
            self.stop_robot()
            return

        # 1. Find Lookahead Point
        lookahead_dist = self.get_parameter('lookahead_dist').value
        goal_point = None
        
        # Simple search for first point further than lookahead_dist
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_yaw = self.get_robot_yaw()

        target_idx = -1
        
        for i, pose in enumerate(self.current_path):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            
            if dist >= lookahead_dist:
                target_idx = i
                break
        
        # If reachable point found
        if target_idx != -1:
            goal_point = self.current_path[target_idx].pose.position
        else:
            # If all points are closer than lookahead, pick the last one (End of path)
            if len(self.current_path) > 0:
                 goal_point = self.current_path[-1].pose.position
                 
        if goal_point is None:
            self.stop_robot()
            return

        # Check if we are close enough to the FINAL goal
        # Check if we are close enough to the FINAL goal
        final_pose = self.current_path[-1].pose.position
        dist_to_final = math.sqrt((final_pose.x - robot_x)**2 + (final_pose.y - robot_y)**2)
        
        goal_tolerance = self.get_parameter('goal_tolerance').value
        if dist_to_final < goal_tolerance: 
            self.get_logger().info("Goal Reached!")
            self.current_path = [] # Clear path
            self.stop_robot()
            return

        # 2. Compute Pure Pursuit
        dx = goal_point.x - robot_x
        dy = goal_point.y - robot_y
        
        # Transform to robot frame
        alpha = math.atan2(dy, dx) - robot_yaw
        
        # Normalize angle -pi to pi
        while alpha > math.pi: alpha -= 2.0 * math.pi
        while alpha < -math.pi: alpha += 2.0 * math.pi
        
        # 3. Compute Steering (Angular Velocity)
        # alpha is the angle to the goal in robot frame.
        k_p = 2.0 # Proportional gain
        angular_vel = k_p * alpha
        
        # Limit max angular velocity
        max_ang = 1.0
        angular_vel = max(-max_ang, min(max_ang, angular_vel))
        
        # 4. Compute Linear Velocity
        # Slow down on sharp turns
        max_speed = self.get_parameter('max_speed').value
        linear_vel = max_speed * (1.0 - abs(angular_vel)/max_ang * 0.5) # Minimum half speed
        
        if linear_vel < 0.0: linear_vel = 0.0

        # Publish
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    print("DEBUG: Entered main function", flush=True)
    rclpy.init(args=args)
    node = PathPlannerNode()
    print("DEBUG: Node initialized, starting spin", flush=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    rclpy.shutdown()

if __name__ == '__main__':
    print("DEBUG: Entered __main__", flush=True)
    main()
