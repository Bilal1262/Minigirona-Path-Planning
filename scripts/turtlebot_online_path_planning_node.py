#!/usr/bin/python3

import numpy as np
import rospy
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from online_planning import StateValidityChecker, move_to_point, compute_path, wrap_angle
from online_planning import DWAPlanner


class OnlinePlanner:
    """
    OnlinePlanner class for path planning and control of a robot.
    """

    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, dominion, distance_threshold):
        """
        Constructor for the OnlinePlanner class.

        Args:
            gridmap_topic (str): Topic for the occupancy grid map.
            odom_topic (str): Topic for the robot's odometry.
            cmd_vel_topic (str): Topic to publish velocity commands.
            dominion (np.array): Dominion [min_x, max_x, min_y, max_y] for path planning.
            distance_threshold (float): Distance threshold for collision checking.
        """
        # Initialize DWA Planner
        self.dwa_planner = DWAPlanner(
            robot_radius=distance_threshold,
            max_lin_vel=0.15,
            max_ang_vel=0.3
        )

        # ATTRIBUTES
        self.path = []  # List of points defining the plan
        self.svc = StateValidityChecker(distance_threshold, is_unknown_valid=True)  # State validity checker
        self.current_pose = None  # Current robot pose [x, y, yaw]
        self.goal = None  # Goal position [x, y]
        self.last_map_time = rospy.Time(0)  # Last time a map was received
        self.dominion = dominion  # Dominion for path planning

        # CONTROLLER PARAMETERS
        self.Kv = 0.5  # Proportional linear velocity gain
        self.Kw = 0.5  # Proportional angular velocity gain
        self.v_max = 0.15  # Maximum linear velocity
        self.w_max = 0.3  # Maximum angular velocity

        # PUBLISHERS
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)  # Velocity command publisher
        self.marker_pub = rospy.Publisher('/path_marker', Marker, queue_size=1)  # Path visualization publisher

        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)  # Occupancy grid subscriber
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)  # Odometry subscriber
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)  # Goal subscriber

        # TIMERS
        rospy.Timer(rospy.Duration(0.1), self.controller)  # Timer for velocity controller

    def get_odom(self, odom: Odometry):
        """
        Callback for odometry data. Updates the current robot pose.

        Args:
            odom (Odometry): Odometry message.
        """
        _, _, yaw = tf.transformations.euler_from_quaternion([
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ])

        # Store current position [x, y, yaw]
        self.current_pose = np.array([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            yaw
        ])

    def get_gridmap(self, gridmap: OccupancyGrid):
        """
        Callback for occupancy grid map. Updates the state validity checker.

        Args:
            gridmap (OccupancyGrid): Occupancy grid map message.
        """
        # Avoid updating the map too often
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 0.1:
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)

            # Check if the current path is still valid
            if len(self.path) > 0:
                total_path = [self.current_pose[:2]] + self.path
                if not self.svc.check_path(total_path):
                    print("Invalid path. Replanning...")
                    self.__send_commnd__(0, 0)
                    self.plan()

    def get_goal(self, goal: PoseStamped):
        """
        Callback for goal position. Computes a plan to the new goal.

        Args:
            goal (PoseStamped): Goal position message.
        """
        if self.svc.there_is_map:
            # Store the goal position
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            rospy.loginfo(f"New goal received: {self.goal}")

            # Check if current pose is available
            if self.current_pose is None:
                rospy.logwarn("Current pose not available. Cannot plan yet.")
                return
            # Plan a new path to the goal
            self.plan()

    def plan(self):
        """
        Plans a path from the current position to the goal.
        """
        # Invalidate previous plan
        self.path = []

        if not self.svc.there_is_map or self.current_pose is None or self.goal is None:
            rospy.logwarn("Skipping planning: Map, pose, or goal not available!")
            return

        print("Computing new path...")
        self.path = compute_path(self.current_pose[:2], self.goal, self.svc, self.dominion, max_iterations=2000)

        # Retry planning if it fails
        if len(self.path) == 0:
            rospy.logwarn("Initial planning failed. Retrying...")
            for _ in range(3):
                self.path = compute_path(self.current_pose[:2], self.goal, self.svc, self.dominion, max_iterations=5000)
                if len(self.path) > 0:
                    break

        if len(self.path) == 0:
            print("Path not found!")
        else:
            print("Path found!")
            self.publish_path()  # Visualize the path
            del self.path[0]  # Remove the initial waypoint (current pose)

    def controller(self, event):
        """
        Timer callback for velocity control. Computes and sends velocity commands.

        Args:
            event: Timer event.
        """
        v = 0
        w = 0

        if len(self.path) == 0:
            return  # Exit if no path is available

        current_target = self.path[0]
        current_position = self.current_pose[:2]

        # Check if the current waypoint is reached
        if np.linalg.norm(current_target - current_position) < 0.1:
            self.path.pop(0)

            if len(self.path) == 0:
                print("Goal reached!")
                self.__send_commnd__(0, 0)
                return
            else:
                print("Waypoint reached. Moving to the next point.")
                current_target = self.path[0]

        # Compute heading and angle
        dx = current_target[0] - self.current_pose[0]
        dy = current_target[1] - self.current_pose[1]
        heading = math.atan2(dy, dx)
        angle = heading - self.current_pose[2]
        angle = (angle + math.pi) % (2 * math.pi) - math.pi  

        if abs(angle) > 0.3:
            self.__send_commnd__(0, 0.2*np.sign(angle))
            
        else:

            # Compute velocity
            v, w = move_to_point(self.current_pose, current_target, self.Kv, self.Kw)
            
            v = np.clip(v, -self.v_max, self.v_max)
            w = np.clip(w, -self.w_max, self.w_max)
            
            self.__send_commnd__(v, w)

    def __send_commnd__(self, v, w):
        """
        Sends velocity commands to the robot.

        Args:
            v (float): Linear velocity.
            w (float): Angular velocity.
        """
        cmd = Twist()
        cmd.linear.x = v
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = -w
        self.cmd_pub.publish(cmd)

    def publish_path(self):
        """
        Publishes the path as a series of line markers for visualization.
        """
        if len(self.path) > 0:
            print("Publishing path...")
            m = Marker()
            m.header.frame_id = 'world_ned'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            m.pose.orientation.w = 1

            # Define colors
            color_red = ColorRGBA(1, 0, 0, 1)
            color_blue = ColorRGBA(0, 0, 1, 1)

            # Add current pose to the path
            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)

            # Add path waypoints
            for n in self.path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)

            self.marker_pub.publish(m)
        else:
            print("No path available.")

if __name__ == '__main__':
    rospy.init_node('turtlebot_online_path_planning_node')
    node = OnlinePlanner('/projected_map', '/turtlebot/odom_ground_truth', '/turtlebot/kobuki/commands/velocity', np.array([-10.0, 10.0, -10.0, 10.0]), 0.2)
    rospy.spin()