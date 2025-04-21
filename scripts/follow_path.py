#!/usr/bin/env python3
import rospy
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from cola2_msgs.msg import BodyVelocityReq
from tf.transformations import euler_from_quaternion
from online_planning import StateValidityChecker, wrap_angle
from pid_controller import PIDController

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)
        
        # Initialize state validity checker
        self.svc = StateValidityChecker(distance=0.2, is_unknown_valid=True)
        
        # Controller parameters
        self.Kv = 1.0  # Proportional linear velocity gain
        self.Kw = 0.5  # Proportional angular velocity gain
        self.v_max = 5.0  # Maximum linear velocity
        self.w_max = 1.0  # Maximum angular velocity
        
        # Current path and pose
        self.current_path = None
        self.current_pose = None
        self.current_waypoint_idx = 0

        self.pid_controller = PIDController()
        
        # Subscribers and publishers
        self.path_sub = rospy.Subscriber('/minigirona/planner/path', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/minigirona/navigator/odometry', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/minigirona/controller/body_velocity_req', BodyVelocityReq, queue_size=10)
        
    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("Received empty path")
            return
            
        self.current_path = msg.poses
        self.current_waypoint_idx = 0
        self.pid_controller.reset()
        rospy.loginfo(f"Received new path with {len(self.current_path)} waypoints")
        
    def odom_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Extract orientation (yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        self.current_pose = np.array([x, y, z, yaw])
        
        if self.current_path is not None:
            self.follow_path()
    
    def follow_path(self):
        if self.current_waypoint_idx >= len(self.current_path):
            rospy.loginfo("Reached end of path")
            self.__send_command__(0, 0, 0, 0, 0, 0)
            return
            
        current_waypoint = self.current_path[self.current_waypoint_idx]
        
        # Get waypoint position
        waypoint_pos = np.array([
            current_waypoint.pose.position.x,
            current_waypoint.pose.position.y,
            current_waypoint.pose.position.z
        ])
        
        # Check if current waypoint is valid
        if not self.svc.is_valid(waypoint_pos):
            rospy.logwarn(f"Waypoint {self.current_waypoint_idx} is not valid, skipping")
            self.current_waypoint_idx += 1

            #but here we will call the from the planner to replan for this again this for now

            return
            
        # Check if we've reached the current waypoint
        distance_to_waypoint = np.linalg.norm(waypoint_pos[:2] - self.current_pose[:2])
        
        if distance_to_waypoint < 0.1:  # Waypoint reached threshold
            rospy.loginfo(f"Reached waypoint {self.current_waypoint_idx}")
            self.current_waypoint_idx += 1
            return
            
        # Compute heading and angle to waypoint (only in x-y plane for underwater vehicle)
        dx = waypoint_pos[0] - self.current_pose[0]
        dy = waypoint_pos[1] - self.current_pose[1]
        heading = math.atan2(dy, dx)
        angle_error = wrap_angle(heading - self.current_pose[3])
        
        # # First correct orientation, then move forward
        # if abs(angle_error) > 0.3:  # Orientation threshold in radians
        #     # Only rotate to face the waypoint
        #     self.__send_command__(0, 0, 0, 0, 0, 0.2 * np.sign(angle_error))
        # else:
        #     # Move forward while maintaining orientation
        #     # Compute distance in x-y plane
        #     distance_xy = np.linalg.norm(waypoint_pos[:2] - self.current_pose[:2])
            
        #     # Compute velocities (simple proportional control)
        #     v_forward = min(self.Kv * distance_xy, self.v_max)
        #     w_z = self.Kw * angle_error
            
        #     # Also control depth if needed
        #     depth_error = waypoint_pos[2] - self.current_pose[2]
        #     v_z = min(self.Kv * abs(depth_error), self.v_max) * np.sign(depth_error)

        #     # Send command (only using x, z linear and z angular for underwater vehicle)
        #     self.__send_command__(v_forward, 0, v_z, 0, 0, w_z)

        target_pose = {
            'x': waypoint_pos[0],
            'y': waypoint_pos[1],
            'z': waypoint_pos[2],
            'theta': self.current_pose[3] # Maintain heading toward waypoint
        }
        
        current_pose_dict = {
            'x': self.current_pose[0],
            'y': self.current_pose[1],
            'z': self.current_pose[2],
            'theta': self.current_pose[3]
        }
        
        # Compute velocities using PID controller
        velocities = self.pid_controller.compute_velocities(current_pose_dict, target_pose)
        
        vx = velocities['linear']['x']
        vy = velocities['linear']['y']
        vz= velocities['linear']['z']

        wz= velocities['angular']['z']
        # Send command (only using x, z linear and z angular for underwater vehicle)
        self.__send_command__(vx, vy, vz, 0, 0, wz)
    
    def __send_command__(self, vx, vy, vz, wx, wy, wz):
        """
        Sends velocity commands to the underwater robot.
        
        Args:
            vx (float): Linear velocity in x (forward)
            vy (float): Linear velocity in y (strafe)
            vz (float): Linear velocity in z (depth)
            wx (float): Angular velocity in x (roll)
            wy (float): Angular velocity in y (pitch)
            wz (float): Angular velocity in z (yaw)
        """
        vel_msg = BodyVelocityReq()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = 'minigirona/base_link'
        vel_msg.goal.requester = 'path_follower'
        vel_msg.goal.priority = 30
        
        # Set linear velocities
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz
        
        # Set angular velocities
        vel_msg.twist.angular.x = wx
        vel_msg.twist.angular.y = wy
        vel_msg.twist.angular.z = wz
        
        # Enable all axes
        vel_msg.disable_axis.x = False
        vel_msg.disable_axis.y = False
        vel_msg.disable_axis.z = False
        vel_msg.disable_axis.roll = False
        vel_msg.disable_axis.pitch = False
        vel_msg.disable_axis.yaw = False
        
        self.vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        follower = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass