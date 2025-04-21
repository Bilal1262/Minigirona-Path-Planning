#!/usr/bin/env python

import rospy
from cola2_msgs.msg import BodyVelocityReq
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Header

class VelocityPublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('velocity_publisher', anonymous=True)
        
        # Create publisher for velocity commands
        self.vel_pub = rospy.Publisher('/minigirona/controller/body_velocity_req', 
                                     BodyVelocityReq, queue_size=10)
        
        # Set publishing rate (e.g., 10 Hz)
        self.rate = rospy.Rate(10)
        
        # Prompt user for velocity inputs
        self.velocities = self.get_user_input()

    def get_user_input(self):
        """Prompt user for linear and angular velocities."""
        print("Enter desired velocities for the vehicle:")
        try:
            linear_x = float(input("Linear X velocity (m/s): "))
            linear_y = float(input("Linear Y velocity (m/s): "))
            linear_z = float(input("Linear Z velocity (m/s): "))
            angular_z = float(input("Angular Z velocity (rad/s): "))
        except ValueError:
            rospy.logerr("Invalid input! Using default velocities (0, 0, 0, 0).")
            linear_x = 0.0
            linear_y = 0.0
            linear_z = 0.0
            angular_z = 0.0
        
        return {
            'linear': {
                'x': linear_x,
                'y': linear_y,
                'z': linear_z
            },
            'angular': {
                'x': 0.0,  # Fixed to 0 as per requirement
                'y': 0.0,  # Fixed to 0 as per requirement
                'z': angular_z
            }
        }

    def publish_velocity(self):
        """Publish the velocity command to the topic."""
        while not rospy.is_shutdown():
            # Create BodyVelocityReq message
            vel_msg = BodyVelocityReq()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.header.frame_id = 'minigirona/base_link'
            vel_msg.goal.requester = 'path_follower'
            vel_msg.goal.priority = 30
            
            # Set linear velocities
            vel_msg.twist.linear.x = self.velocities['linear']['x']
            vel_msg.twist.linear.y = self.velocities['linear']['y']
            vel_msg.twist.linear.z = self.velocities['linear']['z']
            
            # Set angular velocities
            vel_msg.twist.angular.x = self.velocities['angular']['x']
            vel_msg.twist.angular.y = self.velocities['angular']['y']
            vel_msg.twist.angular.z = self.velocities['angular']['z']
            
            # Enable all axes
            vel_msg.disable_axis.x = False
            vel_msg.disable_axis.y = False
            vel_msg.disable_axis.z = False
            vel_msg.disable_axis.roll = False
            vel_msg.disable_axis.pitch = False
            vel_msg.disable_axis.yaw = False
            
            # Publish the message
            self.vel_pub.publish(vel_msg)
            rospy.loginfo("Published velocity: linear=({}, {}, {}), angular=({}, {}, {})".format(
                vel_msg.twist.linear.x, vel_msg.twist.linear.y, vel_msg.twist.linear.z,
                vel_msg.twist.angular.x, vel_msg.twist.angular.y, vel_msg.twist.angular.z))
            
            # Sleep to maintain the rate
            self.rate.sleep()

def main():
    try:
        node = VelocityPublisherNode()
        node.publish_velocity()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

if __name__ == '__main__':
    main()