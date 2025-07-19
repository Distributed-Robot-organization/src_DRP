#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    # set initial pose
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)  # No rotation
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()    
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    
    # set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)  # No rotation
    nav.setInitialPose(initial_pose)
    
    #waity for nav2
    nav.waitUntilNav2Active()
    
    #send nav2 goal
    # PI == 3.14 == 180 --> 1.57 == 90 degrees
    goal_pose1  = create_pose_stamped(nav, 5.0, 2.0, 1.57)  # Set your desired goal position and orientation
    goal_pose2  = create_pose_stamped(nav, 8.0, 2.0, 1.57) 
    goal_pose3  = create_pose_stamped(nav, 10.0, 10.0, 1.57) 
    
    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    nav.followWaypoints(waypoints)
    
    while not nav.isTaskComplete():
        feedback = nav.feedback
        # print(feedback)
    
    print(nav.getResult())
    
    
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()