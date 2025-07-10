#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations



def main():
    rclpy.init()
    nav = BasicNavigator()
    
    # set initial pose
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)  # No rotation
    
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0   
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    # nav.setInitialPose(initial_pose)
    
    #waity for nav2
    nav.waitUntilNav2Active()
    
    #send nav2 goal
    # PI == 3.14 == 180 --> 1.57 == 90 degrees
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57) 
    goal_pose = PoseStamped()
    nav.goToPose(goal_pose)
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 5.0  # Set your desired goal position
    goal_pose.pose.position.y = 2.0  # Set your desired goal position
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    nav.goToPose(goal_pose)
    
    
    
    while not nav.isTaskComplete():
        feedback = nav.feedback() #give current pose of the robot
        print(feedback)
    # node = rclpy.create_node('test_api_nav2')
    # node.get_logger().info('Node initialized successfully.')
    # rclpy.spin(node)
    
    print(nav.getResult())
    
    
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()