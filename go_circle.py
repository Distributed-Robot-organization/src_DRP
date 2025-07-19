#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math

def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
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

def generate_circular_waypoints(navigator, center_x, center_y, radius, num_points=36):
    waypoints = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        # Orient the robot to follow the circle (tangent)
        yaw = angle + math.pi / 2  # rotate 90° for tangent direction
        pose = create_pose_stamped(navigator, x, y, yaw)
        waypoints.append(pose)
    return waypoints

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Set initial pose (can be adjusted to current robot position)
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)  # start near the circle
    nav.setInitialPose(initial_pose)

    # Wait for Nav2 to become active
    nav.waitUntilNav2Active()

    # Generate circular waypoints around (9.0, 9.0)
    radius = 2.0  # You can change this
    waypoints = generate_circular_waypoints(nav, center_x=9.0, center_y=9.0, radius=radius, num_points=36)

    # Send waypoints to follow
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.feedback
        # Optional: print progress
        # print(feedback)

    print(nav.getResult())
    rclpy.shutdown()

if __name__ == "__main__":
    main()


### un altro modo è dare i comandi diretti al robot: 
# # Moto circolare usando cmd_vel
# from geometry_msgs.msg import Twist
# import rclpy
# import math

# def main():
#     rclpy.init()
#     node = rclpy.create_node('circular_motion_controller')
#     pub = node.create_publisher(Twist, '/cmd_vel', 10)
#     rate = node.create_rate(10)

#     twist = Twist()
#     linear_speed = 0.2     # m/s
#     angular_speed = linear_speed / 2.0  # raggio = 2 m

#     twist.linear.x = linear_speed
#     twist.angular.z = angular_speed

#     t_start = node.get_clock().now().seconds_nanoseconds()[0]
#     duration = int(2 * math.pi / angular_speed)  # per fare un giro completo

#     while rclpy.ok():
#         t_now = node.get_clock().now().seconds_nanoseconds()[0]
#         if t_now - t_start > duration:
#             break
#         pub.publish(twist)
#         rate.sleep()

#     # Ferma il robot
#     pub.publish(Twist())
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
