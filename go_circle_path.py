#!/usr/bin/env python3
"""
Esegue un giro completo intorno al punto (9, 9) seguendo un Path continuo.

☑ Requisiti:
    - ROS 2 Humble (o più recente) con Nav2 avviato
    - pacchetto «nav2_simple_commander»
    - topic «/map» come frame globale
"""

import math
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf_transformations


def pose_stamped(clock, x, y, yaw, frame_id='map') -> PoseStamped:
    """Crea un PoseStamped con quaternione da yaw."""
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    pose = PoseStamped()
    pose.header.stamp = clock.now().to_msg()
    pose.header.frame_id = frame_id

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def circular_path(navigator: BasicNavigator,
                  center_x: float, center_y: float,
                  radius: float = 2.0,
                  num_points: int = 180,
                  tangent: bool = True) -> Path:
    """
    Genera un Path circolare chiuso.

    • `num_points` controlla la densità dei campioni (più alto → traiettoria più liscia).  
    • `tangent=True` ⇒ robot orientato tangente alla traiettoria;  
      `tangent=False` ⇒ robot rivolto verso il centro.
    """
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = navigator.get_clock().now().to_msg()

    for i in range(num_points + 1):      # +1 per chiudere perfettamente il cerchio
        theta = 2.0 * math.pi * i / num_points
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)

        if tangent:
            yaw = theta + math.pi / 2.0       # orientazione tangente (antioraria)
        else:
            yaw = math.atan2(center_y - y, center_x - x)  # verso il centro

        path.poses.append(pose_stamped(navigator.get_clock(), x, y, yaw))

    return path
def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 1. Posizione iniziale
    # initial = pose_stamped(navigator.get_clock(), 0.0, 0.0, 0.0)
    # navigator.setInitialPose(initial)
    navigator.waitUntilNav2Active()

    # 2. Spostamento iniziale verso il centro del cerchio
    entry_point = pose_stamped(navigator.get_clock(), 1.0, 0.0, 0.0)
    navigator.waitUntilNav2Active()
    navigator.goThroughPoses([entry_point])
    while not navigator.isTaskComplete():
        pass

    # 3. Genera path circolare
    raw_path = circular_path(navigator, center_x=2.0, center_y=2.0, radius=2.0, num_points=180, tangent=True)

    # 4. Testa smoothing
    print(f"Raw path has {len(raw_path.poses)} poses")
    try:
        smooth_path = navigator.smoothPath(raw_path)
        print(f"Smoothed path has {len(smooth_path.poses)} poses")
    except Exception as e:
        print(f"Smoothing failed: {e}")
        smooth_path = raw_path  # fallback

    # 5. Navigazione
    navigator.followPath(smooth_path)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # print(f"{feedback.percent_complete:.1f}% completato")

    result = navigator.getResult()
    print(f"Risultato: {result}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
