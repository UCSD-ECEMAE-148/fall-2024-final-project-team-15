import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   package_name = 'final_project_pkg'

   ld = LaunchDescription()

   RoboflowOakNode = Node(
           package=package_name,
           executable='person_follower',
           output='screen')

   handDetectionNode = Node(
           package=package_name,
           executable='hand_detection_node',
           output='screen')

   DataUploadNode = Node(
           package=package_name,
           executable='data_uploader',
           output='screen')

   ld.add_action(RoboflowOakNode)
   ld.add_action(handDetectionNode)
   ld.add_action(DataUploadNode)
   return ld
