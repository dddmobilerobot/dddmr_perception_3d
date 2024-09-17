import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
  map2baselink = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["1.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "base_link"]
      )

  b2l = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.2", "0.0", "0.6", "0.0", "0.0", "0.0", "base_link", "unilidar_lidar"]
      )


  #--------provide map/ground to static layer
  pcl_publisher = Node(
          package="mcl_3dl",
          executable="pcl_publisher",
          output="screen",
          parameters=[
              {"global_frame": 'map'},
              {"map_rotate_around_x": 1.570796327},
              {"ground_rotate_around_x": 1.570796327},
              {"map_down_sample": 0.5},
              {"ground_down_sample": 0.5},
              {"map_dir": '/root/dddmr_navigation/src/dddmr_perception_3d/map/map.pcd'},
              {"ground_dir": '/root/dddmr_navigation/src/dddmr_perception_3d/map/ground.pcd'},
          ]
  )  
  

  perception_3d_yaml = os.path.join(
      get_package_share_directory('perception_3d'),
      'config', 'multilayer_spinning_lidar_demo',
      'scanning_lidar_3d_ros.yaml'
      )

  perception_3d_ros = Node(
          package="perception_3d",
          executable="perception_3d_ros_node",
          output="screen",
          parameters = [perception_3d_yaml]
  )  

  rviz = Node(
          package="rviz2",
          executable="rviz2",
          output="screen",
          arguments=['-d', os.path.join(get_package_share_directory('perception_3d'), 'rviz', 'scanning_lidar_3d.rviz')]
  )  
  
  bag_player = ExecuteProcess(
      cmd=[
          "ros2",
          "bag",
          "play",
          "--loop",
          "/root/dddmr_bags/unitree_lidar_point_cloud",
      ],
      output="screen",
  )

  ld = LaunchDescription()

  ld.add_action(map2baselink)
  ld.add_action(b2l)

  ld.add_action(pcl_publisher)
  ld.add_action(perception_3d_ros)
  ld.add_action(rviz)
  
  ld.add_action(TimerAction(period=3.0, actions=[bag_player]))

  return ld