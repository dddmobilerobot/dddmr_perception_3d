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

  b2l1 = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.465", "0.145", "0.235", "0.0", "0.0", "1.72", "base_link", "left1"]
      )

  l12l2 = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.0", "0.0", "0.0", "0.0", "-0.48", "0.0", "left1", "left2"]
      )

  l22left = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.0", "0.0", "0.0", "0.48", "0.0", "0.0", "left2", "left_link"]
      )

  camleft2opt = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.0", "0.0", "0.0", "-1.571", "-0.000", "-1.571", "left_link", "left_depth_optical_frame"]
      )

  b2r1 = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.465", "-0.145", "0.235", "0.0", "0.0", "1.42", "base_link", "right1"]
      )

  r12r2 = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.0", "0.0", "0.0", "0.0", "0.48", "0.0", "right1", "right2"]
      )

  r22right = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.0", "0.0", "0.0", "0.48", "0.0", "0.0", "right2", "right_link"]
      )

  camright2opt = Node(
          package="tf2_ros",
          executable="static_transform_publisher",
          output="screen" ,
          arguments=["0.0", "0.0", "0.0", "-1.571", "-0.000", "-1.571", "right_link", "right_depth_optical_frame"]
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
  
  #--------depth image to pointcloud
  depth2pc_left = Node(
          package="perception_3d",
          executable="depthimg2pointcloud_node",
          name="depthimg2pointcloud_left",
          output="screen",
          parameters=[
              {"topic": '/realsense/left/depth/image_rect_raw'},
              {"info": '/realsense/left/depth/camera_info'},
              {"max_distance": 6.0},
              {"sample_step": 4},
              {"leaf_size": 0.05}
          ]
  )  

  depth2pc_right = Node(
          package="perception_3d",
          executable="depthimg2pointcloud_node",
          name="depthimg2pointcloud_right",
          output="screen",
          parameters=[
              {"topic": '/realsense/right/depth/image_rect_raw'},
              {"info": '/realsense/right/depth/camera_info'},
              {"max_distance": 6.0},
              {"sample_step": 4},
              {"leaf_size": 0.05}
          ]
  )  

  perception_3d_yaml = os.path.join(
      get_package_share_directory('perception_3d'),
      'config', 'depth_camera_demo',
      'multi_depth_camera_3d_ros.yaml'
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
          arguments=['-d', os.path.join(get_package_share_directory('perception_3d'), 'rviz', 'multi_depth_camera_3d.rviz')]
  )  
  
  bag_player = ExecuteProcess(
      cmd=[
          "ros2",
          "bag",
          "play",
          "--loop",
          "/root/dddmr_bags/multi_depth_camera_images",
      ],
      output="screen",
  )

  ld = LaunchDescription()

  ld.add_action(map2baselink)

  ld.add_action(b2l1)
  ld.add_action(l12l2)
  ld.add_action(l22left)
  ld.add_action(camleft2opt)

  ld.add_action(b2r1)
  ld.add_action(r12r2)
  ld.add_action(r22right)
  ld.add_action(camright2opt)

  ld.add_action(pcl_publisher)
  ld.add_action(depth2pc_left)
  ld.add_action(depth2pc_right)
  ld.add_action(perception_3d_ros)
  ld.add_action(rviz)
  
  ld.add_action(TimerAction(period=3.0, actions=[bag_player]))

  return ld