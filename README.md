# dddmr_perception_3d
Perception 3D is graph-based framework allowing user to develope applications for mobile robots, such as path planning, marking/clearing obstacles, creating no-enter/speed limit layer.

<table>
  <tr width="100%">
    <td width="33%"><img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/perception_3d/perception_3d_global_plan.gif"/>Global planning in 3D map</td>
    <td width="33%"><img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/perception_3d/marking_tracking_clearing.gif"/>Marking/Tracking/Clearing</td>
    <td width="33%"><img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/perception_3d/speed_limit_zone.png"/>Speed-limit/no-enter zone</td>
  </tr>
</table> 

Perception 3D:
- Sensor support:
  - [x] Multilayer spinning lidar (Velodyne/Ouster/Leishen)
  - [ ] Depth camera (Realsense/oak)
  - [ ] Scanning Lidar (Livox mid-360/Unitree 4D LiDAR L1)
- Zone feature support:
  - [x] Static layer
  - [x] Speed limit layer
  - [ ] No enter layer
