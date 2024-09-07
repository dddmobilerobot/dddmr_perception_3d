/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <perception_3d/depth_camera/depth_camera_layer.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::DepthCameraLayer, perception_3d::Sensor)

namespace perception_3d
{

template<typename T, typename T2>
double getDistanceBTWPoints(T pt, T2 pt2){

  double dx = pt.x-pt2.x;
  double dy = pt.y-pt2.y;
  double dz = pt.z-pt2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

DepthCameraLayer::DepthCameraLayer(){
  return;
}

DepthCameraLayer::~DepthCameraLayer(){
  pct_marking_.reset();
  frustum_utils_.reset();
}

void DepthCameraLayer::onInitialize()
{ 
  
  clock_ = node_->get_clock();
  
  marking_height_ = -1.0;
  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pc_current_window_.reset(new pcl::PointCloud<pcl::PointXYZI>);

  node_->declare_parameter(name_ + ".is_local_planner", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".is_local_planner", is_local_planner_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "is_local_planner: %d", is_local_planner_);

  node_->declare_parameter(name_ + ".xy_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".xy_resolution", resolution_);
  if(resolution_<=0)
    RCLCPP_ERROR(node_->get_logger().get_child(name_), "xy_resolution: %.2f", resolution_);
  else
    RCLCPP_INFO(node_->get_logger().get_child(name_), "xy_resolution: %.2f", resolution_);

  node_->declare_parameter(name_ + ".height_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".height_resolution", height_resolution_);
  if(height_resolution_<=0)
    RCLCPP_ERROR(node_->get_logger().get_child(name_), "height_resolution: %.2f", height_resolution_);
  else
    RCLCPP_INFO(node_->get_logger().get_child(name_), "height_resolution: %.2f", height_resolution_);

  node_->declare_parameter(name_ + ".perception_window_size", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".perception_window_size", perception_window_size_);
  if(perception_window_size_<=0)
    RCLCPP_ERROR(node_->get_logger().get_child(name_), "perception_window_size: %.2f", perception_window_size_);
  else
    RCLCPP_INFO(node_->get_logger().get_child(name_), "perception_window_size: %.2f", perception_window_size_);

  node_->declare_parameter(name_ + ".segmentation_ignore_ratio", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".segmentation_ignore_ratio", segmentation_ignore_ratio_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "segmentation_ignore_ratio: %.2f", segmentation_ignore_ratio_);

  node_->declare_parameter(name_ + ".euclidean_cluster_extraction_tolerance", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".euclidean_cluster_extraction_tolerance", euclidean_cluster_extraction_tolerance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "euclidean_cluster_extraction_tolerance: %.2f", euclidean_cluster_extraction_tolerance_);  

  node_->declare_parameter(name_ + ".euclidean_cluster_extraction_min_cluster_size", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + ".euclidean_cluster_extraction_min_cluster_size", euclidean_cluster_extraction_min_cluster_size_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "euclidean_cluster_extraction_min_cluster_size: %d", euclidean_cluster_extraction_min_cluster_size_);  
  
  node_->declare_parameter(name_ + ".pub_gbl_marking_for_visualization", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".pub_gbl_marking_for_visualization", pub_gbl_marking_for_visualization_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "pub_gbl_marking_for_visualization: %d", pub_gbl_marking_for_visualization_);

  //@ create cluster marking object
  pct_marking_ = std::make_shared<Marking>(&dGraph_, gbl_utils_->getInflationRadius(), shared_data_->kdtree_ground_, resolution_, height_resolution_);
  frustum_utils_ = std::make_shared<FrustumUtils>();

  //@ initial all publishers
  pub_current_observation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_observation", 2);
  pub_current_window_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_window_marking", 2);
  pub_current_projected_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_projected", 2);
  pub_current_segmentation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_segmentation", 2);
  pub_gbl_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/global_marking", 2);
  pub_dGraph_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/dGraph", 2);
  pub_casting_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/tracing_objects", 2);
  pub_frustum_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/frustum", rclcpp::QoS(1));

  marking_pub_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto loop_time = std::chrono::seconds(1);
  marking_pub_timer_ = node_->create_wall_timer(loop_time, std::bind(&DepthCameraLayer::pubUpdateLoop, this), marking_pub_cb_group_);

  //@ loop observation source
  std::string topics_string;
  node_->declare_parameter(name_ + ".observation_sources", rclcpp::ParameterValue(""));
  node_->get_parameter(name_ + ".observation_sources", topics_string);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Subscribed to Topics: %s", topics_string.c_str());
  std::stringstream ss(topics_string);
  
  std::string source;
  while (ss >> source) {
    
    std::string sub_name = name_ + "." + source;
    double observation_persistence, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame;
    bool inf_is_valid, clearing, marking;
    double obstacle_max_range, obstacle_min_range;
    double FOV_W, FOV_V;
    
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Processing: %s", sub_name.c_str());

    node_->declare_parameter(sub_name + ".topic", rclcpp::ParameterValue(source));
    node_->get_parameter(sub_name + ".topic", topic);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "topic: %s", topic.c_str());

    node_->declare_parameter(sub_name + ".sensor_frame", rclcpp::ParameterValue(""));
    node_->get_parameter(sub_name + ".sensor_frame", sensor_frame);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "sensor_frame: %s", sensor_frame.c_str());

    node_->declare_parameter(sub_name + ".observation_persistence", rclcpp::ParameterValue(0.1));
    node_->get_parameter(sub_name + ".observation_persistence", observation_persistence);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "observation_persistence: %.2f", observation_persistence);

    node_->declare_parameter(sub_name + ".expected_update_rate", rclcpp::ParameterValue(0.1));
    node_->get_parameter(sub_name + ".expected_update_rate", expected_update_rate);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "expected_update_rate: %.2f", expected_update_rate);

    node_->declare_parameter(sub_name + ".min_obstacle_height", rclcpp::ParameterValue(0.0));
    node_->get_parameter(sub_name + ".min_obstacle_height", min_obstacle_height);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "min_obstacle_height: %.2f", min_obstacle_height);

    node_->declare_parameter(sub_name + ".max_obstacle_height", rclcpp::ParameterValue(2.0));
    node_->get_parameter(sub_name + ".max_obstacle_height", max_obstacle_height);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "max_obstacle_height: %.2f", max_obstacle_height);

    node_->declare_parameter(sub_name + ".marking", rclcpp::ParameterValue(true));
    node_->get_parameter(sub_name + ".marking", marking);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "marking: %d", marking);

    node_->declare_parameter(sub_name + ".clearing", rclcpp::ParameterValue(true));
    node_->get_parameter(sub_name + ".clearing", clearing);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "clearing: %d", clearing);

    node_->declare_parameter(sub_name + ".obstacle_max_range", rclcpp::ParameterValue(2.5));
    node_->get_parameter(sub_name + ".obstacle_max_range", obstacle_max_range);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "obstacle_max_range: %.2f", obstacle_max_range);

    node_->declare_parameter(sub_name + ".obstacle_min_range", rclcpp::ParameterValue(0.0));
    node_->get_parameter(sub_name + ".obstacle_min_range", obstacle_min_range);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "obstacle_min_range: %.2f", obstacle_min_range);

    node_->declare_parameter(sub_name + ".FOV_W", rclcpp::ParameterValue(2.0));
    node_->get_parameter(sub_name + ".FOV_W", FOV_W);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "FOV_W: %.2f", FOV_W);

    node_->declare_parameter(sub_name + ".FOV_V", rclcpp::ParameterValue(1.0));
    node_->get_parameter(sub_name + ".FOV_V", FOV_V);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "FOV_V: %.2f", FOV_V);
    
    marking_height_ = std::max(marking_height_, max_obstacle_height);
    // create an observation buffer
    
    observation_buffers_[source]=
      std::shared_ptr<perception_3d::DepthCameraObservationBuffer>(
        new DepthCameraObservationBuffer(
          topic,
          gbl_utils_->tf2Buffer(), node_->get_logger().get_child(name_), clock_,
          gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), sensor_frame,
          obstacle_min_range, obstacle_max_range, 
          min_obstacle_height, max_obstacle_height,
          FOV_W, FOV_V,
          expected_update_rate, observation_persistence)
      );
    
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> fcn = std::bind(&DepthCameraLayer::cbSensor, this, std::placeholders::_1, observation_buffers_[source]);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_camera_sub;
    depth_camera_sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(topic, 2, fcn);
    sub_pc_map_[source] = depth_camera_sub;

  }//end of ss
}

void DepthCameraLayer::cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                    const std::shared_ptr<perception_3d::DepthCameraObservationBuffer>& buffer)
{
  buffer->bufferCloud(*msg);
}

void DepthCameraLayer::selfClear(){


  if(is_local_planner_){return;}

  if(!shared_data_->is_static_layer_ready_)
    return;

  if(shared_data_->map_require_update_[name_]){
    //@ need to regenerate dynamic graph
    resetdGraph();
    shared_data_->map_require_update_[name_] = false;
  }

  try
  {
    trans_gbl2b_ = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Failed to get transforms: %s", e.what());
    return;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_last_observation(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  bool observation_clear = false;

  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  aggregatePointCloudFromObservations(pcl_msg_gbl_);

  if(pcl_msg_gbl_->points.size()>5){
    kdtree_last_observation->setInputCloud(pcl_msg_gbl_);
    observation_clear = false;
  }
  else{
    observation_clear = true;
  }
  
  //@ push latest observation buffer into frustum utils, so we get latest frustum for clearing
  frustum_utils_->setObservationBuffers(observation_buffers_);
  pub_frustum_->publish(frustum_utils_->current_frustums_marker_array_);

  visualization_msgs::msg::MarkerArray markerArray;
  pc_current_window_.reset(new pcl::PointCloud<pcl::PointXYZI>);

  //@ We queue all observation here for later clearing and remarking value
  //@ This is very important!!!!!!!!
  std::vector<perception_3d::marking_voxel> current_observation_ptr;
  
  //@ find robot location and base on perception window, we extract all nearby marked clusters
  int round_robot_base_x_min = ((trans_gbl2b_.transform.translation.x-perception_window_size_)/resolution_);
  int round_robot_base_x_max = ((trans_gbl2b_.transform.translation.x+perception_window_size_)/resolution_);
  int round_robot_base_y_min = ((trans_gbl2b_.transform.translation.y-perception_window_size_)/resolution_);
  int round_robot_base_y_max = ((trans_gbl2b_.transform.translation.y+perception_window_size_)/resolution_);

  //@ TODO: make this threshold more adaptive and robust
  int round_robot_base_z_min = ((trans_gbl2b_.transform.translation.z-marking_height_)/height_resolution_);
  int round_robot_base_z_max = ((trans_gbl2b_.transform.translation.z+marking_height_)/height_resolution_);
  //@RCLCPP_INFO(node_->get_logger(), "Search region at x: %d to %d, y: %d to %d, z: %d to %d", 
  //  round_robot_base_x_min, round_robot_base_x_max, round_robot_base_y_min, round_robot_base_y_max, round_robot_base_z_min, round_robot_base_z_max);

  //@Find min/max iterator
  //auto it_x_min = marking_.lower_bound(round_robot_base_x_min);
  //auto it_x_max = marking_.lower_bound(round_robot_base_x_max);
  auto it_x_min = pct_marking_->getXIter(round_robot_base_x_min);
  auto it_x_max = pct_marking_->getXIter(round_robot_base_x_max);
  
  if(it_x_min==pct_marking_->getEnd() && it_x_min==it_x_max)
    return;

  for(auto it_x = it_x_min; it_x!=it_x_max; it_x++){
    auto it_y_min = (*it_x).second.lower_bound(round_robot_base_y_min);
    auto it_y_max = (*it_x).second.lower_bound(round_robot_base_y_max);
    if(it_y_min==(*it_x).second.end() && it_y_min==it_y_max)
      continue;

    for(auto it_y = it_y_min; it_y!=it_y_max; it_y++){
      if((*it_x).second[(*it_y).first].empty())
        continue;
      //@ A marked point exists, loop z for sight check
      auto it_z_min = (*it_x).second[(*it_y).first].lower_bound(round_robot_base_z_min);
      auto it_z_max = (*it_x).second[(*it_y).first].lower_bound(round_robot_base_z_max);
      if(it_z_min==(*it_x).second[(*it_y).first].end() && it_z_min==it_z_max)
        continue;

      //@ fast segmentation of z axis
      for(auto it_z = it_z_min; it_z!=it_z_max;it_z++){

        if((*it_z).second.pc_== nullptr){
          continue;
        }
        //RCLCPP_INFO(node_->get_logger(), "ptr copy number %lu", (*it_z).second.first.use_count());
        pcl::PointXYZI pt;
        pt.x = (*it_x).first*resolution_;
        pt.y = (*it_y).first*resolution_;
        pt.z = (*it_z).first*height_resolution_;
        pcl::PointCloud<pcl::PointXYZI> casting_check;
        //@ use kd-tree search to clear, see: https://github.com/tsengapola/costmap_depth_camera
        //@ In frustums:
        //@             1. kdtree found something near the marking -> dont clear
        //@             2. kdtreenot found obstalce near the marking -> clear it
        if(!frustum_utils_->isinFrustumsObservations(pt)){
          std::vector<int> id(1);
          std::vector<float> sqdist(1);
          if(!observation_clear && kdtree_last_observation->radiusSearch(pt, 0.05, id, sqdist, 1)>0){
            //RCLCPP_INFO(node_->get_logger(), "Obstacles near the marking: %.2f, %.2f, %.2f, skip clearing the marking.", pt.x, pt.y, pt.z);
            perception_3d::marking_voxel a_voxel;
            a_voxel.x = (*it_x).first;
            a_voxel.y = (*it_y).first;
            a_voxel.z = (*it_z).first;
            current_observation_ptr.push_back(a_voxel);
            *pc_current_window_ += (*(*it_z).second.pc_);
            addCastingMarker(pt, current_observation_ptr.size(), markerArray);
            continue;
          }
          else{
            //RCLCPP_INFO(node_->get_logger(), "Remove the marking: %.2f, %.2f, %.2f, skip clearing the marking.", pt.x, pt.y, pt.z);
            pct_marking_->removePCPtr((*it_z).second);
          }
        }
        //@ Not in frustums:
            //@ 1. if attaches frustums -> clear it
            //@ 2. Not attach -> 
                                //@ 2-1. kdtree found something near the marking -> dont clear
                                //@ 2-2. kdtreenot found obstalce near the marking -> clear it
        else{
          if(frustum_utils_->isAttachFRUSTUMs(pt)){
            //@ Check do all points attach, if all of them attach, clear it
            if(observation_clear){
              //@ observation is clear, clear it
              pct_marking_->removePCPtr((*it_z).second);
            }
            else{
              int engage_count = 0;
              //@ compute engagement ratio
              for(auto marking_pt=(*it_z).second.pc_->points.begin(); marking_pt!=(*it_z).second.pc_->points.end(); marking_pt++){
                std::vector<int> id(1);
                std::vector<float> sqdist(1);
                if(kdtree_last_observation->radiusSearch((*marking_pt), 0.01, id, sqdist, 1)>0){
                  engage_count++;
                }
              }
              if(1.0*engage_count/(*it_z).second.pc_->points.size()>0.1){
                //@ 10% of pc collide obstacle, we should not reject
                perception_3d::marking_voxel a_voxel;
                a_voxel.x = (*it_x).first;
                a_voxel.y = (*it_y).first;
                a_voxel.z = (*it_z).first;
                current_observation_ptr.push_back(a_voxel);
                *pc_current_window_ += (*(*it_z).second.pc_);
                addCastingMarker(pt, current_observation_ptr.size(), markerArray);
              }
              else{
                pct_marking_->removePCPtr((*it_z).second);
              }
            }
          }    
          else{
            //@ Not attach, so do regular check
            if(observation_clear){
              //@ no kd tree radius search will be found
              int engage_count = 0;
              pct_marking_->removePCPtr((*it_z).second);         
            }
            else{
              int engage_count = 0;
              //@ compute engagement ratio
              for(auto marking_pt=(*it_z).second.pc_->points.begin(); marking_pt!=(*it_z).second.pc_->points.end(); marking_pt++){
                std::vector<int> id(1);
                std::vector<float> sqdist(1);
                if(kdtree_last_observation->radiusSearch((*marking_pt), 0.01, id, sqdist, 1)>0){
                  engage_count++;
                }
              }
              if(1.0*engage_count/(*it_z).second.pc_->points.size()>0.1){
                //@ 10% of pc collide obstacle, we should not reject
                perception_3d::marking_voxel a_voxel;
                a_voxel.x = (*it_x).first;
                a_voxel.y = (*it_y).first;
                a_voxel.z = (*it_z).first;
                current_observation_ptr.push_back(a_voxel);
                *pc_current_window_ += (*(*it_z).second.pc_);
                addCastingMarker(pt, current_observation_ptr.size(), markerArray);
              }
              else{
                pct_marking_->removePCPtr((*it_z).second);
              }                            
            }
          } 
                 
        }
      }

    }
  }

  //pct_marking_->updateCleared(current_observation_ptr);
  
  if(pub_casting_->get_subscription_count()>0){
    pub_casting_->publish(markerArray);
  }

  if(pub_current_window_marking_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    pc_current_window_->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pc_current_window_, ros_pc2_msg);
    pub_current_window_marking_->publish(ros_pc2_msg);     
  }
}

void DepthCameraLayer::selfMark(){
  
  if(is_local_planner_){return;}

  if(!shared_data_->is_static_layer_ready_)
    return;

  if(!shared_data_->isAllUpdated()){
    return;
  }

  try
  {
    trans_gbl2b_ = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Failed to get transforms: %s", e.what());
    return;
  }

  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  aggregatePointCloudFromObservations(pcl_msg_gbl_);
  
  //@ aggregated observation is in global frame already
  if(pcl_msg_gbl_->points.size()<=5)
    return;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr pc_kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
  pc_kdtree->setInputCloud (pcl_msg_gbl_);

  std::vector<pcl::PointIndices> cluster_indices_segmentation;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_segmentation;
  ec_segmentation.setClusterTolerance (euclidean_cluster_extraction_tolerance_);
  ec_segmentation.setMinClusterSize (euclidean_cluster_extraction_min_cluster_size_);
  ec_segmentation.setMaxClusterSize (pcl_msg_gbl_->points.size());
  ec_segmentation.setSearchMethod (pc_kdtree);
  ec_segmentation.setInputCloud (pcl_msg_gbl_);
  ec_segmentation.extract (cluster_indices_segmentation);

  float intensity_cnt = 100;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_segmentation.begin (); it != cluster_indices_segmentation.end (); ++it)
  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZ centroid;
    pcl::PointXYZ centroid_base_link;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      
      //@For visualization purpose
      pcl::PointXYZI i_pt;
      i_pt.x = pcl_msg_gbl_->points[*pit].x;
      i_pt.y = pcl_msg_gbl_->points[*pit].y;
      i_pt.z = pcl_msg_gbl_->points[*pit].z;
      i_pt.intensity = intensity_cnt;
      centroid.x += i_pt.x;
      centroid.y += i_pt.y;
      centroid.z += i_pt.z;

      cloud_cluster->points.push_back(i_pt); 
      cloud_clusters->points.push_back(i_pt);

    } 
    intensity_cnt += 100;
    centroid.x/=it->indices.size();
    centroid.y/=it->indices.size();
    centroid.z/=it->indices.size();

    //@ Sometimes the lidar accidently add ground scan (due to lego loam did not segment them correctly)
    //@ Therefore we implement following temporal solution -> when cluster attach ground, ignore it!
    std::vector<int> id(1);
    std::vector<float> sqdist(1);
    if(shared_data_->kdtree_ground_->radiusSearch(centroid, 0.1, id, sqdist, 1)){
      continue;
    }

    //@ Test a cluster is in static. If the cluster is in static, we dont need to add it because we can save memory.
    //@ Voxelized marking pc so we can boost the speed when later doing concave clearing check
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_cluster);
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter (*cloud_cluster);
    size_t hit=0;
    id.clear();
    sqdist.clear();
    if(segmentation_ignore_ratio_<=0.999){
      for(auto a_pt=cloud_cluster->points.begin();a_pt!=cloud_cluster->points.end();a_pt++){
        if(shared_data_->kdtree_map_->radiusSearch(centroid, 0.1, id, sqdist, 1)){
          hit++;
          if(hit>cloud_cluster->points.size()*segmentation_ignore_ratio_)
            break;
        }
      }      
    }
    
    if(hit<=cloud_cluster->points.size()*segmentation_ignore_ratio_){

      //@ Project pc base on robot RPY:
      // This is not the perfect solution, because the robot may stand on the ground but the obstalce in on slope
      // Maybe the best approach is to project base on the ground normal
      
      tf2::Quaternion rotation(trans_gbl2b_.transform.rotation.x, trans_gbl2b_.transform.rotation.y, trans_gbl2b_.transform.rotation.z, trans_gbl2b_.transform.rotation.w);
      tf2::Vector3 vector(0, 0, 1);
      tf2::Vector3 base_link_normal = tf2::quatRotate(rotation, vector);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      coefficients->values.resize (4);
      coefficients->values[0] = base_link_normal[0];
      coefficients->values[1] = base_link_normal[1];
      coefficients->values[2] = base_link_normal[2];
      double d = -trans_gbl2b_.transform.translation.x*base_link_normal[0]-trans_gbl2b_.transform.translation.y*base_link_normal[1]-trans_gbl2b_.transform.translation.z*base_link_normal[2];
      coefficients->values[3] = d;
      // Create the filtering object
      
      //pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      //pcl::ProjectInliers<pcl::PointXYZI> proj;
      //proj.setModelType (pcl::SACMODEL_PLANE);
      //proj.setInputCloud (cloud_cluster);
      //proj.setModelCoefficients (coefficients);
      //proj.filter (*projected_cloud_cluster);
      //*projected_cloud_clusters += (*projected_cloud_cluster);
      


      if(frustum_utils_->isinFrustumsObservations(centroid)){
        //@ store the cluster in marking
        //RCLCPP_INFO(node_->get_logger(), "Add marking at: %.2f, %.2f, %.2f", centroid.x, centroid.y, centroid.z);
        pct_marking_->addPCPtr(centroid.x, centroid.y, centroid.z, cloud_cluster, coefficients);
      }
    }
    else{
      RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Reject cluster with size: %lu at %f,%f,%f", cloud_cluster->points.size(), centroid.x, centroid.y, centroid.z);
    }
    
  }

  if(pub_current_projected_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    projected_cloud_clusters->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*projected_cloud_clusters, ros_pc2_msg);
    pub_current_projected_->publish(ros_pc2_msg);
  }

  if(pub_current_segmentation_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    cloud_clusters->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*cloud_clusters, ros_pc2_msg);
    pub_current_segmentation_->publish(ros_pc2_msg);
  }
}

void DepthCameraLayer::aggregatePointCloudFromObservations(const pcl::PointCloud<pcl::PointXYZI>::Ptr& resulting_pcl)
{
  std::vector<perception_3d::DepthCameraObservation> observations;
  for(auto it=observation_buffers_.begin(); it!=observation_buffers_.end();it++)
  {
    (*it).second->getObservations(observations);
  }
  for(auto it=observations.begin(); it!=observations.end();it++){
    *resulting_pcl += (*(*it).cloud_);
  }
}

void DepthCameraLayer::addCastingMarker(const pcl::PointXYZI& pt, size_t id, visualization_msgs::msg::MarkerArray& markerArray){

    //@ Creater marker
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = gbl_utils_->getGblFrame();;
    markerEdge.header.stamp = clock_->now();
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    //markerEdge.lifetime = ros::Duration(2.0);
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.pose.orientation.w = 1.0;
    markerEdge.ns = "edges";
    markerEdge.scale.x = 0.03;
    markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
    markerEdge.color.a = 0.2;
    //@ mark
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;   
    markerEdge.points.push_back(p);
    Eigen::Affine3d trans_gbl2b_af3 = tf2::transformToEigen(trans_gbl2b_);
    p.x = trans_gbl2b_af3.translation().x();
    p.y = trans_gbl2b_af3.translation().y();
    p.z = trans_gbl2b_af3.translation().z();  
    markerEdge.points.push_back(p);
    markerEdge.id = id+1;
    markerArray.markers.push_back(markerEdge);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DepthCameraLayer::getObservation(){
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  aggregatePointCloudFromObservations(sensor_current_observation_);
  return sensor_current_observation_;
}

void DepthCameraLayer::resetdGraph(){

  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s starts to reset dynamic graph.", name_.c_str());
  dGraph_.clear();
  dGraph_.initial(shared_data_->static_ground_size_, gbl_utils_->getMaxObstacleDistance());
  pct_marking_ = std::make_shared<Marking>(&dGraph_, gbl_utils_->getInflationRadius(), shared_data_->kdtree_ground_, resolution_, height_resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s done dynamic graph regeneration.", name_.c_str());
}

double DepthCameraLayer::get_dGraphValue(const unsigned int index){
  return pct_marking_->get_dGraphValue(index);
}

bool DepthCameraLayer::isCurrent(){

  bool current = true;
  for(auto i=observation_buffers_.begin(); i!=observation_buffers_.end();i++){
    current = current && (*i).second->isCurrent();
  }
  return current;
}


void DepthCameraLayer::pubUpdateLoop()
{

  if(shared_data_->map_require_update_[name_]){
    return;
  }

  if(pub_gbl_marking_for_visualization_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg (new pcl::PointCloud<pcl::PointXYZI>);
    //for(auto itx=marking_.begin();itx!=marking_.end();itx++){
    for(auto itx=pct_marking_->getBegin();itx!=pct_marking_->getEnd();itx++){
      for(auto ity=(*itx).second.begin();ity!=(*itx).second.end();ity++){
        if((*itx).second[(*ity).first].empty())
          return;
        for(auto itz=(*itx).second[(*ity).first].begin();itz!=(*itx).second[(*ity).first].end();itz++){
          if((*itz).second.pc_== nullptr){
            continue;
          }
          pcl::PointXYZ pt;
          pt.x = (*itx).first*resolution_;
          pt.y = (*ity).first*resolution_;
          pt.z = (*itz).first*height_resolution_;   
          if((*itz).second.pc_->points.size()>1)
            *pcl_msg += (*(*itz).second.pc_);  
        }
      }
    }
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    pcl_msg->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pcl_msg, ros_pc2_msg);
    pub_gbl_marking_->publish(ros_pc2_msg);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg2 (new pcl::PointCloud<pcl::PointXYZI>);
  for(size_t index=0;index<shared_data_->static_ground_size_;index++){
    pcl::PointXYZI ipt;
    ipt.x = shared_data_->pcl_ground_->points[index].x;
    ipt.y = shared_data_->pcl_ground_->points[index].y;
    ipt.z = shared_data_->pcl_ground_->points[index].z;   
    ipt.intensity = pct_marking_->get_dGraphValue(index);
    pcl_msg2->push_back(ipt);
  }
  sensor_msgs::msg::PointCloud2 ros_pc2_msg2;
  pcl_msg2->header.frame_id = gbl_utils_->getGblFrame();
  pcl::toROSMsg(*pcl_msg2, ros_pc2_msg2);
  pub_dGraph_->publish(ros_pc2_msg2);
  
}

}