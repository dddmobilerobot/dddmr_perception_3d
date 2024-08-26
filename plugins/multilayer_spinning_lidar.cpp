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
#include <perception_3d/multilayer_spinning_lidar.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::MultiLayerSpinningLidar, perception_3d::Sensor)

namespace perception_3d
{


template<typename T, typename T2>
double getDistanceBTWPoints(T pt, T2 pt2){

  double dx = pt.x-pt2.x;
  double dy = pt.y-pt2.y;
  double dz = pt.z-pt2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

MultiLayerSpinningLidar::MultiLayerSpinningLidar(){
  return;
}

MultiLayerSpinningLidar::~MultiLayerSpinningLidar(){
}

void MultiLayerSpinningLidar::ptrInitial(){
  pcl_msg_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pc_current_window_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void MultiLayerSpinningLidar::onInitialize()
{ 

  ptrInitial();

  node_->declare_parameter(name_ + ".topic", rclcpp::ParameterValue(""));
  node_->get_parameter(name_ + ".topic", topic_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "topic: %s", topic_.c_str());

  node_->declare_parameter(name_ + ".vertical_FOV_top", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".vertical_FOV_top", vertical_FOV_top_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "vertical_FOV_top: %.1f", vertical_FOV_top_);

  node_->declare_parameter(name_ + ".vertical_FOV_bottom", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".vertical_FOV_bottom", vertical_FOV_bottom_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "vertical_FOV_bottom: %.1f", vertical_FOV_bottom_);

  node_->declare_parameter(name_ + ".scan_effective_positive_start", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_positive_start", scan_effective_positive_start_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_positive_start: %.1f", scan_effective_positive_start_);

  node_->declare_parameter(name_ + ".scan_effective_positive_end", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_positive_end", scan_effective_positive_end_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_positive_end: %.1f", scan_effective_positive_end_);

  node_->declare_parameter(name_ + ".scan_effective_negative_start", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_negative_start", scan_effective_negative_start_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_negative_start: %.1f", scan_effective_negative_start_);

  node_->declare_parameter(name_ + ".scan_effective_negative_end", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_negative_end", scan_effective_negative_end_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_negative_end: %.1f", scan_effective_negative_end_);

  node_->declare_parameter(name_ + ".euclidean_cluster_extraction_tolerance", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".euclidean_cluster_extraction_tolerance", euclidean_cluster_extraction_tolerance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "euclidean_cluster_extraction_tolerance: %.1f", euclidean_cluster_extraction_tolerance_);

  node_->declare_parameter(name_ + ".euclidean_cluster_extraction_min_cluster_size", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + ".euclidean_cluster_extraction_min_cluster_size", euclidean_cluster_extraction_min_cluster_size_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "euclidean_cluster_extraction_min_cluster_size: %d", euclidean_cluster_extraction_min_cluster_size_);

  node_->declare_parameter(name_ + ".euclidean_cluster_extraction_max_cluster_size", rclcpp::ParameterValue(100));
  node_->get_parameter(name_ + ".euclidean_cluster_extraction_max_cluster_size", euclidean_cluster_extraction_max_cluster_size_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "euclidean_cluster_extraction_max_cluster_size: %d", euclidean_cluster_extraction_max_cluster_size_);

  clock_ = node_->get_clock();
  last_observation_time_ = clock_->now();

  node_->declare_parameter(name_ + ".xy_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".xy_resolution", resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "xy_resolution: %.2f", resolution_);

  node_->declare_parameter(name_ + ".height_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".height_resolution", height_resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "height_resolution: %.2f", height_resolution_);

  node_->declare_parameter(name_ + ".marking_height", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".marking_height", marking_height_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "marking_height: %.2f", marking_height_);

  node_->declare_parameter(name_ + ".perception_window_size", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".perception_window_size", perception_window_size_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "perception_window_size: %.2f", perception_window_size_);

  node_->declare_parameter(name_ + ".segmentation_ignore_ratio", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".segmentation_ignore_ratio", segmentation_ignore_ratio_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "segmentation_ignore_ratio: %.2f", segmentation_ignore_ratio_);

  node_->declare_parameter(name_ + ".is_local_planner", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".is_local_planner", is_local_planner_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "is_local_planner: %d", is_local_planner_);

  node_->declare_parameter(name_ + ".expected_sensor_time", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".expected_sensor_time", expected_sensor_time_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "expected_sensor_time: %.2f", expected_sensor_time_);

  node_->declare_parameter(name_ + ".pub_gbl_marking_for_visualization", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".pub_gbl_marking_for_visualization", pub_gbl_marking_for_visualization_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "pub_gbl_marking_for_visualization: %d", pub_gbl_marking_for_visualization_);

  sensor_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sensor_cb_group_;

  sensor_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, 2, 
    std::bind(&MultiLayerSpinningLidar::cbSensor, this, std::placeholders::_1), sub_options);
  
  pub_current_observation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_observation", 2);
  pub_current_window_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_window_marking", 2);
  pub_current_projected_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_projected", 2);
  pub_current_segmentation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_segmentation", 2);
  pub_gbl_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/global_marking", 2);
  pub_dGraph_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/dGraph", 2);

  pub_casting_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/tracing_objects", 2);

  pct_marking_ = std::make_shared<Marking>(&dGraph_, gbl_utils_->getInflationRadius(), shared_data_->kdtree_ground_, resolution_, height_resolution_);
  get_first_tf_ = false;
  
  marking_pub_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto loop_time = std::chrono::seconds(1);
  marking_pub_timer_ = node_->create_wall_timer(loop_time, std::bind(&MultiLayerSpinningLidar::pubUpdateLoop, this), marking_pub_cb_group_);

}

void MultiLayerSpinningLidar::cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  //@transform to point cloud library format first so we can leverage PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *pcl_msg);

  //@Create two trans, baselink->sensor and map->baselink

  try
  {
    trans_b2s_ = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getRobotFrame(), msg->header.frame_id, tf2::TimePointZero);

    trans_gbl2b_ = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), tf2::TimePointZero);

  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Failed to get transforms: %s", e.what());
    return;
  }

  get_first_tf_ = true;
  
  RCLCPP_INFO_THROTTLE(node_->get_logger().get_child(name_), *clock_, 60000, "Receiving Lidar topic: %s", topic_.c_str());

  //@Justify affine 3d
  //Eigen::Affine3d a = tf2::transformToEigen(trans_gbl2b_);
  //Eigen::Affine3d b = tf2::transformToEigen(trans_b2s_);
  //Eigen::Affine3d c = tf2::transformToEigen(trans_gbl2s_);
  //Eigen::Affine3d d = a*b;

  //RCLCPP_INFO(node_->get_logger().get_child(name_), "trans: %f,%f,%f ---> %f,%f,%f", c.translation().x(),c.translation().y(),c.translation().z(), d.translation().x(),d.translation().y(),d.translation().z());
  //RCLCPP_INFO_STREAM(node_->get_logger().get_child(name_), "Rotation c: " << c.rotation());
  //RCLCPP_INFO_STREAM(node_->get_logger().get_child(name_), "Rotation d: " << d.rotation());
  
  Eigen::Affine3d trans_b2s_af3 = tf2::transformToEigen(trans_b2s_);
  pcl::transformPointCloud(*pcl_msg, *pcl_msg, trans_b2s_af3);

  //@Get affine tf from gbl to sensor
  Eigen::Affine3d trans_gbl2b_af3 = tf2::transformToEigen(trans_gbl2b_);
  trans_gbl2s_af3_ = trans_gbl2b_af3*trans_b2s_af3;
  trans_gbl2s_ = tf2::eigenToTransform (trans_gbl2s_af3_);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pcl_msg);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-perception_window_size_, perception_window_size_);
  pass.filter (*pcl_msg);
  pass.setInputCloud (pcl_msg);
  pass.setFilterFieldName ("y");
  pass.filter (*pcl_msg);
  pass.setInputCloud (pcl_msg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, marking_height_);
  pass.filter (*pcl_msg);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (pcl_msg);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*pcl_msg);

  //@Protect Mark/Clear functions
  std::unique_lock<std::recursive_mutex> lock(marking_mutex_);

  pcl_msg_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_msg_ = pcl_msg;

  if(is_local_planner_){
    //@ put to current observation, different for global/local
    Eigen::Affine3d trans_gbl2b_af3 = tf2::transformToEigen(trans_gbl2b_);
    pcl::transformPointCloud(*pcl_msg_, *pcl_msg_, trans_gbl2b_af3);
    pcl::copyPointCloud(*pcl_msg_, *sensor_current_observation_);
  }

  //@ update time
  last_observation_time_ = clock_->now();

  if(pub_current_observation_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    pcl_msg_->header.frame_id = gbl_utils_->getRobotFrame();
    pcl::toROSMsg(*pcl_msg_, ros_pc2_msg);
    pub_current_observation_->publish(ros_pc2_msg);
  }

}

void MultiLayerSpinningLidar::selfMark(){
  
  if(is_local_planner_){return;}

  if(!get_first_tf_ || !shared_data_->is_static_layer_ready_)
    return;

  if(!shared_data_->isAllUpdated()){
    return;
  }
  
  std::unique_lock<std::recursive_mutex> lock(marking_mutex_);

  if(pcl_msg_->points.size()<=5)
    return;
  //@ Transform into global frame
  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3d trans_gbl2b_af3 = tf2::transformToEigen(trans_gbl2b_);
  pcl::transformPointCloud(*pcl_msg_, *pcl_msg_gbl_, trans_gbl2b_af3);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr pc_kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  pc_kdtree->setInputCloud (pcl_msg_gbl_);

  std::vector<pcl::PointIndices> cluster_indices_segmentation;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_segmentation;
  ec_segmentation.setClusterTolerance (euclidean_cluster_extraction_tolerance_);
  ec_segmentation.setMinClusterSize (euclidean_cluster_extraction_min_cluster_size_);
  ec_segmentation.setMaxClusterSize (euclidean_cluster_extraction_max_cluster_size_);
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
      

      //@ store the cluster in marking
      pct_marking_->addPCPtr(centroid.x, centroid.y, centroid.z, cloud_cluster, coefficients);


    }
    else{
      RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Reject cluster with size: %lu at %f,%f,%f, because it is located in the static layer", cloud_cluster->points.size(), centroid.x, centroid.y, centroid.z);
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

void MultiLayerSpinningLidar::selfClear(){
  
  if(is_local_planner_){return;}

  if(! get_first_tf_ || ! shared_data_->is_static_layer_ready_)
    return;

  if(shared_data_->map_require_update_[name_]){
    //@ need to regenerate dynamic graph
    resetdGraph();
    shared_data_->map_require_update_[name_] = false;
  }

  std::unique_lock<std::recursive_mutex> lock(marking_mutex_);

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_last_observation(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  bool observation_clear = false;

  if(pcl_msg_gbl_->points.size()>5){
    kdtree_last_observation->setInputCloud(pcl_msg_gbl_);
    observation_clear = false;
  }
  else{
    observation_clear = true;
  }
  
  visualization_msgs::msg::MarkerArray markerArray;
  pc_current_window_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  //@ We queue all observation here for later clearing and remarking value
  //@ This is very important!!!!!!!!
  std::vector<perception_3d::marking_voxel> current_observation_ptr;

  int round_robot_base_x_min = ((trans_gbl2b_.transform.translation.x-perception_window_size_)/resolution_);
  int round_robot_base_x_max = ((trans_gbl2b_.transform.translation.x+perception_window_size_)/resolution_);
  int round_robot_base_y_min = ((trans_gbl2b_.transform.translation.y-perception_window_size_)/resolution_);
  int round_robot_base_y_max = ((trans_gbl2b_.transform.translation.y+perception_window_size_)/resolution_);

  //@ TODO: make this threshold more adaptive and robust
  int round_robot_base_z_min = ((trans_gbl2b_.transform.translation.z-marking_height_)/height_resolution_);
  int round_robot_base_z_max = ((trans_gbl2b_.transform.translation.z+marking_height_)/height_resolution_);


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
        pcl::PointXYZ pt;
        pt.x = (*it_x).first*resolution_;
        pt.y = (*it_y).first*resolution_;
        pt.z = (*it_z).first*height_resolution_;
        pcl::PointCloud<pcl::PointXYZI> casting_check;

        if(!isinLidarObservation(pt)){
  
          perception_3d::marking_voxel a_voxel;
          a_voxel.x = (*it_x).first;
          a_voxel.y = (*it_y).first;
          a_voxel.z = (*it_z).first;
          current_observation_ptr.push_back(a_voxel);
          *pc_current_window_ += (*(*it_z).second.first);
          addCastingMarker(pt, current_observation_ptr.size(), markerArray);
          continue;
        }
        else{
          //@ get point cloud along the ray for casting
          bool skip_clearing = false;
          if(!observation_clear){
            getCastingPointCloud(pt, casting_check);
            for(auto a_pt=casting_check.points.begin(); a_pt!=casting_check.points.end(); a_pt++){

              //@ Create a point for kd-tree
              pcl::PointXYZ pt_i;
              pt_i.x = (*a_pt).x;
              pt_i.y = (*a_pt).y;
              pt_i.z = (*a_pt).z;
              double search_distance = (*a_pt).intensity/3. + 0.1; //@ decrease spot size
              std::vector<int> id;
              std::vector<float> sqdist;
              if(kdtree_last_observation->radiusSearch(pt_i, search_distance, id, sqdist)>0){
                skip_clearing = true;
                break;
              }
            }            
          }

          //Hit obstacle when ray tracing, so we skip clearing->meaning that we add this frame to observation
          if(skip_clearing){
            perception_3d::marking_voxel a_voxel;
            a_voxel.x = (*it_x).first;
            a_voxel.y = (*it_y).first;
            a_voxel.z = (*it_z).first;
            current_observation_ptr.push_back(a_voxel);
            *pc_current_window_ += (*(*it_z).second.first);
            addCastingMarker(pt, current_observation_ptr.size(), markerArray);
            continue;
          }
          
          
          std::vector<int> id;
          std::vector<float> sqdist;
          //@ I am not sure what happen below, looks like I redo check again but the threshold (1) is different
          if(kdtree_last_observation->radiusSearch(pt, 0.2, id, sqdist)>1){
            *pc_current_window_ += (*(*it_z).second.first);

            perception_3d::marking_voxel a_voxel;
            a_voxel.x = (*it_x).first;
            a_voxel.y = (*it_y).first;
            a_voxel.z = (*it_z).first;
            current_observation_ptr.push_back(a_voxel);
            addCastingMarker(pt, current_observation_ptr.size(), markerArray);
          }       
          else{
            pct_marking_->removePCPtr((*it_z).second.first);
          } 
                 
        }
      }

    }
  }

  pct_marking_->updateCleared(current_observation_ptr);
  //@ put to current observation, different for global/local
  sensor_current_observation_ = pc_current_window_;
  
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

void MultiLayerSpinningLidar::getCastingPointCloud(pcl::PointXYZ& cluster_center, pcl::PointCloud<pcl::PointXYZI>& pc_for_check){

  
  //@We leverage intensity as distance from cluster_center to check point
  
  
  double l = cluster_center.x - trans_gbl2s_af3_.translation().x();
  double m = cluster_center.y - trans_gbl2s_af3_.translation().y();
  double n = cluster_center.z - trans_gbl2s_af3_.translation().z();


  if(n>=0){
    for(auto step_z=cluster_center.z;step_z>=trans_gbl2s_af3_.translation().z();step_z=step_z-0.05){
      pcl::PointXYZI pt;
      pt.z = step_z;
      pt.y = (pt.z- cluster_center.z)/n * m + cluster_center.y;
      pt.x = (pt.z- cluster_center.z)/n * l + cluster_center.x; 
      pt.intensity = getDistanceBTWPoints(cluster_center, pt);  
      pc_for_check.push_back(pt); 
    }
  }
  else{
    for(auto step_z=cluster_center.z;step_z<=trans_gbl2s_af3_.translation().z();step_z=step_z+0.05){
      pcl::PointXYZI pt;
      pt.z = step_z;
      pt.y = (pt.z- cluster_center.z)/n * m + cluster_center.y;
      pt.x = (pt.z- cluster_center.z)/n * l + cluster_center.x;   
      pt.intensity = getDistanceBTWPoints(cluster_center, pt);  
      pc_for_check.push_back(pt); 
    }    
  }

}

bool MultiLayerSpinningLidar::isinLidarObservation(pcl::PointXYZ& pc){

  
  //@ Solving vertical distance to sensor plan, so we can compute sin for vertical FOV check
  
  tf2::Quaternion rotation(trans_gbl2s_.transform.rotation.x, trans_gbl2s_.transform.rotation.y, trans_gbl2s_.transform.rotation.z, trans_gbl2s_.transform.rotation.w);
  tf2::Vector3 vector(0, 0, 1);
  tf2::Vector3 sensor_normal = tf2::quatRotate(rotation, vector);

  double d = -trans_gbl2s_.transform.translation.x*sensor_normal[0]-trans_gbl2s_.transform.translation.y*sensor_normal[1]-trans_gbl2s_.transform.translation.z*sensor_normal[2];
  double p2plane = pc.x*sensor_normal[0]+pc.y*sensor_normal[1]+pc.z*sensor_normal[2]+d;
  double dx = pc.x-trans_gbl2s_.transform.translation.x;
  double dy = pc.y-trans_gbl2s_.transform.translation.y; 
  double dz = pc.z-trans_gbl2s_.transform.translation.z;
  double p2s = sqrt(dx*dx+dy*dy+dz*dz);
  double result = asin (p2plane/p2s) * 180.0 / 3.1415926535;
  if(result<vertical_FOV_bottom_ || result>vertical_FOV_top_)
    return false;
  
  
  //@ Leverage shortest angle to rule out yaw angle
  

  
  // Generate a pose pointing from sen sensor to cloud centroid
  // Remember, here we are all in global frame
  
  double vx,vy,vz;
  vx = pc.x - trans_gbl2s_.transform.translation.x;
  vy = pc.y - trans_gbl2s_.transform.translation.y;
  vz = pc.z - trans_gbl2s_.transform.translation.z;
  double unit = sqrt(vx*vx + vy*vy + vz*vz);
  
  tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

  tf2::Vector3 up_vector(1.0, 0.0, 0.0);
  tf2::Vector3 right_vector = axis_vector.cross(up_vector);
  right_vector.normalized();
  tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
  q.normalize();
  
  //@ We generate vector from sensor to cluster, and set origin by sensor translation, so we get sensor->sensor_pointing_centroid 
  tf2::Transform tf2_gbl2sensor_pointing_clustercentroid;
  tf2_gbl2sensor_pointing_clustercentroid.setRotation(q);
  tf2_gbl2sensor_pointing_clustercentroid.setOrigin(tf2::Vector3(trans_gbl2s_.transform.translation.x, trans_gbl2s_.transform.translation.y, trans_gbl2s_.transform.translation.z));
   
  //@Transform trans_gbl2s_ to tf2 -> Get sensor to global, so that we later can get base_link2gbl * gbl2lastpose
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2s;
  tf2::fromMsg(trans_gbl2s_, tf2_trans_gbl2s);
  auto tf2_trans_gbl2s_inverse = tf2_trans_gbl2s.inverse();
  //@ Get sensor to cluster centroid
  tf2::Transform tf2_sensor2sensor_pointing_clustercentroid;
  tf2_sensor2sensor_pointing_clustercentroid.mult(tf2_trans_gbl2s_inverse, tf2_gbl2sensor_pointing_clustercentroid);
  //@Get RPY
  tf2::Matrix3x3 m(tf2_sensor2sensor_pointing_clustercentroid.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //Although the test shows that yaw is already the shortest, we will use shortest_angular_distance anyway.
  yaw = angles::shortest_angular_distance(0.0, yaw);
  yaw = yaw * 180.0 / 3.1415926535;// change to degree
  if(yaw>=0 && (yaw<scan_effective_positive_start_ || yaw>scan_effective_positive_end_))
    return false;
  else if(yaw<0 && (yaw>scan_effective_negative_start_ || yaw<scan_effective_negative_end_))
    return false;
  else
    return true;

}

void MultiLayerSpinningLidar::pubUpdateLoop()
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
          pcl::PointXYZ pt;
          pt.x = (*itx).first*resolution_;
          pt.y = (*ity).first*resolution_;
          pt.z = (*itz).first*height_resolution_;   
          if((*itz).second.first->points.size()>1)
            *pcl_msg += (*(*itz).second.first);  
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

void MultiLayerSpinningLidar::addCastingMarker(const pcl::PointXYZ& pt, size_t id, visualization_msgs::msg::MarkerArray& markerArray){

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
    p.x = trans_gbl2s_af3_.translation().x();
    p.y = trans_gbl2s_af3_.translation().y();
    p.z = trans_gbl2s_af3_.translation().z();  
    markerEdge.points.push_back(p);
    markerEdge.id = id+1;
    markerArray.markers.push_back(markerEdge);
}


void MultiLayerSpinningLidar::resetdGraph(){

  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s starts to reset dynamic graph.", name_.c_str());
  dGraph_.clear();
  dGraph_.initial(shared_data_->static_ground_size_, gbl_utils_->getMaxObstacleDistance());
  pct_marking_ = std::make_shared<Marking>(&dGraph_, gbl_utils_->getInflationRadius(), shared_data_->kdtree_ground_, resolution_, height_resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s done dynamic graph regeneration.", name_.c_str());
}

double MultiLayerSpinningLidar::get_dGraphValue(const unsigned int index){
  return pct_marking_->get_dGraphValue(index);
}

bool MultiLayerSpinningLidar::isCurrent(){
  
  auto time_diff = (clock_->now() - last_observation_time_).seconds();
  if(time_diff > expected_sensor_time_)
    current_ = false;
  else
    current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MultiLayerSpinningLidar::getObservation(){

  return sensor_current_observation_;

}

}//end of name space