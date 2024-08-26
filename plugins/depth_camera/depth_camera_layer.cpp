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
}

void DepthCameraLayer::onInitialize()
{ 
  
  clock_ = node_->get_clock();

  node_->declare_parameter(name_ + ".is_local_planner", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".is_local_planner", is_local_planner_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "is_local_planner: %d", is_local_planner_);

  node_->declare_parameter(name_ + ".xy_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".xy_resolution", resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "xy_resolution: %.2f", resolution_);

  node_->declare_parameter(name_ + ".height_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".height_resolution", height_resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "height_resolution: %.2f", height_resolution_);

  node_->declare_parameter(name_ + ".segmentation_ignore_ratio", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".segmentation_ignore_ratio", segmentation_ignore_ratio_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "segmentation_ignore_ratio: %.2f", segmentation_ignore_ratio_);

  //@ create cluster marking object
  pct_marking_ = std::make_shared<Marking>(&dGraph_, gbl_utils_->getInflationRadius(), shared_data_->kdtree_ground_, resolution_, height_resolution_);
  
  //@ initial all publishers
  pub_current_observation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_observation", 2);
  pub_current_window_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_window_marking", 2);
  pub_current_projected_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_projected", 2);
  pub_current_segmentation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/current_segmentation", 2);
  pub_gbl_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/global_marking", 2);
  pub_dGraph_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/dGraph", 2);
  pub_casting_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/tracing_objects", 2);

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
  bool a = isCurrent();
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%d", a);
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg_gbl_;
  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  aggregatePointCloudFromObservations(pcl_msg_gbl_);
  
  //@ aggregated observation is in global frame already
  if(pcl_msg_gbl_->points.size()<=5)
    return;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr pc_kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
  pc_kdtree->setInputCloud (pcl_msg_gbl_);

  std::vector<pcl::PointIndices> cluster_indices_segmentation;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_segmentation;
  double euclidean_cluster_extraction_tolerance_ = 0.1;
  int euclidean_cluster_extraction_min_cluster_size_ = 1;
  int euclidean_cluster_extraction_max_cluster_size_ = 10;
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

}