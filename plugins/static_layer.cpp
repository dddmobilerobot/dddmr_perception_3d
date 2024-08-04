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
#include <perception_3d/static_layer.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::StaticLayer, perception_3d::Sensor)

namespace perception_3d
{

StaticLayer::StaticLayer(){

}

StaticLayer::~StaticLayer(){

}

void StaticLayer::onInitialize()
{ 
  
  ptrInitial();

  cbs_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;

  node_->declare_parameter(name_ + ".radius_of_ground_connection", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".radius_of_ground_connection", radius_of_ground_connection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "radius_of_ground_connection: %.2f", radius_of_ground_connection_);

  node_->declare_parameter(name_ + ".use_adaptive_connection", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".use_adaptive_connection", use_adaptive_connection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "use_adaptive_connection: %d", use_adaptive_connection_);  
  
  node_->declare_parameter(name_ + ".adaptive_connection_number", rclcpp::ParameterValue(20));
  node_->get_parameter(name_ + ".adaptive_connection_number", adaptive_connection_number_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "adaptive_connection_number: %d", adaptive_connection_number_);  

  node_->declare_parameter(name_ + ".hollow_hole_tolerance", rclcpp::ParameterValue(5));
  node_->get_parameter(name_ + ".hollow_hole_tolerance", hollow_hole_tolerance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "hollow_hole_tolerance: %d", hollow_hole_tolerance_);    
  
  node_->declare_parameter(name_ + ".turning_weight", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".turning_weight", turning_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "turning_weight: %.2f", turning_weight_);    

  node_->declare_parameter(name_ + ".intensity_search_radius", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".intensity_search_radius", intensity_search_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "intensity_search_radius: %.2f", intensity_search_radius_);    

  node_->declare_parameter(name_ + ".intensity_search_punish_weight", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".intensity_search_punish_weight", intensity_search_punish_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "intensity_search_punish_weight: %.2f", intensity_search_punish_weight_);    

  node_->declare_parameter(name_ + ".static_imposing_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".static_imposing_radius", static_imposing_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "static_imposing_radius: %.2f", static_imposing_radius_);    

  node_->declare_parameter(name_ + ".is_local_planner", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".is_local_planner", is_local_planner_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "is_local_planner: %d", is_local_planner_);      

  
  pub_dGraph_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/dGraph", 2);

  pcl_map_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
    std::bind(&StaticLayer::cbMap, this, std::placeholders::_1), sub_options);

  pcl_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
    std::bind(&StaticLayer::cbGround, this, std::placeholders::_1), sub_options);


}

void StaticLayer::ptrInitial(){
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  //shared_data_->kdtree_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  //shared_data_->kdtree_ground_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->sGraph_ptr_ = std::make_shared<perception_3d::StaticGraph>();
  new_map_ = new_ground_ = is_local_planner_ = false;
}

void StaticLayer::cbMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::unique_lock<std::recursive_mutex> lock(cb_mutex_);
  /*transform to point cloud library format first so we can leverage PCL*/
  pcl::fromROSMsg(*msg, *pcl_map_);  

  if(shared_data_->static_map_size_!=pcl_map_->points.size()){
    new_map_ = true;
    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s receive new \033[1;32mMap\033[0m with size: %lu", name_.c_str(), pcl_map_->points.size());
  }
  shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  shared_data_->kdtree_map_->setInputCloud(pcl_map_);
  shared_data_->pcl_map_ = pcl_map_;
}

void StaticLayer::cbGround(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  std::unique_lock<std::recursive_mutex> lock(cb_mutex_);
  pcl::fromROSMsg(*msg, *pcl_ground_);  

  if(shared_data_->static_ground_size_!=pcl_ground_->points.size()){
    new_ground_ = true;
    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s receive new \033[1;32mGround\033[0m with size: %lu", name_.c_str(), pcl_ground_->points.size());
  }  
  shared_data_->static_ground_size_ = pcl_ground_->points.size();
  shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  shared_data_->kdtree_ground_->setInputCloud(pcl_ground_);
  shared_data_->pcl_ground_ = pcl_ground_;
 
}


void StaticLayer::selfMark(){
  if(pub_dGraph_->get_subscription_count()>0){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg2 (new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t index=0;index<shared_data_->static_ground_size_;index++){
      pcl::PointXYZI ipt;
      ipt.x = shared_data_->pcl_ground_->points[index].x;
      ipt.y = shared_data_->pcl_ground_->points[index].y;
      ipt.z = shared_data_->pcl_ground_->points[index].z;   
      ipt.intensity = get_dGraphValue(index);
      pcl_msg2->push_back(ipt);
    }
    sensor_msgs::msg::PointCloud2 ros_pc2_msg2;
    pcl_msg2->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pcl_msg2, ros_pc2_msg2);
    pub_dGraph_->publish(ros_pc2_msg2);
  }
}

void StaticLayer::selfClear(){

  std::unique_lock<std::recursive_mutex> lock(cb_mutex_);
  if(new_ground_ && new_map_){

    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s has already received two msg.", name_.c_str());
    shared_data_->requireUpdate();

    shared_data_->map_require_update_[name_] = false;
    new_ground_ = false;
    new_map_ = false;

    //@ radius search connection to generate sGraph
    resetdGraph();
    if(!is_local_planner_)
      radiusSearchConnection();
    
    shared_data_->is_static_layer_ready_ = true;
  }
  else if(new_ground_!=new_map_){ //@ disable ready flag when state is different
    shared_data_->is_static_layer_ready_ = false;
  }

}

void StaticLayer::radiusSearchConnection(){
  unsigned int index_cnt = 0;
  for(auto it = pcl_ground_->points.begin();it!=pcl_ground_->points.end();it++){

    pcl::PointXYZ pcl_node;
    pcl_node = (*it);

    //@Kd-tree to find nn point for planar equation
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(!use_adaptive_connection_){
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, radius_of_ground_connection_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    }
    else{

      int hard_interrupt_cnt = 100;
      float search_r = 0.5;
      int search_cnt = 1;
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      while(pointIdxRadiusSearch.size()<adaptive_connection_number_ && hard_interrupt_cnt>0){
        search_cnt++;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);    
        hard_interrupt_cnt--;   
      }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr nn_pc (new pcl::PointCloud<pcl::PointXYZ>);
    for(auto it = pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end();it++){
      
      //chekc relative z value for the edge, because we need to eliminate stair and wheel chair passage issue
      edge_t a_edge;
      auto node = index_cnt;
      a_edge.first = (*it);
      double z_diff = fabs(pcl_ground_->points[node].z - pcl_ground_->points[a_edge.first].z);
      if(z_diff >=0.1){
        RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Connection between %u and %u is %.2f",node, a_edge.first, z_diff);
        continue;
      }
      //@Create an edge
      a_edge.second = sqrt(pcl::geometry::squaredDistance(pcl_ground_->points[node], pcl_ground_->points[a_edge.first]));
      shared_data_->sGraph_ptr_->insertNode(node, a_edge);

      //@Push bach the points for plane equation later
      nn_pc->push_back(pcl_ground_->points[(*it)]);
    }

    float weight = 1.0;
    float intensity_weight = 1.0;
    float max_radius = intensity_search_radius_; //this value is suggested to be 1.0 meters, because if it is too large, the narrow passage will be miscalculated

    //@ consider this scenario to be boundary of ground
    if(nn_pc->points.size()<5){
      //This node is orphan, set high weight to it
      weight = 1000;
    }
    else{
      //@Use RANSAC to get normal
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.05); 

      seg.setInputCloud (nn_pc);
      seg.segment (*inliers, *coefficients);
      
      //@We get planar equation (coefficients), we now calculate z by iterate x,y value
      //@We use polar coordinate to generate points (i.e.: iterate theta with given radius)
      //@We search multiple rings
      int reject_threshold = 0;
      for(float ring_radius=max_radius; ring_radius>0; ring_radius-=0.25){
        for(float d_theta=-3.1415926; d_theta<=3.1415926; d_theta+=0.174){ //per 10 deg
          pcl::PointXYZ pcl_ring;
          pcl_ring.x = pcl_node.x + ring_radius*sin(d_theta);
          pcl_ring.y = pcl_node.y + ring_radius*cos(d_theta);
          pcl_ring.z = (-coefficients->values[3]-coefficients->values[0]*pcl_ring.x-coefficients->values[1]*pcl_ring.y)/coefficients->values[2];
          if(isinf(pcl_ring.z)){
            pcl_ring.z = 0.0;
            //RCLCPP_INFO(this->get_logger().get_child(name_), "%.2f,%.2f,%.2f", pcl_ring.x, pcl_ring.y, pcl_ring.z);
          }
          if(std::isnan(pcl_ring.z))
            continue;
          //@Search around the ring
          std::vector<int> pointIdxRadiusSearch_ring;
          std::vector<float> pointRadiusSquaredDistance_ring;
          if(shared_data_->kdtree_ground_->radiusSearch (pcl_ring, 0.3, pointIdxRadiusSearch_ring, pointRadiusSquaredDistance_ring)<1) //0.3 is related to resolution, looks good for 0.5 m voxel
            reject_threshold++;
        }
      }
      if(reject_threshold>hollow_hole_tolerance_)
        intensity_weight += reject_threshold*intensity_search_punish_weight_;

      //@ use map to impose weight on each node
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_map_->radiusSearch (pcl_node, 2.0, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_z_axes(new pcl::PointCloud<pcl::PointXYZ>);
      for(auto i_pcl_z_axes=pointIdxRadiusSearch.begin();i_pcl_z_axes!=pointIdxRadiusSearch.end();i_pcl_z_axes++){
        pcl_z_axes->push_back(pcl_map_->points[*i_pcl_z_axes]);
      }

      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (pcl_z_axes);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (pcl_node.z+0.1, pcl_node.z+1.0);
      pass.filter (*pcl_z_axes);
      for(auto pt=pcl_z_axes->points.begin(); pt!=pcl_z_axes->points.end(); pt++){
        double dx = (*pt).x - pcl_node.x;
        double dy = (*pt).y - pcl_node.y;
        dGraph_.setValue(index_cnt, sqrt(dx*dx+dy*dy)+0.25);
      }
      
    }
    shared_data_->sGraph_ptr_->insertWeight(index_cnt, intensity_weight);//make the value of weight to 1 for non-weighted A*
    index_cnt++;
  }
  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Static graph has been generated.");
}

void StaticLayer::resetdGraph(){
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s starts to reset dynamic graph.", name_.c_str());
  dGraph_.clear();
  dGraph_.initial(shared_data_->static_ground_size_, gbl_utils_->getMaxObstacleDistance());
}

double StaticLayer::get_dGraphValue(const unsigned int index){
  return dGraph_.getValue(index);
}

bool StaticLayer::isCurrent(){
  
  current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StaticLayer::getObservation(){

  return sensor_current_observation_;

}

}//end of name space
