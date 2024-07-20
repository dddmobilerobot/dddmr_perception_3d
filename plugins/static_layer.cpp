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
  new_map_ = new_ground_ = false;
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
  
}

void StaticLayer::selfClear(){

  std::unique_lock<std::recursive_mutex> lock(cb_mutex_);
  if(new_ground_ && new_map_){

    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s has already received two msg.", name_.c_str());
    shared_data_->requireUpdate();

    shared_data_->map_require_update_[name_] = false;
    new_ground_ = false;
    new_map_ = false;
    shared_data_->is_static_layer_ready_ = true;
  }
  else if(new_ground_!=new_map_){ //@ disable ready flag when state is different
    shared_data_->is_static_layer_ready_ = false;
  }

}

void StaticLayer::resetdGraph(){}

double StaticLayer::get_dGraphValue(const unsigned int index){
  //@ No dGraph value in static layer, return 100 km
  return 99999.9;
}

bool StaticLayer::isCurrent(){
  
  current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StaticLayer::getObservation(){

  return sensor_current_observation_;

}

}//end of name space
