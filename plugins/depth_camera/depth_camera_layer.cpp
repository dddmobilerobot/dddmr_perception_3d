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
  buffer->lock();
  buffer->bufferCloud(*msg);
  buffer->unlock();
}

void DepthCameraLayer::selfClear(){
  bool a = isCurrent();
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%d", a);
}

void DepthCameraLayer::selfMark(){

}

pcl::PointCloud<pcl::PointXYZI>::Ptr DepthCameraLayer::getObservation(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr dpc;
  dpc.reset(new pcl::PointCloud<pcl::PointXYZI>());
  return dpc;
}

void DepthCameraLayer::resetdGraph(){
  
}

double DepthCameraLayer::get_dGraphValue(const unsigned int index){
  return 9999999;
}

bool DepthCameraLayer::isCurrent(){

  bool current = true;
  for(auto i=observation_buffers_.begin(); i!=observation_buffers_.end();i++){
    current = current && (*i).second->isCurrent();
  }
  return current;
}

}