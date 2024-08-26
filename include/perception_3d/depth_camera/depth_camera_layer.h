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
#ifndef PERCEPTION_3D_DEPTH_CAMERA_LAYER_H_
#define PERCEPTION_3D_DEPTH_CAMERA_LAYER_H_

#include <perception_3d/sensor.h>
#include <perception_3d/cluster_marking.h>
#include <perception_3d/depth_camera/depth_camera_observation_buffer.hpp>

namespace perception_3d
{

class DepthCameraLayer: public Sensor{

  public:

    DepthCameraLayer();
    ~DepthCameraLayer();

    void onInitialize();
    void selfClear();
    void selfMark();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    void resetdGraph();
    double get_dGraphValue(const unsigned int index);
    bool isCurrent();

  private:
    
    std::shared_ptr<perception_3d::Marking> pct_marking_;

    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_observation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_segmentation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_projected_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_window_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cleared_window_marking_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_casting_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gbl_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dGraph_;

    void cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                    const std::shared_ptr<perception_3d::DepthCameraObservationBuffer>& buffer);
    void aggregatePointCloudFromObservations(const pcl::PointCloud<pcl::PointXYZI>::Ptr& resulting_pcl)  ;

    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_pc_map_; 
    std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>> observation_buffers_;
    
    bool is_local_planner_;
    double resolution_, height_resolution_;
    double segmentation_ignore_ratio_;
    geometry_msgs::msg::TransformStamped trans_gbl2b_;
};

}//end of name space
#endif