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
#include <rclcpp/rclcpp.hpp>
#include <perception_3d/depth_camera/depth_camera_observation_buffer.hpp>

using namespace std::chrono_literals;
namespace perception_3d
{
DepthCameraObservationBuffer::DepthCameraObservationBuffer(std::string topic_name,
                    double observation_keep_time,
                    double expected_update_rate,
                    double min_obstacle_height,
                    double max_obstacle_height,
                    double obstacle_range,
                    tf2_ros::Buffer& tf2_buffer,
                    std::string global_frame,
                    std::string sensor_frame,
                    double FOV_V,
                    double FOV_W,
                    double min_detect_distance,
                    double max_detect_distance,
                    rclcpp::Clock::SharedPtr clock,
                    rclcpp::Logger logger) 
: tf2_buffer_(tf2_buffer)
, observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time))
, expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate))
, last_updated_(clock->now())
, global_frame_(global_frame)
, sensor_frame_(sensor_frame)
, topic_name_(topic_name)
, min_obstacle_height_(min_obstacle_height)
, max_obstacle_height_(max_obstacle_height)
, obstacle_range_(obstacle_range)
, FOV_V_(FOV_V)
, FOV_W_(FOV_W)
, min_detect_distance_(min_detect_distance)
, max_detect_distance_(max_detect_distance)
, clock_(clock)
, logger_(logger)
{
}

DepthCameraObservationBuffer::~DepthCameraObservationBuffer()
{
}

void DepthCameraObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  // create a new observation on the list to be populated
  observation_list_.push_front(DepthCameraObservation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;
  
  if(sensor_frame_ == "")
  {
    RCLCPP_WARN_STREAM(logger_,"Warning: Sensor frame is not provided in yaml file. Using pointcloud header frame id");
  }
  
  /// For debugging only
  //RCLCPP_WARN_STREAM(logger_, "+++++++++++++++++ global frame: " << global_frame_.c_str());
  //RCLCPP_WARN_STREAM(logger_, "+++++++++++++++++ local frame: " << origin_frame.c_str());
  
  /// Check the cloud size 
  pcl::PointCloud<pcl::PointXYZI>::Ptr rawcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(cloud, *rawcloud);
  if(rawcloud->size() >20000)
  {
    RCLCPP_ERROR_STREAM(logger_, "Raw cloud size " << rawcloud->size() <<" is larger than 20000 points. Exiting.. ");
    return;
  }
  
  try
  {
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    geometry_msgs::msg::TransformStamped T_S_C_msg;
    T_S_C_msg = tf2_buffer_.lookupTransform(global_frame_, origin_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
    observation_list_.front().origin_.x = T_S_C_msg.transform.translation.x;
    observation_list_.front().origin_.y = T_S_C_msg.transform.translation.y;
    observation_list_.front().origin_.z = T_S_C_msg.transform.translation.z;
    
    /// Update camera parameters
    observation_list_.front().min_detect_distance_ = min_detect_distance_;
    observation_list_.front().max_detect_distance_ = max_detect_distance_;
    observation_list_.front().FOV_W_ = FOV_W_;
    observation_list_.front().FOV_V_ = FOV_V_;
    
    /// Find frustum vertex (8 points) and transform it to global.
    /// !!! Frustum vertex is usually based on camera_link frame (realsense).
    observation_list_.front().findFrustumVertex();
    
    pcl_conversions::toPCL(cloud.header.stamp, observation_list_.front().frustum_->header.stamp);
    observation_list_.front().frustum_->header.frame_id = origin_frame;
    
    //@get af3 to conver pcl
    Eigen::Affine3d trans_b2s_af3 = tf2::transformToEigen(T_S_C_msg);

    /// ToDo: Remove dependency on pcl_ros to transform the pointcloud
    pcl::transformPointCloud(*observation_list_.front().frustum_, *observation_list_.front().frustum_, trans_b2s_af3);
    
    observation_list_.front().frustum_->header.frame_id = global_frame_;
    
    /// Find frustum normal and plane, note that the planes/normals are in global frame
    /// !!! findFrustumNormal() will assign BRNear_&&TLFar_  which are both in global frame
    observation_list_.front().findFrustumNormal();
    observation_list_.front().findFrustumPlane();
    
    /// Transform the point cloud, from camera_depth_optical_frame
    
    point_cloud_ptr global_frame_cloud(new sensor_msgs::msg::PointCloud2());
    geometry_msgs::msg::TransformStamped tf_stamped = 
    tf2_buffer_.lookupTransform(global_frame_, cloud.header.frame_id, tf2_ros::fromMsg(cloud.header.stamp));
    //tf2_buffer_.lookupTransform(global_frame_, cloud.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
    tf2::doTransform(cloud, *global_frame_cloud, tf_stamped);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*global_frame_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*global_frame_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*global_frame_cloud, "z");

    for (; iter_x !=iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_)
      {
        pcl::PointXYZI tmp_pt;
        tmp_pt.x = *iter_x;
        tmp_pt.y = *iter_y;
        tmp_pt.z = *iter_z;
        tmp_pt.intensity = 0;
        observation_list_.front().cloud_->push_back(tmp_pt);
      }
    }

    if(observation_list_.front().cloud_->size() >10000)
    {
      RCLCPP_ERROR_STREAM(logger_, "ObservationDepth size " << observation_list_.front().cloud_->size() <<" is larger than 5000 points. Exiting.. ");
      return;
    }

    pcl_conversions::toPCL(clock_->now(), observation_list_.front().cloud_->header.stamp);
    observation_list_.front().cloud_->header.frame_id = global_frame_;

    ///RCLCPP_WARN_STREAM(logger_, "++++++observation cloud size: " << observation_list_.front().cloud_->size() << ", count: " << tmp_count);
  }
  catch (tf2::TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    RCLCPP_ERROR(logger_,"TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
                 cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = clock_->now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void DepthCameraObservationBuffer::getObservations(std::vector<DepthCameraObservation>& observations)
{
 
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  std::list<DepthCameraObservation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    observations.push_back(*obs_it);
  }
}

void DepthCameraObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    std::list<DepthCameraObservation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == rclcpp::Duration(rclcpp::Duration::from_seconds(0.0)))
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      perception_3d::DepthCameraObservation& obs = *obs_it;
      
      const rclcpp::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
      
      if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool DepthCameraObservationBuffer::isCurrent() const
{
  
  /// A quick hack
  return true;
  
  if (expected_update_rate_ == rclcpp::Duration(rclcpp::Duration::from_seconds(0.0)))
    return true;

  const rclcpp::Duration update_time = clock_->now() - last_updated_;
  bool current = update_time.seconds() <= expected_update_rate_.seconds();
  
  if (!current)
  {
    RCLCPP_WARN(logger_, "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
      topic_name_.c_str(), update_time.seconds(), expected_update_rate_.seconds());
  }
  return current;
}

void DepthCameraObservationBuffer::resetLastUpdated()
{

  last_updated_ = clock_->now();
}

}
