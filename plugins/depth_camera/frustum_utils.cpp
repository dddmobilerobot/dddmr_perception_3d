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

#include <perception_3d/depth_camera/frustum_utils.h>

using namespace std::chrono_literals;
namespace perception_3d
{

void FrustumUtils::setObservationBuffers(std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>>& observation_buffers){
  observation_buffers_ = observation_buffers;

  //@ generate frustum marker for visualization from observation buffers
  current_frustums_marker_array_.markers.clear();
  for (auto it = observation_buffers.begin(); it != observation_buffers.end(); ++it)
  {

    std::vector<perception_3d::DepthCameraObservation> observations_of_a_camera;
    (*it).second->getObservations(observations_of_a_camera);
    perception_3d::DepthCameraObservation& obs = observations_of_a_camera.back();

    visualization_msgs::msg::Marker frustum_marker;
    frustum_marker.header.frame_id = (*it).second->getGlobalFrame();
    frustum_marker.action = visualization_msgs::msg::Marker::ADD;
    frustum_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    frustum_marker.pose.orientation.w = 1.0;
    frustum_marker.ns = (*it).first;
    frustum_marker.id = 0;
    /* Note of frustum sequence -> TLNear: Top Left Near
    TLNear = frustum_->points[0];
    TRNear = frustum_->points[1];
    BLNear = frustum_->points[2];
    BRNear = frustum_->points[3];
    TLFar = frustum_->points[4];
    TRFar = frustum_->points[5];
    BLFar = frustum_->points[6];
    BRFar = frustum_->points[7];
    */

    frustum_marker.scale.x = 0.02;
    frustum_marker.color.r = 1.0; frustum_marker.color.g = 0.6; frustum_marker.color.b = 0.0;
    frustum_marker.color.a = 0.6;
    geometry_msgs::msg::Point gpt1, gpt2;
    gpt1.x = obs.frustum_->points[0].x; gpt1.y = obs.frustum_->points[0].y; gpt1.z = obs.frustum_->points[0].z; 
    gpt2.x = obs.frustum_->points[1].x; gpt2.y = obs.frustum_->points[1].y; gpt2.z = obs.frustum_->points[1].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[2].x; gpt1.y = obs.frustum_->points[2].y; gpt1.z = obs.frustum_->points[2].z; 
    gpt2.x = obs.frustum_->points[3].x; gpt2.y = obs.frustum_->points[3].y; gpt2.z = obs.frustum_->points[3].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[4].x; gpt1.y = obs.frustum_->points[4].y; gpt1.z = obs.frustum_->points[4].z; 
    gpt2.x = obs.frustum_->points[5].x; gpt2.y = obs.frustum_->points[5].y; gpt2.z = obs.frustum_->points[5].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[6].x; gpt1.y = obs.frustum_->points[6].y; gpt1.z = obs.frustum_->points[6].z; 
    gpt2.x = obs.frustum_->points[7].x; gpt2.y = obs.frustum_->points[7].y; gpt2.z = obs.frustum_->points[7].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[0].x; gpt1.y = obs.frustum_->points[0].y; gpt1.z = obs.frustum_->points[0].z; 
    gpt2.x = obs.frustum_->points[4].x; gpt2.y = obs.frustum_->points[4].y; gpt2.z = obs.frustum_->points[4].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[1].x; gpt1.y = obs.frustum_->points[1].y; gpt1.z = obs.frustum_->points[1].z; 
    gpt2.x = obs.frustum_->points[5].x; gpt2.y = obs.frustum_->points[5].y; gpt2.z = obs.frustum_->points[5].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[2].x; gpt1.y = obs.frustum_->points[2].y; gpt1.z = obs.frustum_->points[2].z; 
    gpt2.x = obs.frustum_->points[6].x; gpt2.y = obs.frustum_->points[6].y; gpt2.z = obs.frustum_->points[6].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[3].x; gpt1.y = obs.frustum_->points[3].y; gpt1.z = obs.frustum_->points[3].z; 
    gpt2.x = obs.frustum_->points[7].x; gpt2.y = obs.frustum_->points[7].y; gpt2.z = obs.frustum_->points[7].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[4].x; gpt1.y = obs.frustum_->points[4].y; gpt1.z = obs.frustum_->points[4].z; 
    gpt2.x = obs.frustum_->points[6].x; gpt2.y = obs.frustum_->points[6].y; gpt2.z = obs.frustum_->points[6].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[5].x; gpt1.y = obs.frustum_->points[5].y; gpt1.z = obs.frustum_->points[5].z; 
    gpt2.x = obs.frustum_->points[7].x; gpt2.y = obs.frustum_->points[7].y; gpt2.z = obs.frustum_->points[7].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[0].x; gpt1.y = obs.frustum_->points[0].y; gpt1.z = obs.frustum_->points[0].z; 
    gpt2.x = obs.frustum_->points[2].x; gpt2.y = obs.frustum_->points[2].y; gpt2.z = obs.frustum_->points[2].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    gpt1.x = obs.frustum_->points[1].x; gpt1.y = obs.frustum_->points[1].y; gpt1.z = obs.frustum_->points[1].z; 
    gpt2.x = obs.frustum_->points[3].x; gpt2.y = obs.frustum_->points[3].y; gpt2.z = obs.frustum_->points[3].z; 
    frustum_marker.points.push_back(gpt1);
    frustum_marker.points.push_back(gpt2);
    current_frustums_marker_array_.markers.push_back(frustum_marker);
  }
}

bool FrustumUtils::isAttachFRUSTUMs(pcl::PointXYZI& testPoint)
{
  //@ we check if the point attaches frustum to solve edge case, if it attaches frustum we clear it.
  for (auto it = observation_buffers_.begin(); it != observation_buffers_.end(); ++it)
  {
    std::vector<perception_3d::DepthCameraObservation> observations_of_a_camera;
    (*it).second->getObservations(observations_of_a_camera);
    perception_3d::DepthCameraObservation& obs = observations_of_a_camera.back();
    for(auto it_plane=obs.frustum_plane_equation_.begin();it_plane!=obs.frustum_plane_equation_.end();it_plane++)
    {
      float a = (*it_plane)[0];
      float b = (*it_plane)[1];
      float c = (*it_plane)[2];
      float d = (*it_plane)[3];
      float dis = fabs(a*testPoint.x+b*testPoint.y+c*testPoint.z+d);
      dis = dis/sqrt(a*a+b*b+c*c);
      float dis2rej = 0.12;

      if(dis <= dis2rej && hypot(testPoint.x-obs.origin_.x, testPoint.y-obs.origin_.y)<obs.max_detect_distance_+0.5)
      {
      //find one frustum such that no attachment and inside frumstum
        for (auto it_inner = observation_buffers_.begin(); it_inner != observation_buffers_.end(); ++it_inner)
        {
          std::vector<perception_3d::DepthCameraObservation> observations_of_a_inner_camera;
          (*it_inner).second->getObservations(observations_of_a_inner_camera);
          if((*it).first==(*it_inner).first)
          {
          //ROS_WARN("Same frustum.");
            continue;
          }
          else if(isInsideFRUSTUMwoAttach(observations_of_a_inner_camera.back(), testPoint))
          {
            return false;
          }
        }
        return true;
      }
    } 
  }
  return false;
}

bool FrustumUtils::isInsideFRUSTUMwoAttach(perception_3d::DepthCameraObservation& observation, pcl::PointXYZI& testPoint)
{
  for(auto it=observation.frustum_plane_equation_.begin();it!=observation.frustum_plane_equation_.end();it++)
  {
    float a = (*it)[0];
    float b = (*it)[1];
    float c = (*it)[2];
    float d = (*it)[3];
    float dis = fabs(a*testPoint.x+b*testPoint.y+c*testPoint.z+d);
    dis = dis/sqrt(a*a+b*b+c*c);
    float dis2rej = 0.12;
    if(dis<=dis2rej && hypot(testPoint.x-observation.origin_.x, testPoint.y-observation.origin_.y)<observation.max_detect_distance_+0.5)
    {
      return false;
    }
  } 

  pcl::PointXYZ testPoint_XYZ;
  testPoint_XYZ.x = testPoint.x;
  testPoint_XYZ.y = testPoint.y;
  testPoint_XYZ.z = testPoint.z;

  pcl::PointXYZ vec_form_pc2corner;
  double test;
  for(int i=0;i<6;i++)
  {
    test = 0.0;
    if(i<3)
    {
      vec_form_pc2corner = observation.getVec(observation.BRNear_,testPoint_XYZ);
      //ROS_DEBUG("%d, %.2f, %.2f, %.2f",i, vec_form_pc2corner.point.x,vec_form_pc2corner.point.y,vec_form_pc2corner.point.z);
      test = vec_form_pc2corner.x*observation.frustum_normal_->points[i].x + vec_form_pc2corner.y*observation.frustum_normal_->points[i].y +vec_form_pc2corner.z*observation.frustum_normal_->points[i].z;
      if(test<0)
      {
        //ROS_DEBUG("Reject by %d plane.", i);
        return false;
      }
    }
    else
    {
      vec_form_pc2corner = observation.getVec(observation.TLFar_,testPoint_XYZ);
      //ROS_DEBUG("%d: %.2f, %.2f, %.2f",i, vec_form_pc2corner.point.x,vec_form_pc2corner.point.y,vec_form_pc2corner.point.z);
      test = vec_form_pc2corner.x*observation.frustum_normal_->points[i].x + vec_form_pc2corner.y*observation.frustum_normal_->points[i].y +vec_form_pc2corner.z*observation.frustum_normal_->points[i].z;
      if(test<0)
      {
        //ROS_DEBUG("Reject by %d plane.", i);
        return false;
      }
    }
  }
  return true;
}

bool FrustumUtils::isinFrustumsObservations(pcl::PointXYZ& testPoint){
  pcl::PointXYZI testPoint_XYZI;
  testPoint_XYZI.x = testPoint.x;
  testPoint_XYZI.y = testPoint.y;
  testPoint_XYZI.z = testPoint.z;
  return isinFrustumsObservations(testPoint_XYZI);
}

bool FrustumUtils::isinFrustumsObservations(pcl::PointXYZI& testPoint){

  pcl::PointXYZ testPoint_XYZ;
  testPoint_XYZ.x = testPoint.x;
  testPoint_XYZ.y = testPoint.y;
  testPoint_XYZ.z = testPoint.z;
  //RCLCPP_INFO(node_->get_logger(),"Test: %.2f, %.2f, %.2f", testPoint.x, testPoint.y, testPoint.z);
  //@ each observationbuffer is one camera, each camera possess many observation
  //@ we loop camera to get frustum from the last observarion
  for (auto it = observation_buffers_.begin(); it != observation_buffers_.end(); ++it){  
    bool one_frustum_test = true; //if we test the point in one of the frustum, then we can exit
    std::vector<perception_3d::DepthCameraObservation> observations_of_a_camera;
    (*it).second->getObservations(observations_of_a_camera);
    perception_3d::DepthCameraObservation& obs = observations_of_a_camera.back();
    pcl::PointXYZ vec_form_pc2corner;
    double test;
    for(int i=0;i<6;i++)
    {
      test = 0.0;
      if(i<3)
      {
        vec_form_pc2corner = obs.getVec(obs.BRNear_,testPoint_XYZ);
        
        #if(DEBUG)
        RCLCPP_INFO(node_->get_logger(),"%d, %.2f, %.2f, %.2f",i, vec_form_pc2corner.x,vec_form_pc2corner.y,vec_form_pc2corner.z);
        #endif
        
        test = vec_form_pc2corner.x*obs.frustum_normal_->points[i].x + vec_form_pc2corner.y*obs.frustum_normal_->points[i].y +vec_form_pc2corner.z*obs.frustum_normal_->points[i].z;
        
        if(test<0)
        {
          #if(DEBUG)
          RCLCPP_INFO(node_->get_logger(),"Reject by %d plane.", i);
          #endif
          one_frustum_test = false;
          break;
        }
      }
      else
      {
        vec_form_pc2corner = obs.getVec(obs.TLFar_,testPoint_XYZ);
        
        #if(DEBUG)
        RCLCPP_INFO(node_->get_logger(),"%d: %.2f, %.2f, %.2f",i, vec_form_pc2corner.x,vec_form_pc2corner.y,vec_form_pc2corner.z);
        #endif

        test = vec_form_pc2corner.x*obs.frustum_normal_->points[i].x + vec_form_pc2corner.y*obs.frustum_normal_->points[i].y +vec_form_pc2corner.z*obs.frustum_normal_->points[i].z;
        if(test<0)
        {
          #if(DEBUG)
          RCLCPP_INFO(node_->get_logger(),"Reject by %d plane.", i);
          #endif
          one_frustum_test = false;
          break;
        }
      }
    }
    if(one_frustum_test)
    {
      return true;
    }
  }
  return false;
}

} //end of ns