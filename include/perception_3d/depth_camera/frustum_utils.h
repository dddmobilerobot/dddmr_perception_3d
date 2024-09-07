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
#ifndef PERCEPTION_3D_DEPTH_CAMERA_FRUSTUM_UTILS_H_
#define PERCEPTION_3D_DEPTH_CAMERA_FRUSTUM_UTILS_H_

#include <perception_3d/sensor.h>
#include <perception_3d/depth_camera/depth_camera_observation_buffer.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace perception_3d
{

class FrustumUtils{

  public:
    void setObservationBuffers(std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>>& observation_buffers);
    bool isinFrustumsObservations(pcl::PointXYZI& testPoint);
    bool isAttachFRUSTUMs(pcl::PointXYZI& testPoint);
    bool isInsideFRUSTUMwoAttach(perception_3d::DepthCameraObservation& observation, pcl::PointXYZI& testPoint);
    bool isinFrustumsObservations(pcl::PointXYZ& testPoint);
    visualization_msgs::msg::MarkerArray current_frustums_marker_array_;
  private:
    std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>> observation_buffers_;
    

};

}//end of name space
#endif