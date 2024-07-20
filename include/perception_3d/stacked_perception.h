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
#ifndef PERCEPTION_3D_STACKED_PERCEPTION_H_
#define PERCEPTION_3D_STACKED_PERCEPTION_H_

#include <perception_3d/sensor.h>

/*Debug*/
#include <sys/time.h>
#include <time.h>

/*To iterate the ymal for plugins and also for mutex lock*/
#include <pluginlib/class_loader.hpp>
# include <geometry_msgs/msg/pose_stamped.hpp>

namespace perception_3d
{

class StackedPerception{

  public:

    StackedPerception(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger);
    ~StackedPerception();

    /*Plugin operations*/
    void addPlugin(std::shared_ptr<Sensor> plugin);
    std::vector<std::shared_ptr<Sensor> >* getPlugins();

    /*Collected all marked pcl*/
    void doClear_then_Mark();
    
    /*Reset the sensors*/
    void resetdGraph();

    /*Get value from d graph. Loop all plugins and return minimum one (closest to obstacles)*/
    double get_min_dGraphValue(const unsigned int index);

    /*Aggregate observation pointers for local planner*/
    void aggregateObservations();
    
    /*Collect all opinions and return as a vector*/
    std::vector<perception_3d::PerceptionOpinion> getOpinions();
    
    std::shared_ptr<perception_3d::SharedData> getSharedDataPtr(){return shared_data_;}

    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex mutex_t;
    mutex_t* getMutex()
    {
      return access_;
    }

    /*Check sensors are publish topics*/
    bool isSensorOK();
    
  private:
    std::vector<std::shared_ptr<Sensor>> plugins_;

    /*mutex*/
    mutex_t* access_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_prune_plan_;

    std::shared_ptr<SharedData> shared_data_;

    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
};

}//end of name space

#endif