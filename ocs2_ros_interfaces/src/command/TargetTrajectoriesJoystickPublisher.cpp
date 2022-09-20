/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_ros_interfaces/command/TargetTrajectoriesJoystickPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <condition_variable>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesJoystickPublisher::TargetTrajectoriesJoystickPublisher(::rclcpp::Node::SharedPtr& nodeHandle, const std::string& topicPrefix,
                                                                         const scalar_array_t& targetCommandLimits,
                                                                         CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun)
    : targetCommandLimits_(Eigen::Map<const vector_t>(targetCommandLimits.data(), targetCommandLimits.size())),
      commandLineToTargetTrajectoriesFun_(std::move(commandLineToTargetTrajectoriesFun)),
      node_(nodeHandle)  {
  observationReceived_ = false;
  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::msg::MPCObservation::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    observationReceived_ = true;
  };
  observationSubscriber_ = nodeHandle->create_subscription<ocs2_msgs::msg::MPCObservation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));

  command_ << 0,0,0,0;
  //subsribe to joystick
  joysubscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&TargetTrajectoriesJoystickPublisher::joyCallback, this, std::placeholders::_1));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesJoystickPublisher::publishJoystickCommand(const std::string& commadMsg) {
  while (rclcpp::ok()) {
    Eigen::Vector4d joy_command = getLatestJoyCommand().cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);
    
    // get the latest observation
    ::rclcpp::spin_some(node_);

    if (observationReceived_) {
      SystemObservation observation;
      {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        observation = latestObservation_;
      }

      // Rotate joystick command to world frame since that's what ocs2 expects
      Eigen::Quaterniond curr_rot = Eigen::AngleAxisd(observation.state[9],Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(observation.state[10],Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(observation.state[11],Eigen::Vector3d::UnitX());
      joy_command.segment<3>(0) = curr_rot.matrix()*joy_command.segment<3>(0);
      const vector_t commandLineInput(joy_command);
      
      // get TargetTrajectories
      const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(commandLineInput, observation);

      // publish TargetTrajectories
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    }
  }  // end of while loop
}

void TargetTrajectoriesJoystickPublisher::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{
  const std::lock_guard<std::mutex> lock(joymutex_);
  command_(0) = msg->axes[1];
  command_(1) = msg->axes[0];
  command_(2) = 0.0;
  command_(3) = msg->axes[3]*90;
}

Eigen::Vector4d TargetTrajectoriesJoystickPublisher::getLatestJoyCommand(){
  const std::lock_guard<std::mutex> lock(joymutex_);
  return command_;
}

}  // namespace ocs2
