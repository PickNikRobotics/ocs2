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

#include "ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h"

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
TargetTrajectoriesKeyboardPublisher::TargetTrajectoriesKeyboardPublisher(::rclcpp::Node::SharedPtr& nodeHandle, const std::string& topicPrefix,
                                                                         const scalar_array_t& targetCommandLimits,
                                                                         CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun)
    : targetCommandLimits_(Eigen::Map<const vector_t>(targetCommandLimits.data(), targetCommandLimits.size())),
      commandLineToTargetTrajectoriesFun_(std::move(commandLineToTargetTrajectoriesFun)),
      node_(nodeHandle)  {
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
    "joy", 10, std::bind(&TargetTrajectoriesKeyboardPublisher::joyCallback, this, std::placeholders::_1));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishKeyboardCommand(const std::string& commadMsg) {
  while (rclcpp::ok()) {
    // get command line
    std::cout << commadMsg << ": ";
    const size_t targetCommandSize = targetCommandLimits_.size();
    const vector_t commandLineInput = vector_t::Zero(targetCommandSize).cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);

    // get the latest observation
    ::rclcpp::spin_some(node_);

    if (observationReceived_) {
      SystemObservation observation;
      {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        observation = latestObservation_;
      }

      // get TargetTrajectories
      const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(commandLineInput, observation);

      // publish TargetTrajectories
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

      // display
      std::cout << "The following command is published: [" << toDelimitedString(commandLineInput) << "]\n\n";
    }
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishJoystickCommand(const std::string& commadMsg) {
  while (rclcpp::ok()) {
    std::cout << commadMsg << ": ";
    std::this_thread::sleep_for(5s);
    // getCommandLine();
    // vector_t commandLineInput = getCommandLine().cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);
    const vector_t commandLineInput = getLatestJoyCommand().cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);
    std::cout << "command is: " << commandLineInput.transpose() << std::endl;
    // get the latest observation
    ::rclcpp::spin_some(node_);
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(commandLineInput, observation);

    // publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
  }  // end of while loop
}

void TargetTrajectoriesKeyboardPublisher::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{
  const std::lock_guard<std::mutex> lock(joymutex_);
  command_(0) = msg->axes[1];
  command_(1) = msg->axes[0];
  command_(2) = 0.0;
  command_(3) = msg->axes[3];
  // new_data_.set_value(command);
  std::cout << "filled in data" << std::endl;
}

Eigen::Vector4d TargetTrajectoriesKeyboardPublisher::getLatestJoyCommand(){
  const std::lock_guard<std::mutex> lock(joymutex_);
  return command_;
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t TargetTrajectoriesKeyboardPublisher::getCommandLine() {
  // get command line as one long string
  auto shouldTerminate = []() { return !rclcpp::ok(); };
  const std::string line = getCommandLineString(shouldTerminate);

  // a line to words
  const std::vector<std::string> words = stringToWords(line);

  const size_t targetCommandSize = targetCommandLimits_.size();
  vector_t targetCommand = vector_t::Zero(targetCommandSize);
  for (size_t i = 0; i < std::min(words.size(), targetCommandSize); i++) {
    targetCommand(i) = static_cast<scalar_t>(stof(words[i]));
  }

  return targetCommand;
}

}  // namespace ocs2
