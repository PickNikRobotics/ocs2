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

#include <rclcpp/rclcpp.hpp>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>

using namespace ocs2;
using namespace mobile_manipulator;

static auto LOGGER = rclcpp::get_logger("LeggedRobotDDPMpcNode");

auto declareAndGetStringParam = [] (rclcpp::Node::SharedPtr &node, const std::string &param, std::string &param_value) {
  if (!node->has_parameter(param))
  {
    node->declare_parameter(param, std::string(""));
  }
  rclcpp::Parameter parameter;
  node->get_parameter(param, parameter);
  param_value = parameter.as_string();
  RCLCPP_INFO_STREAM(LOGGER, "Retrieved parameter " << param << " with value " << param_value);
};

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_mpc");
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;
  declareAndGetStringParam(nodeHandle, "task_file", taskFile);
  declareAndGetStringParam(nodeHandle, "lib_folder", libFolder);
  declareAndGetStringParam(nodeHandle, "urdf_file", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  // Robot interface
  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  // ROS ReferenceManager
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr(
      new ocs2::RosReferenceManager(robotName, interface.getReferenceManagerPtr()));
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
                               interface.getOptimalControlProblem(), interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  // Successful exit
  return 0;
}
