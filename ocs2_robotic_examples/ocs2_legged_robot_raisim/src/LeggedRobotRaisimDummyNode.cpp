/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ros/init.h>
#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h"
#include "ocs2_legged_robot_raisim/LeggedRobotRaisimVisualizer.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 5) {
    throw std::runtime_error("No robot name, config folder, target command file, or description file specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string configName(programArgs[2]);
  const std::string targetCommandFile(programArgs[3]);
  const std::string descriptionFile(programArgs[4]);

  // initialize ros node
  ros::init(argc, argv, robotName + "_raisim_dummy");
  ros::NodeHandle nodeHandle;

  // legged robot interface
  LeggedRobotInterface interface(configName, targetCommandFile, urdf::parseURDFFile(descriptionFile));

  // raisim rollout
  LeggedRobotRaisimConversions conversions(interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
                                           interface.getInitialState());
  RaisimRolloutSettings raisimRolloutSettings(ros::package::getPath("ocs2_legged_robot_raisim") + "/config/raisim.info", "rollout", true);
  conversions.loadSettings(ros::package::getPath("ocs2_legged_robot_raisim") + "/config/raisim.info", "rollout", true);
  RaisimRollout raisimRollout(
      ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/urdf/anymal.urdf",
      ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/meshes",
      [&](const vector_t& state, const vector_t& input) { return conversions.stateToRaisimGenCoordGenVel(state, input); },
      [&](const Eigen::VectorXd& q, const Eigen::VectorXd& dq) { return conversions.raisimGenCoordGenVelToState(q, dq); },
      [&](double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
        return conversions.inputToRaisimGeneralizedForce(time, input, state, q, dq);
      },
      nullptr, raisimRolloutSettings,
      [&](double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
        return conversions.inputToRaisimPdTargets(time, input, state, q, dq);
      });

  // terrain
  raisim::HeightMap* terrainPtr = nullptr;
  std::unique_ptr<RaisimHeightmapRosConverter> heightmapPub;
  if (raisimRolloutSettings.generateTerrain_) {
    raisim::TerrainProperties terrainProperties;
    terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
    terrainProperties.seed = raisimRolloutSettings.terrainSeed_;
    terrainPtr = raisimRollout.generateTerrain(terrainProperties);
    conversions.setTerrain(*terrainPtr);
    heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
    heightmapPub->publishGridmap(*terrainPtr, "odom");
  }

  // mrt
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&raisimRollout);
  mrt.launchNodes(nodeHandle);

  // visualization
  CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                       interface.modelSettings().contactNames3DoF);
  std::shared_ptr<LeggedRobotRaisimVisualizer> leggedRobotRaisimVisualizer(new LeggedRobotRaisimVisualizer(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));
  leggedRobotRaisimVisualizer->updateTerrain();

  // legged robot dummy
  MRT_ROS_Dummy_Loop leggedRobotDummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummy.subscribeObservers({leggedRobotRaisimVisualizer});

  // initial state
  SystemObservation initObservation;
  initObservation.mode = ModeNumber::STANCE;
  initObservation.time = 0.0;
  initObservation.state = interface.getInitialState();
  initObservation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);

  // initial command
  TargetTrajectories initTargetTrajectories({initObservation.time}, {initObservation.state}, {initObservation.input});

  // run dummy
  leggedRobotDummy.run(initObservation, initTargetTrajectories);

  return 0;
}