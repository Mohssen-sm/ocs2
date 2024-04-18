/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_double_integrator_ros/DoubleIntegratorDummyVisualization.h"

namespace ocs2 {
namespace double_integrator {

void DoubleIntegratorDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  jointPublisher_ = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void DoubleIntegratorDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy,
                                                const CommandData& command) {
  const auto& targetTrajectories = command.mpcTargetTrajectories_;
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] = "pri1";
  joint_state.position[0] = observation.state(0);
  joint_state.name[1] = "pri2";
  joint_state.position[1] = observation.state(1);
  joint_state.name[2] = "rev1";
  joint_state.position[2] = observation.state(2);
  joint_state.name[3] = "pri1_target";
  joint_state.position[3] = targetTrajectories.stateTrajectory[0](0);
  joint_state.name[4] = "pri2_target";
  joint_state.position[4] = targetTrajectories.stateTrajectory[0](1);
  joint_state.name[5] = "rev1_target";
  joint_state.position[5] = targetTrajectories.stateTrajectory[0](2);

  jointPublisher_.publish(joint_state);
}

}  // namespace double_integrator
}  // namespace ocs2
