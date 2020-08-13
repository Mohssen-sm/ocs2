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

#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

namespace mobile_manipulator {

class MobileManipulatorDummyVisualization final : public ocs2::DummyObserver {
 public:
  explicit MobileManipulatorDummyVisualization(ros::NodeHandle& nodeHandle) { launchVisualizerNode(nodeHandle); }

  ~MobileManipulatorDummyVisualization() override = default;

  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& policy, const ocs2::CommandData& command) override;

 private:
  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishObservation(const ros::Time& timeStamp, const ocs2::SystemObservation& observation);
  void publishDesiredTrajectory(const ros::Time& timeStamp, const ocs2::CostDesiredTrajectories& costDesiredTrajectory);
  void publishOptimizedTrajectory(const ros::Time& timeStamp, const ocs2::PrimalSolution& policy);

  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
  tf::TransformBroadcaster tfBroadcaster_;

  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher stateOptimizedPosePublisher_;
};

}  // namespace mobile_manipulator
