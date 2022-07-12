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

#include <ros/ros.h>

#include <ocs2_oc/synchronized_module/AugmentedLagrangianObserver.h>

namespace ocs2 {
namespace ros {

/**
 * Creates a ROS-based callback for AugmentedLagrangianObserver that publishes a term's LagrangianMetrics at the
 * requested lookahead time points.
 *
 * @param [in] nodeHandle: ROS node handle.
 * @param [in] observingTimePoints: An array of lookahead times for which we want to publish the values of LagrangianMetrics.
 * @param [in] topicNames: An array of topic names. For each observing time points, you should provide a unique topic name.
 * @return A callback which can be set to SolverObserverModule in order to observe a requested term's LagrangianMetrics.
 */
AugmentedLagrangianObserver::metrics_callback_t createMetricsCallback(::ros::NodeHandle& nodeHandle,
                                                                      const scalar_array_t& observingTimePoints,
                                                                      const std::vector<std::string>& topicNames);

/**
 * Creates a ROS-based callback for AugmentedLagrangianObserver that publishes a term's Lagrange multiplier at the
 * requested lookahead time points.
 *
 * @param [in] nodeHandle: ROS node handle.
 * @param [in] observingTimePoints: An array of lookahead times for which we want to publish the values of multiplier.
 * @param [in] topicNames: An array of topic names. For each observing time points, you should provide a unique topic name.
 * @return A callback which can be set to SolverObserverModule in order to observe a requested term's multiplier.
 */
AugmentedLagrangianObserver::multiplier_callback_t createMultiplierCallback(::ros::NodeHandle& nodeHandle,
                                                                            const scalar_array_t& observingTimePoints,
                                                                            const std::vector<std::string>& topicNames);

}  // namespace ros
}  // namespace ocs2
