/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include <ros/init.h>

#include "ocs2_anymal_croc/AnymalInterface.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_croc_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalCrocInterface(anymal::getTaskFileFolderCroc(taskName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePathCroc(taskName));
  const auto ddpSettings = ocs2::ddp::loadSettings(anymal::getTaskFilePathCroc(taskName));
  quadrupedMpcNode(nodeHandle, *anymalInterface, mpcSettings, ddpSettings);

  return 0;
}
