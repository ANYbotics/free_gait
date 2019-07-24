/*
 * FreeGaitActionServer.hpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_ros/StepRosConverter.hpp"

// Anymal model
#include "anymal_model/AnymalModel.hpp"

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <free_gait_msgs/ExecuteStepsAction.h>

// STD
#include <string>
#include <memory>
#include <atomic>

namespace free_gait {

class FreeGaitActionServer
{
 public:
  FreeGaitActionServer(ros::NodeHandle nodeHandle, const std::string& name,
                       Executor& executor, AdapterBase& adapter);

  virtual ~FreeGaitActionServer();

  void initialize();
//  void setExecutor(std::shared_ptr<Executor> executor);
//  void setAdapter(std::shared_ptr<AdapterBase> adapter);
  void start();
  void update();
  void block();
  void unblock();
  void shutdown();

  bool isActive();
  bool isBlocked();
  void goalCallback();
  void preemptCallback();
  void publishFeedback();
  void setSucceeded();
  void setPreempted();
  void setAborted();

 private:
  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;
  free_gait::Executor& executor_;

  //! ROS converter.
  StepRosConverter adapter_;

  //! Action server.
  std::string name_;
  actionlib::SimpleActionServer<free_gait_msgs::ExecuteStepsAction> server_;
  free_gait_msgs::ExecuteStepsActionResult result_;

  //! True if in process of initializing a new goal.
  std::atomic<bool> isInitializingNewGoal_;
  //! True if in process of preempting.
  bool isPreempting_;

  //! True if server is blocked (will not accept any goals)
  bool isBlocked_;

  //! Number of steps of the current goal.
  size_t nStepsInCurrentGoal_;
};

} /* namespace */
