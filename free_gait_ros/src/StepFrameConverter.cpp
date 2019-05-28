/*
 * StepFrameConverter.cpp
 *
 *  Created on: Nov 11, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <free_gait_ros/StepFrameConverter.hpp>
#include <kindr_ros/kindr_ros.hpp>

namespace free_gait {

StepFrameConverter::StepFrameConverter(const AdapterBase& adapter)
    : adapter_(adapter)
{
}

bool StepFrameConverter::adaptCoordinates(StepQueue& stepQueue, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame)          
{
  for (Step& step : stepQueue.queue_) {
    if (!adaptCoordinates(step, sourceFrameId, targetFrameId, transformInSourceFrame)) return false;
  }
  return true;
}


bool StepFrameConverter::adaptCoordinates(Step& step, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame)
{
  // Leg motions.
  for (const auto& legMotion : step.getLegMotions()) {

    // Foostep.
    if (legMotion.second->getType() == LegMotionBase::Type::Footstep) {
      Footstep& footstep = dynamic_cast<Footstep&>(*(legMotion.second));
      if (!adaptCoordinates(footstep, sourceFrameId, targetFrameId, transformInSourceFrame)) return false;
    }

    // EndEffectorTrajectory.
    if (legMotion.second->getType() == LegMotionBase::Type::EndEffectorTrajectory) {
      EndEffectorTrajectory& endEffectorTrajectory = dynamic_cast<EndEffectorTrajectory&>(*(legMotion.second));
      if (!adaptCoordinates(endEffectorTrajectory, sourceFrameId, targetFrameId, transformInSourceFrame)) return false;
    }

  }

  // Base motion.
  if (step.hasBaseMotion()) {

    const auto& baseMotion = step.getBaseMotion();

    // Base Auto.
//    if (baseMotion.getType() == BaseMotionBase::Type::Auto) {
//      BaseAuto& baseAuto = dynamic_cast<BaseAuto&>(baseMotion);
//      if (!adaptCoordinates(baseAuto, sourceFrameId, targetFrameId, transformInTargetFrame)) return false;
//    }

    // Base Trajectory.
    if (baseMotion.getType() == BaseMotionBase::Type::Trajectory){
      const BaseTrajectory& baseTrajectory = dynamic_cast<const BaseTrajectory&>(baseMotion);
      if (!adaptCoordinates(const_cast<BaseTrajectory&> (baseTrajectory), sourceFrameId, targetFrameId, transformInSourceFrame)) return false;
    }
  }
  return true;
}

bool StepFrameConverter::adaptCoordinates(Footstep& footstep, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame)
{
  Transform transform;
  if (footstep.getFrameId(ControlLevel::Position) == sourceFrameId) {
    if (!getTransform(sourceFrameId, targetFrameId, transformInSourceFrame, transform)) return false;
    footstep.target_ = transform.transform(footstep.target_);
    footstep.frameId_ = targetFrameId;
  }

  return true;
}

bool StepFrameConverter::adaptCoordinates(EndEffectorTrajectory& endEffectorTrajectory, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame)
{
  Transform transform;
  if (endEffectorTrajectory.getFrameId(ControlLevel::Position) == sourceFrameId) {
    if (!getTransform(sourceFrameId, targetFrameId, transformInSourceFrame, transform)) return false;
    for (auto& knot : endEffectorTrajectory.values_.at(ControlLevel::Position)){
      knot = (transform.transform(Position(knot))).vector();
    }
    endEffectorTrajectory.frameIds_.at(ControlLevel::Position) = targetFrameId;
  }

  return true;
}

bool StepFrameConverter::adaptCoordinates(BaseTrajectory& baseTrajectory, const std::string& sourceFrameId,
                                          const std::string& targetFrameId,
                                          const Transform& transformInSourceFrame)
{
  Transform transform;
  if (baseTrajectory.getFrameId(ControlLevel::Position) == sourceFrameId) {
    if (!getTransform(sourceFrameId, targetFrameId, transformInSourceFrame, transform)) return false;
    for (auto& knot : baseTrajectory.values_.at(ControlLevel::Position)){
      knot.getPosition() = transform.transform(knot.getPosition());
      knot.getRotation() = transform.getRotation()*knot.getRotation();
    }
    baseTrajectory.frameIds_.at(ControlLevel::Position) = targetFrameId;
  }

  return true;
}

bool StepFrameConverter::getTransform(const std::string& sourceFrameId,
                                      const std::string& targetFrameId,
                                      const Transform& transformInSourceFrame,
                                      Transform& transform)
{

  transform = adapter_.transformPose(sourceFrameId,targetFrameId, transformInSourceFrame);
  return true;
}

} /* namespace free_gait */
