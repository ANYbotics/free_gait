/*
 * AdapterBase.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <free_gait_core/executor/AdapterBase.hpp>

namespace free_gait {

AdapterBase::AdapterBase()
{
}

AdapterBase::~AdapterBase()
{
}

bool AdapterBase::frameIdExists(const std::string& frameId) const
{
  if (frameId == getBaseFrameId()) return true;
  if (frameId == getWorldFrameId()) return true;
  return false;
}

Position AdapterBase::transformPosition(const std::string& inputFrameId,
                                        const std::string& outputFrameId,
                                        const Position& position) const
{
  //TODO(paco): Clean-up function
  Position transformedPosition;
  bool frameError = false;

  if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedPosition = position;
    } else if (outputFrameId == getWorldFrameId()) {
      transformedPosition = getPositionWorldToBaseInWorldFrame() + getOrientationBaseToWorld().rotate(position);
    } else if (frameIdExists(outputFrameId)) {
      const Position positionInWorld = transformPosition(inputFrameId, getWorldFrameId(), position);
      transformedPosition = transformPosition(getWorldFrameId(), outputFrameId, positionInWorld);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedPosition = getOrientationBaseToWorld().inverseRotate(position - getPositionWorldToBaseInWorldFrame());
    } else if (outputFrameId == getWorldFrameId()) {
      transformedPosition = position;
    } else if (frameIdExists(outputFrameId)) {
      transformedPosition = getFrameTransform(outputFrameId).inverseTransform(position);
    } else {
      frameError = true;
    }

  } else if (frameIdExists(inputFrameId)) {

    if (outputFrameId == getBaseFrameId()) {
      const Position positionInWorld = transformPosition(inputFrameId, getWorldFrameId(), position);
      transformedPosition = transformPosition(getWorldFrameId(), outputFrameId, positionInWorld);
    } else if (outputFrameId == getWorldFrameId()) {
      transformedPosition = getFrameTransform(inputFrameId).transform(position);
    } else {
      frameError = true;
    }
  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming position (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedPosition;
}

RotationQuaternion AdapterBase::transformOrientation(const std::string& inputFrameId,
                                                     const std::string& outputFrameId,
                                                     const RotationQuaternion& orientation) const
{
  RotationQuaternion transformedOrientation;
  bool frameError = false;

  if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedOrientation = orientation;
    } else if (frameIdExists(outputFrameId)) {
      const Pose transform(getFrameTransform(outputFrameId));
      transformedOrientation = transform.getRotation().inverted() * orientation;
    } else {
      frameError = true;
    }

  } else if (frameIdExists(inputFrameId)) {

    if (outputFrameId == getWorldFrameId()) {
      const Pose transform(getFrameTransform(inputFrameId));
      transformedOrientation = transform.getRotation() * orientation;
    } else {
      frameError = true;
    }

  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming orientation (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedOrientation;
}

Pose AdapterBase::transformPose(const std::string& inputFrameId, const std::string& outputFrameId,
                                const Pose& pose) const
{
  Pose transformedPose;
  transformedPose.getPosition() = transformPosition(inputFrameId, outputFrameId, pose.getPosition());
  transformedPose.getRotation() = transformOrientation(inputFrameId, outputFrameId, pose.getRotation());
  return transformedPose;
}

LinearVelocity AdapterBase::transformLinearVelocity(const std::string& inputFrameId,
                                                    const std::string& outputFrameId,
                                                    const LinearVelocity& linearVelocity) const
{
  LinearVelocity transformedLinearVelocity(
      transformVector(inputFrameId, outputFrameId, Vector(linearVelocity)));
  return transformedLinearVelocity;
}

LocalAngularVelocity AdapterBase::transformAngularVelocity(
    const std::string& inputFrameId, const std::string& outputFrameId,
    const LocalAngularVelocity& angularVelocity) const
{
  Vector transformedVector = transformVector(inputFrameId, outputFrameId, Vector(angularVelocity.vector()));
  LocalAngularVelocity transformedAngularVelocity(transformedVector.toImplementation());
  return transformedAngularVelocity;
}

Twist AdapterBase::transformTwist(const std::string& inputFrameId, const std::string& outputFrameId,
                                  const Twist& twist) const
{
  Twist transformedTwist;
  transformedTwist.getTranslationalVelocity() = transformLinearVelocity(
      inputFrameId, outputFrameId, twist.getTranslationalVelocity());
  transformedTwist.getRotationalVelocity() = transformAngularVelocity(
      inputFrameId, outputFrameId, twist.getRotationalVelocity());
  return transformedTwist;
}

LinearAcceleration AdapterBase::transformLinearAcceleration(const std::string& inputFrameId,
                                                            const std::string& outputFrameId,
                                                            const LinearAcceleration& linearAcceleration) const
{
  LinearAcceleration transformedLinearAcceleration(
      transformVector(inputFrameId, outputFrameId, Vector(linearAcceleration)));
  return transformedLinearAcceleration;
}

Vector AdapterBase::transformVector(const std::string& inputFrameId,
                                    const std::string& outputFrameId, const Vector& vector) const
{
  Vector transformedVector;
  bool frameError = false;

  if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedVector = vector;
    } else if (outputFrameId == getWorldFrameId()) {
      transformedVector = getOrientationBaseToWorld().rotate(vector);
    } else if (frameIdExists(outputFrameId)) {
      const Vector vectorInOdom = transformVector(inputFrameId, getWorldFrameId(), vector);
      transformedVector = transformVector(getWorldFrameId(), outputFrameId, vectorInOdom);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedVector = getOrientationBaseToWorld().inverseRotate(vector);
    } else if (outputFrameId == getWorldFrameId()) {
      transformedVector = vector;
    } else if (frameIdExists(outputFrameId)) {
      transformedVector = getFrameTransform(outputFrameId).getRotation().rotate(vector);
    } else {
      frameError = true;
    }

  } else if (frameIdExists(inputFrameId)) {

    if (outputFrameId == getBaseFrameId()) {
      const Vector vectorInOdom = transformVector(inputFrameId, getWorldFrameId(), vector);
      transformedVector = transformVector(getWorldFrameId(), outputFrameId, vectorInOdom);
    } else if (outputFrameId == getWorldFrameId()) {
      transformedVector = getFrameTransform(inputFrameId).getRotation().rotate(vector);
    } else {
      frameError = true;
    }

  } else {
    frameError = true;
  }

  if (frameError) {
    const std::string message = "Invalid frame for transforming vector (input frame: " + inputFrameId + ", output frame: " + outputFrameId + ").";
    throw std::invalid_argument(message);
  }
  return transformedVector;
}

} /* namespace free_gait */
