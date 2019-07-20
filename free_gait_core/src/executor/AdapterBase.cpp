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
  Position transformedPosition;
  bool frameError = false;

  if (inputFrameId == outputFrameId) {
    return position;
  }

  if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedPosition = getPositionWorldToBaseInWorldFrame() + getOrientationBaseToWorld().rotate(position);
    } else if (frameIdExists(outputFrameId)) {
      const Position positionInWorld = getPositionWorldToBaseInWorldFrame() + getOrientationBaseToWorld().rotate(position);
      transformedPosition = getFrameTransform(outputFrameId).inverseTransform(positionInWorld);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedPosition = getOrientationBaseToWorld().inverseRotate(position - getPositionWorldToBaseInWorldFrame());
    } else if (frameIdExists(outputFrameId)) {
      transformedPosition = getFrameTransform(outputFrameId).inverseTransform(position);
    } else {
      frameError = true;
    }

  } else if (frameIdExists(inputFrameId)) {

    if (outputFrameId == getBaseFrameId()) {
      const Position positionInWorld = getFrameTransform(inputFrameId).transform(position);
      transformedPosition = getOrientationBaseToWorld().inverseRotate(positionInWorld - getPositionWorldToBaseInWorldFrame());
    } else if (outputFrameId == getWorldFrameId()) {
      transformedPosition = getFrameTransform(inputFrameId).transform(position);
    } else if (frameIdExists(outputFrameId)) {
        const Position positionInWorld = getFrameTransform(inputFrameId).transform(position);
        transformedPosition = getFrameTransform(outputFrameId).inverseTransform(positionInWorld);
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

  if (inputFrameId == outputFrameId) {
    return orientation;
  }

  if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedOrientation = getOrientationBaseToWorld().inverted() * orientation ;
    } else if (frameIdExists(outputFrameId)) {
      transformedOrientation = getFrameTransform(outputFrameId).getRotation().inverted() * orientation;
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedOrientation = getOrientationBaseToWorld() * orientation;
    } else if (frameIdExists(outputFrameId)) {
      const RotationQuaternion orientationToWorld = getOrientationBaseToWorld() * orientation;
      transformedOrientation = getFrameTransform(outputFrameId).getRotation().inverted() * orientationToWorld;
    } else {
      frameError = true;
    }

  } else if (frameIdExists(inputFrameId)) {

    if (outputFrameId == getWorldFrameId()) {
      transformedOrientation = getFrameTransform(inputFrameId).getRotation() * orientation;
    } else if (outputFrameId == getBaseFrameId()) {
      const RotationQuaternion orientationToWorld = getOrientationBaseToWorld() * orientation;
      transformedOrientation =  getFrameTransform(inputFrameId).getRotation().inverted() * orientationToWorld;
    } else if (frameIdExists(outputFrameId)) {
      const RotationQuaternion orientationToWorld = getFrameTransform(inputFrameId).getRotation() * orientation;
      transformedOrientation = getFrameTransform(outputFrameId).getRotation().inverted() * orientationToWorld;
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

  if (inputFrameId == outputFrameId) {
    return vector;
  }

  if (inputFrameId == getBaseFrameId()) {

    if (outputFrameId == getWorldFrameId()) {
      transformedVector = getOrientationBaseToWorld().rotate(vector);
    } else if (frameIdExists(outputFrameId)) {
      const Vector vectorInWorld = getOrientationBaseToWorld().rotate(vector);
      transformedVector = getFrameTransform(outputFrameId).getRotation().inverseRotate(vectorInWorld);
    } else {
      frameError = true;
    }

  } else if (inputFrameId == getWorldFrameId()) {

    if (outputFrameId == getBaseFrameId()) {
      transformedVector = getOrientationBaseToWorld().inverseRotate(vector);
    } else if (frameIdExists(outputFrameId)) {
      transformedVector = getFrameTransform(outputFrameId).getRotation().inverseRotate(vector);
    } else {
      frameError = true;
    }

  } else if (frameIdExists(inputFrameId)) {

    if (outputFrameId == getBaseFrameId()) {
      const Vector vectorInWorld = getFrameTransform(inputFrameId).getRotation().rotate(vector);
      transformedVector = getOrientationBaseToWorld().inverseRotate(vectorInWorld);
    } else if (outputFrameId == getWorldFrameId()) {
      transformedVector = getFrameTransform(inputFrameId).getRotation().rotate(vector);
    } else if (frameIdExists(outputFrameId)) {
      const Vector vectorInWorld = getFrameTransform(inputFrameId).getRotation().rotate(vector);
      transformedVector = getFrameTransform(outputFrameId).getRotation().inverseRotate(vectorInWorld);
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
