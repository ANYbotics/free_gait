/*
 * TypeDefs.hpp
 *
 *  Created on: Jun 1, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// anymal model
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/typedefs.hpp>

// STL
#include <unordered_map>
#include <map>


namespace free_gait {

struct EnumClassHash
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

// Import enum aliases.
using AD = anymal_description::AnymalDescription;

using LimbEnum = AD::LimbEnum;
using BranchEnum = AD::BranchEnum;
using JointNodeEnum = AD::JointNodeEnum;
using ContactEnum = AD::ContactEnum;
using FrameTransformEnum = AD::ConcreteTopology::FrameTransformEnum;

// Import kindr aliases.
using Transform = romo::Pose;
using romo::Pose;
using romo::Twist;
using romo::RotationQuaternion;
using romo::AngleAxis;
using romo::RotationMatrix;
using romo::EulerAnglesZyx;
using romo::RotationVector;
using romo::EulerAnglesXyz;
using romo::EulerAnglesXyzDiff;
using romo::Position;
using Position2 = kindr::Position<double, 2>;
using romo::LinearVelocity;
using romo::LocalAngularVelocity;
using romo::EulerAnglesZyxDiff;
using romo::LinearAcceleration;
using romo::AngularAcceleration;
using romo::Force;
using romo::Torque;
using romo::Vector;

// Import robot-specific kindr quantities.
using anymal_model::GeneralizedCoordinates;
using anymal_model::GeneralizedVelocities;
using anymal_model::GeneralizedAccelerations;
using anymal_model::JointPositions;
using JointPositionsLeg = anymal_model::JointPositionsLimb;
using anymal_model::JointVelocities;
using JointVelocitiesLeg = anymal_model::JointVelocitiesLimb;
using anymal_model::JointAccelerations;
using JointAccelerationsLeg = anymal_model::JointAccelerationsLimb;
using JointEfforts = anymal_model::JointTorques;
using JointEffortsLeg = anymal_model::JointTorquesLimb;

enum class ControlLevel
{
  Position,
  Velocity,
  Acceleration,
  Effort
};

const std::vector<LimbEnum> limbEnumCounterClockWiseOrder = { LimbEnum::LF_LEG,
                                                              LimbEnum::LH_LEG,
                                                              LimbEnum::RH_LEG,
                                                              LimbEnum::RF_LEG };

typedef std::unordered_map<ControlLevel, bool, EnumClassHash> ControlSetup;
typedef std::unordered_map<LimbEnum, Position, EnumClassHash> Stance;
typedef std::unordered_map<LimbEnum, Position2, EnumClassHash> PlanarStance;

struct CompareByCounterClockwiseOrder;
void getFootholdsCounterClockwiseOrdered(const Stance& stance, std::vector<Position>& footholds);

} // namespace
