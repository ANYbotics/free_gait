/*
 * StepFrameConverter.hpp
 *
 *  Created on: Nov 11, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <free_gait_core/free_gait_core.hpp>

// STD
#include <memory>
#include <string>

namespace free_gait {

class StepFrameConverter
{
 public:
  StepFrameConverter(const AdapterBase& adapter);
  virtual ~StepFrameConverter() = default;

  bool adaptCoordinates(StepQueue& stepQueue, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform());

  bool adaptCoordinates(Step& step, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform());

  bool adaptCoordinates(Footstep& footstep, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform());

  bool adaptCoordinates(EndEffectorTrajectory& endEffectorTrajectory, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame = Transform());

  bool adaptCoordinates(BaseTrajectory& baseTrajectory, const std::string& sourceFrameId,
                        const std::string& targetFrameId,
                        const Transform& transformInSourceFrame);

  bool getTransform(const std::string& sourceFrameId, const std::string& targetFrameId,
                    const Transform& transformInSourceFrame,
                    Transform& transform);

 private:
  const AdapterBase& adapter_;
};

} /* namespace free_gait */
