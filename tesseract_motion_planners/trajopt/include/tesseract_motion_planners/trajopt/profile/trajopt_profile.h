/**
 * @file trajopt_profile.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/fwd.hpp>
#include <vector>
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_command_language/fwd.h>
#include <tesseract_command_language/profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_solver_config.h>

namespace tesseract_planning
{
struct TrajOptCostAndConstraintInfo
{
  std::vector<std::shared_ptr<trajopt::TermInfo>> cost_infos;
  std::vector<std::shared_ptr<trajopt::TermInfo>> cnt_infos;
};

class TrajOptPlanProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptPlanProfile>;

  TrajOptPlanProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  virtual TrajOptCostAndConstraintInfo createCostAndConstraintInfo(const MoveInstructionPoly& parent_instruction,
                                                                   const tesseract_common::ManipulatorInfo& manip_info,
                                                                   const std::shared_ptr<const tesseract_environment::Environment>& env,
                                                                   const std::vector<std::string>& active_links,
                                                                   int index) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

class TrajOptCompositeProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptCompositeProfile>;

  TrajOptCompositeProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  virtual void apply(trajopt::ProblemConstructionInfo& pci,
                     int start_index,
                     int end_index,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     const std::vector<int>& fixed_indices) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

class TrajOptSolverProfile : public Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptSolverProfile>;

  TrajOptSolverProfile();

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  virtual TrajOptSolverConfig createSolverConfig() const = 0;

  virtual std::vector<sco::Optimizer::Callback> createSolverCallbacks() const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptPlanProfile)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptCompositeProfile)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptSolverProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
