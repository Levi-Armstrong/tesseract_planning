/**
 * @file trajopt_ifopt_motion_planner_task.cpp
 * @brief TrajOpt Ifopt motion planner task
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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

#include <tesseract_task_composer/nodes/trajopt_ifopt_motion_planner_task.h>

namespace tesseract_planning
{
TrajOptIfoptMotionPlannerTask::TrajOptIfoptMotionPlannerTask(std::string input_key,
                                                             std::string output_key,
                                                             bool format_result_as_input,
                                                             bool is_conditional,
                                                             std::string name)
  : TrajOptIfoptMotionPlannerTaskBase(std::move(input_key),
                                      std::move(output_key),
                                      format_result_as_input,
                                      is_conditional,
                                      std::move(name))
{
}

template <class Archive>
void TrajOptIfoptMotionPlannerTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptMotionPlannerTaskBase);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptMotionPlannerTaskBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptMotionPlannerTaskBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptMotionPlannerTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptMotionPlannerTask)
