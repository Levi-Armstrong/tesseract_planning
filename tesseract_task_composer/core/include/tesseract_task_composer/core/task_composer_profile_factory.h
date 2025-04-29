/**
 * @file profile.h
 * @brief This is a profile base class
 *
 * @author Levi Armstrong
 * @date April 24, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_PROFILE_FACTORY_H
#define TESSERACT_COMMON_PROFILE_FACTORY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <map>
#include <memory>
#include <set>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <boost_plugin_loader/fwd.h>
#include <boost_plugin_loader/macros.h>
#include <filesystem>

// clang-format off
#define TESSERACT_ADD_PROFILE_PLUGIN(DERIVED_CLASS, ALIAS)                                                    \
    EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, Profile)
// clang-format on

namespace YAML
{
  class Node;
}

namespace tesseract_planning
{

/** @brief Profile Factory class used for loading profiles */
class TaskComposerProfileFactory
{
public:
  using Ptr = std::shared_ptr<TaskComposerProfileFactory>;
  using ConstPtr = std::shared_ptr<const TaskComposerProfileFactory>;

  virtual ~TaskComposerProfileFactory() = default;

  virtual std::unique_ptr<tesseract_common::Profile> create(const std::string& name,
                                                            const YAML::Node& config,
                                                            const TaskComposerProfileFactory& plugin_factory) const = 0;

protected:
  static std::string getSection();
  friend class boost_plugin_loader::PluginLoader;
};

class TaskComposerProfilePluginFactory
{
public:
  TaskComposerProfilePluginFactory();
  ~TaskComposerProfilePluginFactory();
  TaskComposerProfilePluginFactory(const TaskComposerProfilePluginFactory&) = delete;
  TaskComposerProfilePluginFactory& operator=(const TaskComposerProfilePluginFactory&) = delete;
  TaskComposerProfilePluginFactory(TaskComposerProfilePluginFactory&&) noexcept;
  TaskComposerProfilePluginFactory& operator=(TaskComposerProfilePluginFactory&&) noexcept;

  /**
   * @brief Load plugins from a configuration object
   * @param config The config object
   */
  TaskComposerProfilePluginFactory(const tesseract_common::ProfilePluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  TaskComposerProfilePluginFactory(const YAML::Node& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  TaskComposerProfilePluginFactory(const std::filesystem::path& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  TaskComposerProfilePluginFactory(const std::string& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Loads plugins from a configuration object
   * @param config the config object
   */
  void loadConfig(const tesseract_common::ProfilePluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  void loadConfig(const YAML::Node& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  void loadConfig(const std::filesystem::path& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  void loadConfig(const std::string& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Add location for the plugin loader to search
   * @param path The full path to the directory
   */
  void addSearchPath(const std::string& path);

  /**
   * @brief Get the plugin search paths
   * @return The search paths
   */
  std::set<std::string> getSearchPaths() const;

  /**
   * @brief Clear the search paths
   *
   */
  void clearSearchPaths();

  /**
   * @brief Add a library to search for plugin name
   * @param library_name The library name without the prefix or suffix
   */
  void addSearchLibrary(const std::string& library_name);

  /**
   * @brief Get the plugin search libraries
   * @return The search libraries
   */
  std::set<std::string> getSearchLibraries() const;

  /**
   * @brief Clean the search libraries
   */
  void clearSearchLibraries();

  /**
   * @brief Add a forward kinematics plugin to the manager
   * @details Task can have multiple profiles. Forexample the motion planning task has move, composite and solver profiles. The section name allows for this distinction when defining profiles.
   * @param task_ns The task namespace
   * @param section_name The task section namespace
   * @param profile_name The profile name
   * @param plugin_info The plugin information
   */
  void addProfilePlugin(const std::string& task_ns,
                        const std::string& section_name,
                        const std::string& profile_name,
                        tesseract_common::PluginInfo plugin_info);

  /**
   * @brief Get the map of profile plugins
   * @return A map of plugins
   */
  std::map<std::string, std::map<std::string, tesseract_common::PluginInfoContainer>> getProfilePlugins() const;

  /**
   * @brief remove profile plugin from the manager
   * @param task_ns The task namespace
   * @param section_name The task section namespace
   * @param profile_name The profile name
   */
  void removeProfilePlugin(const std::string& task_ns, const std::string& section_name, const std::string& profile_name);

  /**
   * @brief Set a task namespace default profile
   * @param task_ns The task namespace
   * @param section_name The task section namespace
   * @param profile_name The profile name
   */
  void setDefaultProfilePlugin(const std::string& task_ns, const std::string& section_name, const std::string& profile_name);

  /**
   * @brief Get the default profile name for task section
   * @param task_ns The task namespace
   * @param section_name The task section namespace
   * @return The default profile name
   */
  std::string getDefaultProfilePlugin(const std::string& task_ns, const std::string& section_name) const;

  /**
   * @brief Create profile from stored plugin information
   * @param task_ns The task namespace
   * @param section_name The task section namespace
   * @param profile_name The profile name
   * @return The profile if it exists, otherwise nullptr
   */
  std::unique_ptr<tesseract_common::Profile> createProfile(const std::string& task_ns, const std::string& section_name, const std::string& profile_name) const;

  /**
   * @brief Create profile using provided plugin information
   * @param task_ns The task namespace
   * @param section_name The task section namespace
   * @param profile_name The profile name
   * @param plugin_info The plugin information to create profile
   * @return The profile if it exists, otherwise nullptr
   */
  std::unique_ptr<tesseract_common::Profile>
  createProfile(const std::string& task_ns, const std::string& section_name, const std::string& profile_name, const tesseract_common::PluginInfo& plugin_info) const;

  /**
   * @brief Save the plugin information to a yaml config file
   * @param file_path The file path
   */
  void saveConfig(const std::filesystem::path& file_path) const;

  /**
   * @brief Get the plugin information config as a yaml node
   * @return The plugin information config yaml node/
   */
  YAML::Node getConfig() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> impl_;

  void loadConfig(const YAML::Node& config);
};
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMON_PROFILE_FACTORY_H
