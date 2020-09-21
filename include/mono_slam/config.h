#ifndef MONO_SLAM_CONFIG_H_
#define MONO_SLAM_CONFIG_H_

#include "mono_slam/common_include.h"

class Config {
 private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;

  // Private constructor preventing instantiation to make a singleton (i.e. no
  // objects can be created).
  Config() {}

 public:
  // Making destructor public to make the error message more friendly.
  // Safely destruct the singleton.
  ~Config();

  // Read a configuration file.
  static bool ReadConfigFile(const std::string& config_file);

  // Access the settings: get setting by key.
  template <typename T>
  static T Get(const std::string& key);
};

#endif  // MONO_SLAM_CONFIG_H_
