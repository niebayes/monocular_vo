#ifndef MONO_SLAM_CONFIG_H_
#define MONO_SLAM_CONFIG_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

// Implement the Singleton design pattern to allow global access and to ensure
// that only one instance exists.
class Config {
 private:
  static sptr<Config> config_;
  cv::FileStorage file_;

  // Private constructor preventing instantiation to make a singleton (i.e. no
  // objects can be created).
  Config();

 public:
  // Making destructor public to make the error message more friendly.
  // Safely destruct the singleton.
  ~Config();

  // Set the configuration file.
  static bool setConfigFile(const string& config_file);

  // Generic getter.
  template <typename T>
  static T get(const string& key);
};

}  // namespace mono_slam

#endif  // MONO_SLAM_CONFIG_H_
