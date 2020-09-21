#ifndef MONO_SLAM_SLAM_SYSTEM_H_
#define MONO_SLAM_SLAM_SYSTEM_H_

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/common_include.h"
#include "mono_slam/dataset.h"
#include "mono_slam/feature_manager.h"
#include "mono_slam/front_end_tracking.h"
#include "mono_slam/map.h"
#include "mono_slam/viewer.h"

namespace mono_slam {

class Tracking;
class LocalMapping;
class Map;
class FeatureManager;
class Viewer;
class Dataset;

class SLAMSystem {
 public:
  SLAMSystem(const std::string& vocabulary_file,
             const std::string& config_file) {
    if (!InitSystem(vocabulary_file, config_file))
      LOG(ERROR) << "Failed to initialize SLAM system";
  }

  bool InitSystem(const std::string& vocabulary_file,
                  const std::string& config_file) {
    return true;
  }

 private:
  Tracking::Ptr tracker_ = nullptr;
  LocalMapping::Ptr local_mapper_ = nullptr;
  Map::Ptr map_ = nullptr;
  FeatureManager::Ptr feat_mgr_ = nullptr;
  Viewer::Ptr viewer_ = nullptr;
  Dataset::Ptr dataset_ = nullptr;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_SLAM_SYSTEM_H_