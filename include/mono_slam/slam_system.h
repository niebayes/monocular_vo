#ifndef MONO_SLAM_SLAM_SYSTEM_H_
#define MONO_SLAM_SLAM_SYSTEM_H_

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/config.h"
#include "mono_slam/dataset.h"
#include "mono_slam/feature_manager.h"
#include "mono_slam/front_end_map_initialization.h"
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
class Camera;
class Initializer;

class SLAMSystem {
 public:
  SLAMSystem(const std::string& config_file);

  bool Init();

  void Run();

  bool Step();

 private:
  Tracking::Ptr tracker_ = nullptr;
  LocalMapping::Ptr local_mapper_ = nullptr;
  Map::Ptr map_ = nullptr;
  FeatureManager::Ptr feat_mgr_ = nullptr;
  Viewer::Ptr viewer_ = nullptr;
  Dataset::Ptr dataset_ = nullptr;
  Camera::Ptr cam_ = nullptr;
  Initializer::Ptr initializer_ = nullptr;

  const string config_file_;
};

SLAMSystem::SLAMSystem(const std::string& config_file)
    : config_file_(config_file) {}

bool SLAMSystem::Init() {
  // Read settings from configuration file.
  if (!Config::SetConfigFile(config_file_)) return false;

  // Initialize dataset.
  dataset_ =
      Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_path")));
  if (!dataset_->Init()) return false;

  // Initialize camera.
  cam_ = Camera::Ptr(
      make_shared<Camera>(Config::Get<double>("fx"), Config::Get<double>("fy"),
                          Config::Get<double>("cx"), Config::Get<double>("cy"),
                          Vec4{
                              Config::Get<double>("k1"),
                              Config::Get<double>("k2"),
                              Config::Get<double>("p1"),
                              Config::Get<double>("p2"),
                          }));

  // Initialize feature manager.
  feat_mgr_ = FeatureManager::Ptr

  // Initialize initializer.
  initializer_ = Initializer::Ptr(
      make_shared<Initializer>(Config::Get<int>("min_num_features_init"),
                               Config::Get<int>("min_num_matched_features"),
                               Config::Get<int>("min_num_inlier_matches")));

  return true;
}

}  // namespace mono_slam

#endif  // MONO_SLAM_SLAM_SYSTEM_H_