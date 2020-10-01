#ifndef MONO_SLAM_SYSTEM_H_
#define MONO_SLAM_SYSTEM_H_

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/config.h"
#include "mono_slam/dataset.h"
#include "mono_slam/front_end_map_initialization.h"
#include "mono_slam/front_end_tracking.h"
#include "mono_slam/map.h"
#include "mono_slam/viewer.h"

using namespace std::chrono;

namespace mono_slam {

class Tracking;
class LocalMapping;
class Map;
class Viewer;
class Dataset;
class Camera;
class Initializer;

class System {
 public:
  using Ptr = sptr<System>;

  System(const string& config_file);

  // Init the system: load user-provided settings and link system components.
  bool init();

  // Run system in an infinite loop till the dataset is exhausted.
  void run();

  // Reset system.
  void reset();

 private:
  // System components.
  sptr<Tracking> tracker_ = nullptr;
  sptr<LocalMapping> local_mapper_ = nullptr;
  Map::Ptr map_ = nullptr;
  sptr<Viewer> viewer_ = nullptr;

  // User-specified objects.
  Dataset::Ptr dataset_ = nullptr;
  vector<double> timestamps_;
  const string config_file_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_SYSTEM_H_