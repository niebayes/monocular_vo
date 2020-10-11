#ifndef MONO_SLAM_SYSTEM_H_
#define MONO_SLAM_SYSTEM_H_

#include "mono_slam/common_include.h"
#include "mono_slam/dataset.h"
#include "mono_slam/local_mapping.h"
#include "mono_slam/map.h"
#include "mono_slam/tracking.h"
#include "mono_slam/viewer.h"

namespace mono_slam {

class Tracking;
class LocalMapping;
class Map;
class Viewer;
class Dataset;

class System : public std::enable_shared_from_this<System> {
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
  uptr<Dataset> dataset_ = nullptr;
  vector<double> timestamps_;
  const string config_file_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_SYSTEM_H_