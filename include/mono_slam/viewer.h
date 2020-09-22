#ifndef MONO_SLAM_VIEWER_H_
#define MONO_SLAM_VIEWER_H_

#include "mono_slam/common_include.h"
#include "mono_slam/map.h"
#include "mono_slam/system.h"

namespace mono_slam {

class System;
class Map;

class Viewer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Viewer>;

  Viewer();

  void Reset();

  // Setters to link components.
  void SetMap(const Map::Ptr& map);
  void SetSystem(const sptr<System>& system);

 private:
  Map::Ptr map_ = nullptr;
  sptr<System> system_ = nullptr;
};

void Viewer::Reset() {}

void Viewer::SetMap(const Map::Ptr& map) { map_ = map; }
void Viewer::SetSystem(const sptr<System>& system) { system_ = system; }

}  // namespace mono_slam

#endif  // MONO_SLAM_VIEWER_H_