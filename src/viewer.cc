#include "mono_slam/viewer.h"

namespace mono_slam {

void Viewer::Reset() {}

void Viewer::SetMap(const Map::Ptr& map) { map_ = map; }

void Viewer::SetSystem(const sptr<System>& system) { system_ = system; }

}  // namespace mono_slam
