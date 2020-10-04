#include "mono_slam/viewer.h"

namespace mono_slam {

void Viewer::reset() {}

void Viewer::setMap(const Map::Ptr& map) { map_ = map; }

void Viewer::setSystem(const sptr<System>& system) { system_ = system; }

}  // namespace mono_slam
