#include "mono_slam/viewer.h"

namespace mono_slam {

Viewer::Viewer() {}

void Viewer::reset() {}

void Viewer::setMap(sptr<Map> map) { map_ = map; }

void Viewer::setSystem(sptr<System> system) { system_ = system; }

}  // namespace mono_slam
