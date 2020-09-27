#include "mono_slam/keyframe_database.h"

namespace mono_slam {

KeyframeDB::KeyframeDB(const sptr<Vocabulary>& voc) : voc_(voc) {}

}  // namespace mono_slam
