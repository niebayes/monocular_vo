#ifndef MONO_SLAM_DATASET_H_
#define MONO_SLAM_DATASET_H_

#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/feature_manager.h"

namespace mono_slam {

class Camera;
class FeatureManager;

class Dataset {
 public:
  using Ptr = sptr<Dataset>;

  Dataset(const std::string& dataset_path);

  bool Init();

 private:
 
  // Dataset path.
  std::string dataset_path_;
};

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

bool Dataset::Init() {
  //
}

}  // namespace mono_slam

#endif  // MONO_SLAM_DATASET_H_