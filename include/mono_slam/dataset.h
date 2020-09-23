#ifndef MONO_SLAM_DATASET_H_
#define MONO_SLAM_DATASET_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

class Frame;

class Dataset {
 public:
  using Ptr = uptr<Dataset>;

  Dataset(const string& dataset_path, const string& image_file_name_fmt);

  const cv::Mat& NextImage();

 private:
  // Current image index.
  int img_idx_;
  // Dataset path.
  const string dataset_path_;
  // Image file name format.
  const string image_file_name_fmt_;
  // Image resize factor. A bigger one speeds up tracking.
  const double resize_factor_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_DATASET_H_