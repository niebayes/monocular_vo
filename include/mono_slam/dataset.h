#ifndef MONO_SLAM_DATASET_H_
#define MONO_SLAM_DATASET_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

class Dataset {
 public:
  Dataset(const string& dataset_path, const string& img_file_name_fmt,
          const double img_resize_factor, const int img_start_idx);

  cv::Mat nextImage();

 private:
  // Current image index.
  int img_idx_;
  // Dataset path.
  const string dataset_path_;
  // Image file name format.
  const string img_file_name_fmt_;
  // Image resize factor. A bigger one speeds up tracking.
  const double img_resize_factor_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_DATASET_H_