#include "mono_slam/dataset.h"

namespace mono_slam {

Dataset::Dataset(const string& dataset_path, const string& img_file_name_fmt,
                 const double img_resize_factor, const int img_start_idx)
    : dataset_path_(dataset_path),
      img_file_name_fmt_(img_file_name_fmt),
      img_resize_factor_(img_resize_factor),
      img_idx_(img_start_idx) {}

cv::Mat Dataset::nextImage() {
  boost::format fmt(dataset_path_ + "%" + img_file_name_fmt_);
  cv::Mat image = cv::imread((fmt % img_idx_).str(), cv::IMREAD_GRAYSCALE);
  // Error in reading a single image should not interrupt the system.
  if (image.empty()) return cv::Mat{};
  cv::Mat resized_image;
  image.copyTo(resized_image);
  if (img_resize_factor_ != 1.0)
    cv::resize(image, resized_image, {}, img_resize_factor_, img_resize_factor_,
               cv::INTER_AREA);
  ++img_idx_;
  return resized_image;
}

}  // namespace mono_slam