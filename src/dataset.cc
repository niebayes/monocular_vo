#include "mono_slam/dataset.h"

namespace mono_slam {

Dataset::Dataset(const string& dataset_path, const string& image_file_name_fmt,
                 const double resize_factor)
    : dataset_path_(dataset_path),
      image_file_name_fmt_(image_file_name_fmt),
      resize_factor_(resize_factor),
      img_idx_(0) {}

cv::Mat Dataset::NextImage() {
  boost::format fmt(dataset_path_ + image_file_name_fmt_);
  cv::Mat image = cv::imread((fmt % img_idx_).str(), cv::IMREAD_GRAYSCALE);
  // Error in reading a single image should not interrupt the system.
  if (image.empty()) return cv::Mat{};
  cv::Mat resized_image;
  image.copyTo(resized_image);
  if (resize_factor_ != 1.0)
    cv::resize(image, resized_image, {}, resize_factor_, resize_factor_,
               cv::INTER_AREA);
  img_idx_++;
  return resized_image;
}

}  // namespace mono_slam