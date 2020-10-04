#include "mono_slam/system.h"

#include "mono_slam/local_mapping.h"
#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/config.h"
#include "mono_slam/dataset.h"
#include "mono_slam/initialization.h"
#include "mono_slam/tracking.h"
#include "mono_slam/map.h"
#include "mono_slam/matcher.h"
#include "mono_slam/viewer.h"

namespace mono_slam {

System::System(const string& config_file) : config_file_(config_file) {}

bool System::init() {
  LOG(INFO) << "System is initializing ...";
  // Read settings from configuration file.
  cv::FileStorage config(config_file_, cv::FileStorage::READ);
  if (!config.isOpened()) {
    config.release();
    LOG(FATAL) << "Unable to read configuration file.";
  }

  // Prepare dataset.
  dataset_ = make_unique<Dataset>(
      config["dataset_path"], config["img_file_name_fmt"],
      config["img_resize_factor"], config["img_start_idx"]);

  // Load vocabulary.
  sptr<Vocabulary> voc = make_shared<Vocabulary>(config["voc_file"]);
  
  // Load timestamps.
  const string& timestamp_file = config["timestamp_file"];
  arma::mat timestamps_mat;
  if (timestamp_file.empty())
    LOG(WARNING) << "No timestamp file.";
  else 
    timestamps_mat.load(timestamp_file, arma::file_type::auto_detect, true);
  timestamps_ = arma::conv_to<vector<double>>::from(timestamps_mat);


  // Set camera parameters.
  const double& fx = config["fx"];
  const double& fy = config["fy"];
  const double& cx = config["cx"];
  const double& cy = config["cy"];
  const double& k1 = config["k1"];
  const double& k2 = config["k2"];
  const double& p1 = config["p1"];
  const double& p2 = config["p2"];
  const Vec4 dist_coeffs{k1, k2, p1, p2};
  uptr<Camera> cam = make_unique<Camera>();
  cam->init(fx, fy, cx, cy, dist_coeffs);

  // Release the file as soon as possible.
  config.release();

  // Prepare and link components.
  tracker_ = make_shared<Tracking>();
  local_mapper_ = make_shared<LocalMapping>();
  map_ = make_shared<Map>();
  viewer_ = make_shared<Viewer>();

  tracker_->setSystem(shared_from_this());
  tracker_->setLocalMapper(local_mapper_);
  tracker_->setMap(map_);
  tracker_->setViewer(viewer_);
  tracker_->setInitializer(std::move(initializer));
  tracker_->setVocabulary(voc);
  tracker_->setCamera(cam.get());

  local_mapper_->setSystem(shared_from_this());
  local_mapper_->setTracker(tracker_);
  local_mapper_->setMap(map_);
  local_mapper_->setVocabulary(voc);

  viewer_->setSystem(shared_from_this());
  viewer_->setMap(map_);

  return true;
}

void System::run() {
  LOG(INFO) << "System is running ...";
  if (timestamps_.empty())
    for (;;) tracker_->addImage(dataset_->nextImage());
  else {
    const int num_images = timestamps_.size();
    // Simply discard the last image.
    for (int i = 0; i < num_images - 1; ++i) {
      const steady_clock::time_point t1 = steady_clock::now();

      // Track one image.
      tracker_->addImage(dataset_->nextImage());

      const steady_clock::time_point t2 = steady_clock::now();
      const double consumed_time =
          duration_cast<duration<double>>(t2 - t1).count();

      // (Only) pause the tracking thread to align the time.
      const double delta_t = timestamps_[i + 1] - timestamps_[i];
      if (consumed_time < delta_t)
        std::this_thread::sleep_for(duration<double>(delta_t - consumed_time));
    }
  }
  LOG(INFO) << "Exit system.";
}

void System::reset() {
  tracker_->reset();
  local_mapper_->reset();
  map_->clear();
  viewer_->reset();
}

}  // namespace mono_slam
