#include "mono_slam/system.h"

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/config.h"
#include "mono_slam/dataset.h"
#include "mono_slam/front_end_map_initialization.h"
#include "mono_slam/front_end_tracking.h"
#include "mono_slam/map.h"
#include "mono_slam/matcher.h"
#include "mono_slam/viewer.h"

namespace mono_slam {

System::System(const string& config_file) : config_file_(config_file) {}

bool System::Init() {
  LOG(INFO) << "System is initializing ...";
  // Read settings from configuration file.
  if (!Config::SetConfigFile(config_file_)) return false;

  // Prepare dataset.
  dataset_ = make_unique<Dataset>(Config::Get<string>("dataset_path"),
                                  Config::Get<string>("image_file_name_fmt"),
                                  Config::Get<double>("resize_factor"));

  // Load timestamps.
  const string& timestamp_file = Config::Get<string>("timestamp_file");
  arma::mat timestamps_mat;
  if (timestamp_file.empty())
    LOG(WARNING) << "No timestamp file.";
  else
    timestamps_mat.load(Config::Get<string>("timestamp_file"),
                        arma::file_type::auto_detect, true);
  timestamps_ = arma::conv_to<vector<double>>::from(timestamps_mat);

  // Load settings for tracker.
  // Set vocabulary.
  sptr<Vocabulary> voc =
      make_shared<Vocabulary>(Config::Get<string>("vocabulary_file"));

  // Set initializer parameters.
  uptr<Initializer> initializer =
      make_unique<Initializer>(Config::Get<int>("min_num_features_init"),
                               Config::Get<int>("min_num_matched_features"),
                               Config::Get<int>("min_num_inlier_matches"));

  // Set camera parameters.
  const double& fx = Config::Get<double>("fx");
  const double& fy = Config::Get<double>("fy");
  const double& cx = Config::Get<double>("cx");
  const double& cy = Config::Get<double>("cy");
  const double& k1 = Config::Get<double>("k1");
  const double& k2 = Config::Get<double>("k2");
  const double& p1 = Config::Get<double>("p1");
  const double& p2 = Config::Get<double>("p2");
  const Vec4 dist_coeffs{k1, k2, p1, p2};
  Camera::Ptr cam = make_unique<Camera>();
  cam->Init(fx, fy, cx, cy, dist_coeffs);

  // Set feature detector parameters.
  const int& desired_num_features = Config::Get<int>("desired_num_features");
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(desired_num_features);

  // Set matcher parameters.
  Matcher matcher(Config::Get<double>("matching_threshold"),
                  Config::Get<double>("distance_ratio_test_threshold"));

  // Prepare and link components.
  tracker_ = make_shared<Tracking>();
  local_mapper_ = make_shared<LocalMapping>();
  map_ = make_shared<Map>();
  viewer_ = make_shared<Viewer>();

  System::Ptr this_system(this);

  tracker_->SetSystem(this_system);
  tracker_->SetLocalMapper(local_mapper_);
  tracker_->SetMap(map_);
  tracker_->SetViewer(viewer_);
  tracker_->SetInitializer(std::move(initializer));
  tracker_->SetVocabulary(voc);
  tracker_->SetCamera(std::move(cam));
  tracker_->SetFeatureDetector(detector);

  local_mapper_->SetSystem(this_system);
  local_mapper_->SetTracker(tracker_);
  local_mapper_->SetMap(map_);
  local_mapper_->SetVocabulary(voc);

  viewer_->SetSystem(this_system);
  viewer_->SetMap(map_);

  return true;
}

void System::Run() {
  LOG(INFO) << "System is running ...";
  if (timestamps_.empty())
    for (;;) tracker_->AddImage(dataset_->NextImage());
  else {
    const int num_images = timestamps_.size();
    // Simply discard the last image.
    for (int i = 0; i < num_images - 1; ++i) {
      const steady_clock::time_point t1 = steady_clock::now();

      // Track one image.
      tracker_->AddImage(dataset_->NextImage());

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

void System::Reset() {
  tracker_->Reset();
  local_mapper_->Reset();
  map_->Clear();
  viewer_->Reset();
}

}  // namespace mono_slam
