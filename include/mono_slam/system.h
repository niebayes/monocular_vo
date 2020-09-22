#ifndef MONO_SLAM_SYSTEM_H_
#define MONO_SLAM_SYSTEM_H_

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/config.h"
#include "mono_slam/dataset.h"
#include "mono_slam/front_end_map_initialization.h"
#include "mono_slam/front_end_tracking.h"
#include "mono_slam/map.h"
#include "mono_slam/viewer.h"

namespace mono_slam {

class Tracking;
class LocalMapping;
class Map;
class Viewer;
class Dataset;
class Camera;
class Initializer;

class System {
 public:
  using Ptr = sptr<System>;

  System(const std::string& config_file);

  // Init the system: load user-provided settings and prepare threads.
  bool Init();

  // Run system in an infinite loop.
  void Run();

  // Fetch the next frame from dataset and pass it to tracker.
  bool Step();

  // Reset system.
  void Reset();

 private:
  // Handles.
  Tracking::Ptr tracker_ = nullptr;
  LocalMapping::Ptr local_mapper_ = nullptr;
  Map::Ptr map_ = nullptr;
  Viewer::Ptr viewer_ = nullptr;

  // Objects waiting settings.
  Dataset::Ptr dataset_ = nullptr;
  Camera::Ptr cam_ = nullptr;
  Initializer::Ptr initializer_ = nullptr;
  sptr<Vocabulary> voc_ = nullptr;
  vector<double> timestamps_;

  const string config_file_;
};

System::System(const string& config_file) : config_file_(config_file) {}

bool System::Init() {
  {
  //   // Read settings from configuration file.
  //   if (!Config::SetConfigFile(config_file_)) return false;

  //   // Initialize dataset.
  //   dataset_ = make_shared<Dataset>(Config::Get<string>("dataset_path"));
  //   if (!dataset_->Init()) return false;

  //   // Initialize camera.
  //   const double& fx = Config::Get<double>("fx");
  //   const double& fy = Config::Get<double>("fy");
  //   const double& cx = Config::Get<double>("cx");
  //   const double& cy = Config::Get<double>("cy");
  //   const double& k1 = Config::Get<double>("k1");
  //   const double& k2 = Config::Get<double>("k2");
  //   const double& p1 = Config::Get<double>("p1");
  //   const double& p2 = Config::Get<double>("p2");
  //   const Vec4 dist_coeffs{k1, k2, p1, p2};
  //   cam_ = make_shared<Camera>(fx, fy, cx, cy, dist_coeffs);

  //   // Initialize initializer.
  //   initializer_ =
  //       make_shared<Initializer>(Config::Get<int>("min_num_features_init"),
  //                                Config::Get<int>("min_num_matched_features"),
  //                                Config::Get<int>("min_num_inlier_matches"));

  //   // Load vocabulary.
  //   voc_ = make_shared<Vocabulary>(Config::Get<string>("vocabulary_file"));

  //   // Load timestamps.
  //   const string& timestamp_file = Config::Get<string>("timestamp_file");
  //   arma::mat timestamps_mat;
  //   if (timestamp_file.empty())
  //     LOG(WARNING) << "No timestamp file.";
  //   else
  //     timestamps_mat.load(Config::Get<string>("timestamp_file"),
  //                         arma::file_type::auto_detect, true);
  //   timestamps_ = arma::conv_to<vector<double>>::from(timestamps_mat);
  }

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
  tracker_->SetInitializer(initializer_);
  tracker_->SetVocabulary(voc_);

  local_mapper_->SetSystem(this_system);
  local_mapper_->SetTracker(tracker_);
  local_mapper_->SetMap(map_);
  local_mapper_->SetVocabulary(voc_);

  viewer_->SetSystem(this_system);
  viewer_->SetMap(map_);

  return true;
}

void System::Reset() {
  tracker_->Reset();
  local_mapper_->Reset();
  map_->Clear();
  viewer_->Reset();
}

}  // namespace mono_slam

#endif  // MONO_SLAM_SYSTEM_H_