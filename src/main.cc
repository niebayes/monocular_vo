#include "my_slam/back_end_local_mapping.h"
#include "my_slam/common_include.h"
#include "my_slam/front_end_tracking.h"
#include "my_slam/keyframe_database.h"

class Tracking;
class LocalMapping;
class KeyframeDB;

class SLAMSystem {
 public:
  SLAMSystem(const std::string& vocabulary_file,
             const std::string& config_file) {
    if (InitSystem(vocabulary_file, config_file))
      LOG(ERROR) << "Failed to initialize SLAM system";
  }

  bool InitSystem(const std::string& vocabulary_file,
                  const std::string& config_file) {
    voc_ = DBoW3::Vocabulary(vocabulary_file);
    if (voc_.empty()) return false;
    cv::FileStorage f_config(config_file, cv::FileStorage::READ);
    if (!f_config.isOpened()) return false;

    tracker_ = Tracking::Ptr(new Tracking);

    return true;
  }

 private:
  Tracking::Ptr tracker_ = nullptr;
  LocalMapping::Ptr local_mapper_ = nullptr;
  DBoW3::Vocabulary voc_;
  KeyframeDB::Ptr keyframe_db = nullptr;
};

int main(int argc, char** argv) {
  // CLI arguments:
  // image_file, timestamp_file, config_file, vocabulary_file

  // Create SLAM system
  // Pass in config_file, vocabulary_file
  // Ready all threads

  // Main loop
  // Pass images frame by frame along with the timestamp to Tracking thread.
  // Be careful the delay of timestamp.

  // Shutdown system.

  // Save trajectory and all map points.

  return EXIT_SUCCESS;
}