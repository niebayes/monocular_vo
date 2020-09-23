#include "gflags/gflags.h"
#include "glog/logging.h"
#include "my_slam/slam_system.h"

DEFINE_string(config_file, "app/config_kitti.yaml", "Configuration file.");

int main(int argc, char** argv) {
  GOOGLE_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  if (argc != 1) {
    LOG(INFO) << "Usage: mono_kitti -c=<config_file>";
    LOG(WARNING) << "Use default configuration.";
  }
  System::Ptr system = make_shared<System>(FLAGS_config_file);
  CHECK_EQ(system->Init(), true);
  system->Run();

  return EXIT_SUCCESS;
}