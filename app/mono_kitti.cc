#include "gflags/gflags.h"
#include "glog/logging.h"
#include "mono_slam/system.h"

using namespace mono_slam;

DEFINE_string(config_file, "app/config_kitti.yaml", "Configuration file.");


int main(int argc, char** argv) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  if (argc != 1) {
    LOG(INFO) << "Usage: mono_kitti -c=<config_file>";
    LOG(WARNING) << "Use default configuration.";
  }
  Tracking::Ptr tracker = make_shared<Tracking>();
  System::Ptr system = make_shared<System>(FLAGS_config_file);
  CHECK_EQ(system->Init(), true);
  system->Run();

  return EXIT_SUCCESS;
}