#include "gflags/gflags.h"
#include "glog/logging.h"
#include "mono_slam/system.h"

using namespace mono_slam;

DEFINE_string(c, "app/config_kitti.yaml", "Configuration file.");

int main(int argc, char** argv) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  if (argc != 2) {
    LOG(INFO) << "Usage: mono_kitti -c=<config_file>";
    LOG(WARNING) << "Use default configuration.";
  }
  System::Ptr system = make_shared<System>(FLAGS_c);
  CHECK_EQ(system->init(), true);
  system->run();

  return EXIT_SUCCESS;
}