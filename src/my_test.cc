#include "mono_slam/common_include.h"
#include "mono_slam/system.h"

using namespace mono_slam;

int main() {
  std::string ko{"fx = "};
  cv::FileStorage fin("app/config_kitti.yaml", cv::FileStorage::READ);
  double fx = fin["fx"];
  std::cout << ko << fx << '\n';
  return EXIT_SUCCESS;
}