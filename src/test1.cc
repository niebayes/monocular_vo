#include "mono_slam/common_include.h"

int main() {
  cv::FileStorage file("app/config_kitti.yaml", cv::FileStorage::READ);
  double fx = file["fx"]; 
  std::cout << fx << "\n";

  return EXIT_SUCCESS;
}