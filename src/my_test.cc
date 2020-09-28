#include "mono_slam/common_include.h"

int main() {
  string ko{"fx = "};
  cv::FileStorage fin("app/config_kitti.yaml", cv::FileStorage::READ);
  double fx = fin["fx"];
  cout << ko << fx << '\n';
  return EXIT_SUCCESS;
}