#include "mono_slam/common_include.h"
#include "glog/logging.h"

int main() {
  LOG(INFO) << "test";
  string ko{"fx = "};
  cv::FileStorage fin("app/kitti.yaml", cv::FileStorage::READ);
  double fx = fin["Camera.fx"];
  cout << ko << fx << '\n';

  return EXIT_SUCCESS;
}