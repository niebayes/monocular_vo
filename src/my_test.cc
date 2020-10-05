#include "glog/logging.h"
#include "mono_slam/common_include.h"

void Triangulate(const Vec2& kp1, const Vec2& kp2, const cv::Mat& P1,
                 const cv::Mat& P2, cv::Mat& x3D) {
  cv::Mat A(4, 4, CV_32F);

  A.row(0) = kp1.x() * P1.row(2) - P1.row(0);
  A.row(1) = kp1.y() * P1.row(2) - P1.row(1);
  A.row(2) = kp2.x() * P2.row(2) - P2.row(0);
  A.row(3) = kp2.y() * P2.row(2) - P2.row(1);
  cout << "cv: " << A << '\n';

  cv::Mat u, w, vt;
  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  x3D = vt.row(3).t();
  x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
}

Mat33 to_skew(const Vec3& vec) {
  Mat33 skew_mat;
  skew_mat << 0., -vec(2), vec(1), vec(2), 0., -vec(0), -vec(1), vec(0), 0.;
  return skew_mat;
}

void triangulateLin(const Vec2& pt_1, const Vec2& pt_2, const Mat34& M_1,
                    const Mat34& M_2, Vec3& point) {
  //! A could be [6 x 4] or [4 x 4].
  MatXX A(6, 4);
  cout << pt_1 << '\n';
  cout << to_skew(pt_1.homogeneous()) << '\n';
  A.topRows(3) = to_skew(pt_1.homogeneous()) * M_1;
  A.bottomRows(3) = to_skew(pt_2.homogeneous()) * M_2;
  cout << "eigen: " << A << '\n';

  // Compute 3D points using SVD.
  auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  point = svd.matrixV().rightCols(1).colwise().hnormalized();
}

int main() {
  Vec2 pt_1, pt_2;
  pt_1 << 494, 190;
  pt_2 << 495, 192;
  Mat34 M_1, M_2;
  M_1 << 718.856, 0, 607.193, 0, 0, 718.856, 185.216, 0, 0, 0, 1, 0;
  M_2 << 726.27, 8.77776, 598.24, -651.325, 0.122743, 720.859, 177.262,
      -352.608, 0.0123058, 0.0110659, 0.999863, -0.0357869;
  cv::Mat M1, M2;
  cv::eigen2cv(M_1, M1);
  cv::eigen2cv(M_2, M2);
  Vec3 point_e;
  cv::Mat point_cv;
  triangulateLin(pt_1, pt_2, M_1, M_2, point_e);
  Triangulate(pt_1, pt_2, M1, M2, point_cv);
  cout << "eigen: " << point_e << '\n';
  cout << "cv: " << point_cv << '\n';

  return EXIT_SUCCESS;
}