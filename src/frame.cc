#include "mono_slam/frame.h"

#include "mono_slam/feature.h"
#include "utils/math_utils.h"

namespace mono_slam {

Frame::Frame(const cv::Mat& img, Camera::Ptr cam, const sptr<Vocabulary>& voc,
             const cv::Ptr<cv::FeatureDetector>& detector)
    : id_(frame_cnt_++), is_keyframe_(false), cam_(std::move(cam)) {
  extractFeatures(img, detector);
  computeBoW(voc);
  // TODO(bayes) Optimize when no distortion.
  // Compute image bounds (computed once in the first frame).
  if (id_ == 0) {
    // Matrix containing the four corners of the image:
    // Left upper, right upper, left bottom, right bottom.
    cv::Mat corners;
    frame_utils::computeImageBounds(img, cam_->K(), cam_->distCoeffs(),
                                    corners);
    x_min_ = std::min(corners.at<float>(0, 0), corners.at<float>(2, 0));
    x_max_ = std::max(corners.at<float>(1, 0), corners.at<float>(3, 0));
    y_min_ = std::min(corners.at<float>(0, 1), corners.at<float>(1, 1));
    y_max_ = std::max(corners.at<float>(2, 1), corners.at<float>(3, 1));
  }
}

void Frame::setPose(const SE3& T_c_w) { cam_->setPose(T_c_w); }

void Frame::setPos(const Vec3& pos) { cam_->setPos(pos); }

void Frame::setKeyframe() { is_keyframe_ = true; }

void Frame::extractFeatures(const cv::Mat& img,
                            const cv::Ptr<cv::FeatureDetector>& detector) {
  vector<cv::KeyPoint> kpts;
  cv::Mat descriptors;
  detector->detectAndCompute(img, cv::Mat{}, kpts, descriptors);
  if (cam_->distCoeffs()(0) == 0)
    frame_utils::undistortKeypoints(cam_->K(), cam_->distCoeffs(), kpts);
  const int num_kpts = kpts.size();
  feats_.reserve(num_kpts);
  for (int i = 0; i < num_kpts; ++i) {
    // FIXME Is sptr<Frame>(this) ok?
    feats_.push_back(make_shared<Feature>(sptr<Frame>(this),
                                          Vec2{kpts[i].pt.x, kpts[i].pt.y},
                                          descriptors.row(i), kpts[i].octave));
  }
}

void Frame::computeBoW(const sptr<Vocabulary>& voc) {
  // Collect descriptors.
  vector<cv::Mat> descriptor_vec;
  descriptor_vec.reserve(this->NumObs());
  // FIXME Parameter type in lambda?
  std::transform(feats_.begin(), feats_.end(),
                 std::back_inserter(descriptor_vec),
                 [](const sptr<Feature>& feat) { return feat->descriptor_; });
  voc->transform(descriptor_vec, bow_vec_, feat_vec_, 4);
}

vector<int> Frame::searchFeatures(const Vec2& pt, const int radius,
                                  const int level_low,
                                  const int level_high) const {
  // FIXME Would it be better if we clamp this rather than throw an error?
  CHECK(level_low >= 0 && level_high >= level_low);
  const int num_obs = this->NumObs();
  vector<int> feat_indices;
  feat_indices.reserve(num_obs);
  for (int i = 0; i < num_obs; ++i) {
    const sptr<Feature>& feat = feats_[i];
    const Vec2 dist = (feat->pt_ - pt).cwiseAbs();
    if (dist.x() <= radius && dist.y() <= radius)
      if (feat->level_ >= level_low && feat->level_ <= level_high)
        feat_indices.push_back(i);
  }
  return feat_indices;
}

void Frame::updateConnections() {
  //
  return;
}

double Frame::computeSceneMedianDepth() {
  //
  return 0;
}

bool isObservable(const sptr<MapPoint>& point) const {
  // Test 1: has positive depth.
  const Vec3& p_w = point->pos();
  const Vec3 p_c = cam_->world2camera(p_w);
  if (p_c(2) < 0.) return false;
  // Test 2: does not go out of image boundary.
  const Vec2 repr_pt = cam_->camera2pixel(p_c);
  if (!(repr_pt.x() >= x_min_ && repr_pt.x() <= x_max_ &&
        repr_pt.y() >= y_min_ && repr_pt.y() <= y_max_))
    return false;
  // Test 3: predicted scale is consistent with median scale (i.e. within +- 1).
  const double dist = cam_->getDistToCenter(p_w);
  const int level = math_utils::predictLevel(dist);
  if (!(level >= point->median_view_scale_ - 1 &&
        level <= point->median_view_scale_ + 1))
    return false;
  // Test 4: viewing direction is consistent with mean viewing direction.
  const double cos_view_dir =
      (p_w - cam_->getCamCenter()).dot(point->mean_view_dir_) / dist;
  if (cos_view_dir < std::cos(math_utils::degree2radian(60.))) return false;

  // Store the computed result to be used in searching.
  point->repr_x_ = repr_pt.x();
  point->repr_y_ = repr_pt.y();
  point->level_ = level;
  point->cos_view_dir_ = cos_view_dir;
  return true;
}

int Frame::computeTrackedPoints(const int min_n_obs = 0) const {
  int n_tracked_points = 0;
  for (const Feature::Ptr& feat : feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat);
    if (!point || point->nObs() < min_n_obs) continue;
    ++n_tracked_points;
  }
  return n_tracked_points;
}

namespace frame_utils {

void undistortKeypoints(const Mat33& K, const Vec4& dist_coeffs,
                        std::vector<cv::KeyPoint>& kpts) {
  const int num_kpts = kpts.size();
  cv::Mat kpts_mat(num_kpts, 2, CV_64F);
  for (int i = 0; i < num_kpts; ++i) {
    kpts_mat.at<double>(i, 0) = kpts[i].pt.x;
    kpts_mat.at<double>(i, 1) = kpts[i].pt.y;
  }
  cv::Mat K_, dist_coeffs_;
  cv::eigen2cv(K, K_);
  cv::eigen2cv(dist_coeffs, dist_coeffs_);
  cv::undistortPoints(kpts_mat, kpts_mat, K_, dist_coeffs_, cv::Mat{}, K_);
  for (int i = 0; i < num_kpts; ++i) {
    kpts[i].pt.x = kpts_mat.at<double>(i, 0);
    kpts[i].pt.y = kpts_mat.at<double>(i, 1);
  }
}

void computeImageBounds(const cv::Mat& img, const Mat33& K,
                        const Vec4& dist_coeffs, cv::Mat& corners) {
  corners = cv::Mat(4, 2, CV_32F);
  cv::Mat K_, dist_coeffs_;
  cv::eigen2cv(K, K_);
  cv::eigen2cv(dist_coeffs, dist_coeffs_);
  corners.at<float>(0, 0) = 0.0;
  corners.at<float>(0, 1) = 0.0;
  corners.at<float>(1, 0) = img.cols;
  corners.at<float>(1, 1) = 0.0;
  corners.at<float>(2, 0) = 0.0;
  corners.at<float>(2, 1) = img.rows;
  corners.at<float>(3, 0) = img.cols;
  corners.at<float>(3, 1) = img.rows;
  cv::undistortPoints(corners, corners, K_, dist_coeffs_, cv::Mat{}, K_);
}

}  // namespace frame_utils
}  // namespace mono_slam