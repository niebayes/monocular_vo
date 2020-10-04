#include "mono_slam/config.h"

namespace mono_slam {

bool Config::setConfigFile(const std::string& config_file) {
  if (config_ == nullptr) config_ = sptr<Config>(new Config);
  config_->file_ = cv::FileStorage(config_file, cv::FileStorage::READ);
  if (!config_->file_.isOpened()) {
    LOG(ERROR) << "Failed to loal configuration file.";
    config_->file_.release();
    return false;
  }
  return true;
}

Config::~Config() {
  if (file_.isOpened()) file_.release();
}

template <typename T>
T Config::get(const std::string& key) {
  return T(Config::config_->file_[key]);
}

//! Initialized to nullptr so that it can be properly instantiated (only) on the
//! first use.
sptr<Config> Config::config_ = nullptr;

}  // namespace mono_slam