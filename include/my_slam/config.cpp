#include "my_slam/config.h"

static bool Config::SetConfigFile(const std::string& config_file) {
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
static T Config::Get(const std::string& key) {
  return T(Config::config_->file_[key]);
}

//! Initialize to nullptr so that it can be properly initialized in the first
//! call to SetConfigFile.
sptr<Config> Config::config_ = nullptr;