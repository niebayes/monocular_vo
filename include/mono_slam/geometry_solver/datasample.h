#ifndef UZH_MATLAB_PORT_DATASAMPLE_H_
#define UZH_MATLAB_PORT_DATASAMPLE_H_

#include <tuple>

#include "armadillo"
#include "glog/logging.h"

namespace uzh {

//@brief Imitate matlab's datasample. Evenly draw samples from the data with /
// without replacement.
//@param data [m x n] matrix where the observations are stored row-wise if dim
// is 0 (by default) or column-wise if dim is 1.
//@param num_samples num_samples observations will be returned.
//@param dim Along which direction the observations are stored and the samples
// are drawned. 0 for row-wise and 1 for column wise.
//@param replace If true, the returned samples are guaranteed unique.
//@returns
//- samples -- Drawned samples. The layout is dependent on the parameter dim.
//- sample_indices -- [num_samples x 1] column vector containing the the indices
// of the drawned samples wrt. the data.
template <typename T>
std::tuple<arma::Mat<T> /* samples */, arma::uvec /* sample_indices */>
datasample(const arma::Mat<T>& data, const int num_samples, const int dim = 0,
           const bool replace = true) {
  if (data.empty()) LOG(ERROR) << "Empty data.";
  if (num_samples <= 0) LOG(ERROR) << "num_samples must be positive.";
  if ((dim == 0 && num_samples > data.n_rows) ||
      (dim == 1 && num_samples > data.n_cols)) {
    LOG(ERROR) << "num_samples is greater than the size of the data along dim "
               << dim;
  }

  // arma::arma_rng::set_seed_random();

  const int kNumObservations = arma::size(data, dim);

  // Samples to be drawned.
  arma::Mat<T> samples;
  // Indices through which the observations in data are selected.
  arma::uvec sample_indices(num_samples);

  if (replace) {  // Draw samples with replacement
    // Replacement is achieved with arma::randi which produces number within a
    // range randomly.
    sample_indices = arma::randi<arma::uvec>(
        num_samples, arma::distr_param(0, kNumObservations - 1));
  } else {  // Draw samples without replacement
    // No replacement is achiveved with arma::randperm which shuffles the
    // observations randomly. By definition, there's no replacement.
    sample_indices = arma::randperm(kNumObservations - 1, num_samples);
  }

  if (dim == 0) {
    samples = data.rows(sample_indices);
  } else {
    samples = data.cols(sample_indices);
  }

  return {samples, sample_indices};
}

}  // namespace uzh

#endif  // UZH_MATLAB_PORT_DATASAMPLE_H_