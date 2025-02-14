#ifndef SPARSE_OPTICAL_FLOW_PROBLEM_H
#define SPARSE_OPTICAL_FLOW_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-feature2d/OpticalFlowProblem.h>
#include <ento-math/core.h>
#include <ento-util/containers.h>
#include <image_io/Image.h>
#include <ento-feature2d/feat2d_util.h>

namespace EntoFeature2D
{

template <size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType = Keypoint, typename PixelT = uint8_t>
class SparseOpticalFlowProblem:
  public OpticalFlowProblem<SparseOpticalFlowProblem<Rows, Cols, NumFeats, PixelT>, Rows, Cols, PixelT>
{
public:
  using KeypointT_ = KeypointType;
  static constexpr size_t NumFeats_ = NumFeats;
  static constexpr auto header_buffer = PGMHeader<Rows, Cols, PixelT>::generate()
  //////// Class Members /////////
  FeatureArray<KeypointT_, NumFeats> feats_;
  
private:

};

} // namespace EntoFeature2D


#endif // SPARSE_OPTICAL_FLOW_PROBLEM_H
