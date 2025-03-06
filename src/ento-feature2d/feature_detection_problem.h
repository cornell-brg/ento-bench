#ifndef FEATURE_DETECTION_PROBLEM_H
#define FEATURE_DETECTION_PROBLEM_H

#include <Eigen/Dense>
#include <ento-bench/problem.h>
#include <ento-math/core.h>
#include <ento-util/containers.h>
#include <image_io/Image.h>
#include <ento-feature2d/feat2d_util.h>

namespace EntoFeature2D 
{


template <typename Scalar, typename Kernel, size_t MaxFeats=0, size_t Rows = 320, size_t Cols = 320, typename PixelT = uint8_t>
class FeatureDetectionProblem :
  public EntoBench::EntoProblem<FeatureDetectionProblem<Scalar, Kernel, MaxFeats>>
{
public:
  using Scalar_ = Scalar;
  using Kernel_ = Kernel;
  using PixelT_ = PixelT;
  using KeypointT_ = typename Kernel::KeypointType;
  static constexpr size_t MaxFeats_ = MaxFeats;
  static constexpr size_t ImageRows_ = Rows;
  static constexpr size_t ImageCols_ = Cols;
  static constexpr auto header_buffer = PGMHeader<Rows, Cols, PixelT>::generate();

  //////// Class Members /////////
  // Kernel Callable
  Kernel kernel_;

  // Ground truth 
  FeatureArray<KeypointT_, MaxFeats> feats_gt_;
  std::size_t num_feats_gt_ = MaxFeats;

  // Input (full image)
  //std::array<PixelType, Rows * Cols> img_;
  Image<Rows, Cols, PixelT> img_;

  // FeatureDetection Algorithm Generic Outputs
  FeatureArray<KeypointT_, MaxFeats> feats;

  //////// Class Functions /////////
  // Constructor
  FeatureDetectionProblem(Kernel kernel) : kernel_(std::move(kernel)) {}

  // Problem Interface Functions
  // File I/O
#ifdef NATIVE
  std::string serialize_impl() const;
  bool        deserialize_impl(const std::string& line);
#endif
  bool        deserialize_impl(const char* line);

  void solve_impl();
  bool validate_impl();
  void clear_impl();

  static constexpr const char* header_impl()
  {
    return header_buffer.data();
  }

  // Problem Specific Functions
#ifdef NATIVE
  bool load_image(const std::string& filename);
  bool load_features(const std::string& filename);
#endif // ifdef NATIVE
       
  bool load_image(const char* filename);
  bool load_features(const char* filename);
};

template <typename Scalar, typename Kernel, size_t NumFeats, size_t Rows, size_t Cols, typename PixelType>
bool FeatureDetectionProblem<Scalar, Kernel, NumFeats, Rows, Cols, PixelType>
::deserialize_impl(const std::string &line)
{
  std::istringstream iss(line);
  std::string image_path, feature_path;
  if (!(iss >> image_path >> feature_path))
  {
    ENTO_DEBUG("Invalid line format for feature detection problem");
    return false;
  }

  if (!load_image(image_path)) return false;
  if (!load_features(image_path)) return false;
}


#ifdef NATIVE
template <typename Scalar, typename Kernel, size_t NumFeats, size_t Rows, size_t Cols, typename PixelType>
bool FeatureDetectionProblem<Scalar, Kernel, NumFeats, Rows, Cols, PixelType>::
load_image(const std::string& filename)
{
  int result = img_.image_from_pgm(filename.c_str());
  return (result == 1);
}

template <typename Scalar, typename Kernel, size_t NumFeats, size_t Rows, size_t Cols, typename PixelType>
bool FeatureDetectionProblem<Scalar, Kernel, NumFeats, Rows, Cols, PixelType>::
load_features(const std::string& filename)
{

}
#endif // ifdef NATIVE

} // namespace EntoFeature2D

#endif // FEATURE_DETECTION_PROBLEM_H
