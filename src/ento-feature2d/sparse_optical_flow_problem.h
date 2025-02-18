#ifndef SPARSE_OPTICAL_FLOW_PROBLEM_H
#define SPARSE_OPTICAL_FLOW_PROBLEM_H

#include <ento-math/core.h>

#include <ento-feature2d/optical_flow_problem.h>
#include <ento-feature2d/feat2d_util.h>

#include <image_io/Image.h>

#include <ento-bench/problem.h>
#include <ento-util/containers.h>

namespace EntoFeature2D
{

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType = Keypoint<int16_t>, typename PixelType = int16_t>
class SparseOpticalFlowProblem:
  public OpticalFlowProblem<SparseOpticalFlowProblem<Kernel,
      Rows, Cols, NumFeats, KeypointType, PixelType>, Rows, Cols, PixelType>
{
private:
  Kernel kernel_;

public:
  static constexpr bool SavesResults_ = true;

  using KeypointT_ = KeypointType;
  using CoordT_ = typename KeypointType::CoordT_;
  using PixelT_ = PixelType;
  static constexpr size_t NumFeats_ = NumFeats;
  static constexpr auto img_header_buffer = PGMHeader<Rows, Cols, PixelType>::generate();


  // @TODO: Validator needs to be implemented 
  // typedef LucasKanadeValidator validator_;
  
  //////// Class Members /////////

  // Sparse Input. OpticalFlowProblem base stored img1_ and img2_.
  FeatureArray<KeypointT_, NumFeats> feats_;

  // Outputs 
  EntoUtil::EntoContainer<EntoMath::Vec2<CoordT_>, NumFeats> flows_;
  EntoUtil::EntoContainer<EntoMath::Vec2<CoordT_>, NumFeats> flows_gt_;

  //////// Constructors /////////
  SparseOpticalFlowProblem(Kernel kernel) : kernel_(std::move(kernel)) {};

  /////// Problem Interface ///////
#ifdef NATIVE
  std::string serialize_impl() const;
  bool        deserialize_impl(const std::string& line);
#else
  const char* serialize_impl() const;
  bool        deserialize_impl(const char* line);
#endif 
    
  bool validate_impl();
  bool solve_impl();
  void clear_impl();

  static constexpr const char* header_impl() { return ""; }
  static constexpr const char* output_header_impl() { return ""; }

private:
  bool deserialize_img();
  bool deserialize_features();
};

#ifdef NATIVE
template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_impl(const std::string& line)
{
  std::istringstream iss(line);

  std::string img1_path, img2_path, feat_path; 
  char comma;

  if ( !( iss >> img1_path >> comma) || ( comma != ',' ) )
  { 
    return false;
  }

  if ( !( iss >> img2_path >> comma) || ( comma != ',' ) )
  { 
    return false;
  }

  if ( !( iss >> feat_path ) )
  { 
    return false;
  }


  //img1_.image_from_pgm(img1_path);
  //img2_.image_from_pgm(img2_path);
}

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
std::string SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
serialize_impl() const
{

}

#else
template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_impl(const char* line)
{
  
}

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
const char* SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
serialize_impl() const
{

}
#endif



} // namespace EntoFeature2D



#endif // SPARSE_OPTICAL_FLOW_PROBLEM_H
