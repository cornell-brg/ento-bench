#ifndef OPTICAL_FLOW_PROBLEM_H
#define OPTICAL_FLOW_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-math/core.h>
#include <ento-util/containers.h>
#include <image_io/Image.h>
#include <ento-feature2d/feat2d_util.h>

namespace EntoFeature2D
{

template <typename Derived, size_t Rows, size_t Cols, typename PixelT = uint8_t>
class OpticalFlowProblem :
  public EntoBench::EntoProblem<Derived>
{
public:
  using PixelT_ = PixelT;
  static constexpr size_t ImageRows_ = Rows;
  static constexpr size_t ImageCols_ = Cols;

  Image<Rows, Cols, PixelT> img1_;
  Image<Rows, Cols, PixelT> img2_;

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

protected:
  // Protected Constructor to prevent direct instantiation.
  OpticalFlowProblem() = default;
};

} // namespace EntoFeature2D

#endif
