#ifndef SPARSE_OPTICAL_FLOW_PROBLEM_H
#define SPARSE_OPTICAL_FLOW_PROBLEM_H

#include <ento-math/core.h>

#include <ento-feature2d/optical_flow_problem.h>
#include <ento-feature2d/feat2d_util.h>

#include <image_io/Image.h>

#include <ento-bench/problem.h>
#include <ento-util/containers.h>

#ifdef NATIVE
#include <fstream>
#endif

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

  // Feature inputs. Feats_ for img1_.
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
  bool deserialize_imgs(const std::string& img1_path,
                        const std::string& img2_path);
  bool deserialize_features(const std::string& feats_path);
  bool deserialize_imgs(const char* img1_path,
                        const char* img2_path);
  bool deserialize_features(const char* feats_path);

};

#ifdef NATIVE
template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_impl(const std::string& line)
{
  std::istringstream iss(line);

  ENTO_DEBUG("Line: %s", line.c_str());

  std::string img1_path, img2_path, feat_path, flow_gt_path; 
  char comma;

  if ( !std::getline(iss, img1_path, ',') )
  { 
    ENTO_DEBUG("Failed reading first img path");
    ENTO_DEBUG("IMG1 Path: %s\n", img1_path.c_str());
    return false;
  }

  if ( !std::getline(iss, img2_path, ',') )
  { 
    ENTO_DEBUG("Failed reading second img path");
    ENTO_DEBUG("IMG2 Path: %s\n", img2_path.c_str());
    return false;
  }

  if ( !std::getline(iss, feat_path, ',') )
  { 
    ENTO_DEBUG("Failed reading feat path");
    ENTO_DEBUG("Feat path: %s\n", feat_path.c_str())
    return false;
  }

  if ( !std::getline(iss, flow_gt_path) )
  { 
    ENTO_DEBUG("Failed reading flow_gt_path path");
    ENTO_DEBUG("Flow gt path: %s\n", flow_gt_path.c_str())
    return false;
  }

  bool success = deserialize_imgs(img1_path, img2_path);
  success &= deserialize_features(feat_path);
  return success;
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

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_imgs(const std::string& img1_path, const std::string& img2_path)
{
  bool success = true;
  ENTO_DEBUG("Opening Img1");
  success &= this->img1_.image_from_pgm(img1_path.c_str());
  
  ENTO_DEBUG("Opening Img2");
  success &= this->img2_.image_from_pgm(img2_path.c_str());
  return success;
}

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_imgs(const char* img1_path, const char* img2_path)
{
  bool success = true;
  ENTO_DEBUG("Opening Img1");
  success &= this->img1_.image_from_pgm(img1_path);
  
  ENTO_DEBUG("Opening Img2");
  success &= this->img2_.image_from_pgm(img2_path);
  return success;
}

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_features(const std::string& feats1_path)
{
  std::ifstream feats1_file(feats1_path);

  if (!feats1_file.is_open())
  {
    ENTO_DEBUG("Failed to open feature file: %s", feats1_path.c_str());
    return false;
  }

  int num_feats = 0;
  
  // Read the first line to get num_feats
  feats1_file >> num_feats;

  if (num_feats <= 0 || num_feats != NumFeats)
  {
    ENTO_DEBUG("Invalid feature count in file: %d (file) vs %d (NumFeats template param)", num_feats, NumFeats);
    return false;
  }

  // Read features
  for (int i = 0; i < num_feats; ++i)
  {
    int x1, y1;
    char comma;
    
    if (!(feats1_file >> x1 >> comma >> y1) || comma != ',')
    {
      ENTO_DEBUG("Error parsing feature at index %d", i);
      return false;
    }

    if (!feats_.add_keypoint(KeypointType(x1, y1)) )
    {
      ENTO_DEBUG("Feature array capacity exceeded at index %d", i);
      return false;
    }
  }

  ENTO_DEBUG("Successfully deserialized %d features", num_feats);
  return true;
}

} // namespace EntoFeature2D



#endif // SPARSE_OPTICAL_FLOW_PROBLEM_H
