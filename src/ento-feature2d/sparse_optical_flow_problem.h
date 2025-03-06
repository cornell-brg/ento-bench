#ifndef SPARSE_OPTICAL_FLOW_PROBLEM_H
#define SPARSE_OPTICAL_FLOW_PROBLEM_H

#include <ento-math/core.h>

#include <ento-feature2d/optical_flow_problem.h>
#include <ento-feature2d/feat2d_util.h>

#include <image_io/Image.h>

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/file_path_util.h>

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
  LucasKanadeValidator<KeypointType> validator_;
public:
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = true;

  using KeypointT_ = KeypointType;
  using CoordT_ = typename KeypointType::CoordT_;
  using PixelT_ = PixelType;
  static constexpr size_t NumFeats_ = NumFeats;
  static constexpr auto img_header_buffer = PGMHeader<Rows, Cols, PixelType>::generate();

  // @TODO: Validator needs to be implemented 
  
  //////// Class Members /////////

  // Feature inputs. Feats_ for img1_.
  FeatureArray<KeypointT_, NumFeats> feats_;

  // Outputs 
  FeatureArray<KeypointT_, NumFeats> feats_next_;
  FeatureArray<KeypointT_, NumFeats> feats_next_gt_;
  
  // TODO: Add flag array for found features
  // std::array<bool, NumFeats> feats_next_flags_;

  //////// Constructors /////////
  SparseOpticalFlowProblem(Kernel kernel) : kernel_(std::move(kernel)) {};

  /////// Problem Interface ///////
#ifdef NATIVE
  std::string serialize_impl() const;
  bool        deserialize_impl(const std::string& line);
#else
  const char* serialize_impl() const;
#endif 
  bool        deserialize_impl(const char* line);
    
  bool validate_impl() const;
  void solve_impl();
  void clear_impl();

  static constexpr const char* header_impl() { return "Sparse Optical Flow Dataset"; }
  static constexpr const char* output_header_impl() { return ""; }

private:
#ifdef NATIVE
  bool deserialize_imgs(const std::string& img1_path,
                        const std::string& img2_path);
  bool deserialize_features(const std::string& feats_path,
                            const std::string& feats_next_gt_path);
#endif
  bool deserialize_imgs(const char* img1_path,
                        const char* img2_path);
  bool deserialize_features(const char*, const char*);

};

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
validate_impl() const
{
  return validator_.validate(feats_next_, feats_next_gt_);
}

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
void SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
solve_impl()
{
  kernel_(this->img1_, this->img2_, feats_, &feats_next_);
}

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
void SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
clear_impl()
{
  this->img1_.clear();
  this->img2_.clear();
  feats_next_.clear();
  feats_next_gt_.clear();
}

#ifdef NATIVE
template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_impl(const std::string& line)
{
  std::istringstream iss(line);

  ENTO_DEBUG("Line: %s", line.c_str());

  std::string img1_path, img2_path, feat_path, gt_path; 
  char comma;

  if ( !std::getline(iss, img1_path, ',') )
  { 
    ENTO_ERROR("Failed reading first img path");
    ENTO_ERROR("IMG1 Path: %s\n", img1_path.c_str());
    return false;
  }

  if ( !std::getline(iss, img2_path, ',') )
  { 
    ENTO_ERROR("Failed reading second img path");
    ENTO_ERROR("IMG2 Path: %s\n", img2_path.c_str());
    return false;
  }

  if ( !std::getline(iss, feat_path, ',') )
  { 
    ENTO_ERROR("Failed reading feat path");
    ENTO_ERROR("Feat path: %s\n", feat_path.c_str());
    return false;
  }

  if ( !std::getline(iss, gt_path) )
  { 
    ENTO_ERROR("Failed reading flow_gt_path path");
    ENTO_ERROR("Flow gt path: %s\n", gt_path.c_str());
    return false;
  }

  std::string resolved_img1 = EntoUtil::resolve_path(img1_path);
  std::string resolved_img2 = EntoUtil::resolve_path(img2_path);
  std::string resolved_feat = EntoUtil::resolve_path(feat_path);
  std::string resolved_gt   = EntoUtil::resolve_path(gt_path);

  bool success = deserialize_imgs(resolved_img1, resolved_img2);
  success &= deserialize_features(resolved_feat, resolved_gt);
  
  return success;
}

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
std::string SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
serialize_impl() const
{
  std::ostringstream oss;

  for (size_t i = 0; i < feats_next_.size(); ++i)
  {
    if (i == 0) oss << feats_next_.size() << ",";
    if (i > 0) oss << ",";
    const KeypointType& kp = feats_next_[i];
    oss << kp.x << "," << kp.y;
  }
  return oss.str();
}

#else

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
const char* SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
serialize_impl() const
{

}
#endif

template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_impl(const char* line)
{
  if (!line || *line == '\0')
  {
    ENTO_ERROR("Empty or null input line.");
    return false;
  }

  ENTO_DEBUG("Line: %s", line);

  // Buffers to store extracted file paths
  char img1_path[256], img2_path[256], feat_path[256], gt_path[256];

  // Parse the line using sscanf
  int num_parsed = sscanf(line, "%255[^,],%255[^,],%255[^,],%255s",
                          img1_path, img2_path, feat_path, gt_path);

  if (num_parsed != 4)
  {
    ENTO_ERROR("Failed to parse all expected fields. Parsed: %d", num_parsed);
    return false;
  }

  ENTO_DEBUG("IMG1 Path: %s", img1_path);
  ENTO_DEBUG("IMG2 Path: %s", img2_path);
  ENTO_DEBUG("Feat Path: %s", feat_path);
  ENTO_DEBUG("Flow GT Path: %s", gt_path);

  char resolved_img1[256], resolved_img2[256], resolved_feat[256], resolved_gt[256];

  EntoUtil::resolve_path(img1_path, resolved_img1, sizeof(resolved_img1));
  EntoUtil::resolve_path(img2_path, resolved_img2, sizeof(resolved_img2));
  EntoUtil::resolve_path(feat_path, resolved_feat, sizeof(resolved_feat));
  EntoUtil::resolve_path(gt_path, resolved_gt, sizeof(resolved_gt));


  // Call the corresponding functions to handle the parsed paths
  bool success = deserialize_imgs(resolved_img1, resolved_img2);
  success &= deserialize_features(resolved_feat, resolved_gt);
  return success;
}

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
deserialize_features(const std::string& feats_path,
                     const std::string& gt_path)
{
  std::ifstream feats_file(feats_path);
  std::ifstream gt_file(gt_path);

  if (!feats_file.is_open())
  {
    ENTO_ERROR("Failed to open feature file: %s", feats_path.c_str());
    return false;
  }

  if (!gt_file.is_open())
  {
    ENTO_ERROR("Failed to open gt file: %s", gt_path.c_str());
    return false;
  }

  int num_feats = 0, num_gt_feats = 0;
  
  // Read the first line to get num_feats
  std::string line;
  if (std::getline(feats_file, line))
  {
    std::istringstream iss(line);
    iss >> num_feats;
  }

  if (std::getline(gt_file, line))
  {
    std::istringstream iss(line);
    iss >> num_gt_feats;
  }

  if (num_feats <= 0 || num_feats != NumFeats || num_feats != num_gt_feats)
  {
    ENTO_ERROR("Invalid feature count in file: %d (file) vs %d (NumFeats template param)", num_feats, NumFeats);
    return false;
  }

  ENTO_DEBUG("NUM FEATS: %i, %i", num_feats, num_gt_feats);

  // Read features
  for (int i = 0; i < num_feats; ++i)
  {
    if (!std::getline(feats_file, line))
    {
      ENTO_ERROR("Error reading feature line at index %d", i);
      return false;
    }

    std::istringstream feats_iss(line);
    int x1, y1, x2_gt, y2_gt;
    char comma;
    
    if (!(feats_iss >> x1 >> comma >> y1) || comma != ',')
    {
      ENTO_ERROR("Error parsing feature at index %d", i);
      return false;
    }

    if (!std::getline(gt_file, line))
    {
      ENTO_ERROR("Error reading gt feature line at index %d", i);
      return false;
    }

    std::istringstream gt_iss(line);
    if (!(gt_iss >> x2_gt >> comma >> y2_gt) || comma != ',')
    {
      ENTO_ERROR("Error parsing gt feature at index %d", i);
      return false;
    }

    if (!feats_.add_keypoint(KeypointType(x1, y1)) )
    {
      ENTO_ERROR("Feature array capacity exceeded at index %d", i);
      return false;
    }

    if (!feats_next_gt_.add_keypoint(KeypointType(x2_gt, y2_gt)) )
    {
      ENTO_ERROR("Feature GT next array capacity exceeded at index %d", i);
      return false;
    }

    ENTO_DEBUG("Added feature: %i, %i", x1, y1);
    ENTO_DEBUG("Added feature: %i, %i", feats_[i].x, feats_[i].y);
    ENTO_DEBUG("Added feature gt: %i, %i", x2_gt, y2_gt);
  }

  ENTO_DEBUG("Successfully deserialized %d features", num_feats);
  return true;
}


template <typename Kernel, size_t Rows, size_t Cols, size_t NumFeats, typename KeypointType, typename PixelType>
bool SparseOpticalFlowProblem<Kernel, Rows, Cols, NumFeats, KeypointType, PixelType>::
deserialize_features(const char* feats_path, const char* gt_path)
{
  if (!feats_path || *feats_path == '\0')
  {
    ENTO_ERROR("Invalid or empty feature file path.");
    return false;
  }

  if (!gt_path || *gt_path == '\0')
  {
    ENTO_ERROR("Invalid or empty feature file path.");
    return false;
  }

  FILE* feats_file = fopen(feats_path, "r");
  FILE* gt_file = fopen(gt_path, "r");

  if (!feats_file)
  {
    ENTO_ERROR("Failed to open feature file: %s", feats_path);
    return false;
  }

  if (!gt_file)
  {
    ENTO_ERROR("Failed to open ground truth features (next) file: %s", gt_path);
    return false;
  }

  int num_feats = 0, num_feats_gt = 0;
  // Read the first line to get num_feats
  if (fscanf(feats_file, "%d\n", &num_feats) != 1)
  {
    ENTO_ERROR("Failed to read feature count from input feats file: %s", feats_path);
    fclose(feats_file);
    return false;
  }

  if (fscanf(gt_file, "%d\n", &num_feats_gt) != 1)
  {
    ENTO_ERROR("Failed to read feature count from gt file: %s", feats_path);
    fclose(gt_file);
    return false;
  }

  if (num_feats <= 0 || num_feats != NumFeats || num_feats != num_feats_gt)
  {
    ENTO_ERROR("Invalid feature count in file: %d (file) vs %d (NumFeats template param)", num_feats, NumFeats);
    fclose(feats_file);
    return false;
  }

  // Read features
  for (int i = 0; i < num_feats; ++i)
  {
    int x1, y1, x2_gt, y2_gt;
    char comma;

    if (fscanf(feats_file, "%d%c%d\n", &x1, &comma, &y1) != 3 || comma != ',')
    {
      ENTO_ERROR("Error parsing feature at index %d", i);
      fclose(feats_file);
      return false;
    }
    
    if (fscanf(gt_file, "%d%c%d\n", &x2_gt, &comma, &y2_gt) != 3 || comma != ',')
    {
      ENTO_ERROR("Error parsing gt feature (next) at index %d", i);
      fclose(gt_file);
      return false;
    }

    if (!feats_.add_keypoint(KeypointType(x1, y1)) )
    {
      ENTO_ERROR("Feature array capacity exceeded at index %d", i);
      fclose(feats_file);
      return false;
    }

    if (!feats_next_gt_.add_keypoint(KeypointType(x2_gt, y2_gt)) )
    {
      ENTO_ERROR("Feature GT next array capacity exceeded at index %d", i);
      fclose(feats_file);
      return false;
    }
  }

  ENTO_DEBUG("Successfully deserialized %d features", num_feats);
  fclose(feats_file);
  return true;
}

} // namespace EntoFeature2D



#endif // SPARSE_OPTICAL_FLOW_PROBLEM_H
