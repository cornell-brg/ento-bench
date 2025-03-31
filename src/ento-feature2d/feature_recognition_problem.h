#ifndef FEATURE_RECOGNITION_PROBLEM_H
#define FEATURE_RECOGNITION_PROBLEM_H

#ifdef NATIVE
#include <fstream>
#endif

#include <sstream>
#include <iostream>
#include <vector>
#include <ento-util/debug.h>
#include <ento-feature2d/feat2d_util.h>
#include <image_io/Image.h>
#include <ento-bench/problem.h>
#include <ento-feature2d/orb.h>
// #include other needed headers

namespace EntoFeature2D 
{

template <
  typename Kernel,
  size_t NumFeats,
  size_t Rows,
  size_t Cols,
  typename PixelType,
  bool DoDetection   = true,
  bool DoDescription = false 
>
class FeatureRecognitionProblem :
  public EntoBench::EntoProblem<
    FeatureRecognitionProblem<Kernel, NumFeats, Rows, Cols,
                              PixelType, DoDetection, DoDescription>
  >
  
{
public:
  // Aliases and constexpr values
  using Kernel_ = Kernel;
  using PixelT_ = PixelType;
  using KeypointT_ = typename Kernel::KeypointType;
  static constexpr bool DoDetection_ = DoDetection;
  static constexpr bool DoDescription_ = DoDescription;
  static constexpr size_t NumFeats_ = NumFeats;
  static constexpr size_t ImageRows_ = Rows;
  static constexpr size_t ImageCols_ = Cols;
  static constexpr auto header_buffer = PGMHeader<Rows, Cols, PixelType>::generate();

  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = false;
  static constexpr bool RequiresSetup_ = false;

  
  using DescriptorT_ = std::conditional_t<!DoDescription, 
                         std::monostate, 
                         typename Kernel::DescriptorType>;

  //////// Class Members /////////
  Kernel kernel_;

  // Ground truth. We always needs feats_gt_.
  FeatureArray<KeypointT_, NumFeats> feats_gt_;
  std::size_t num_feats_gt_ = NumFeats;


  std::conditional_t<DoDescription_,
                     std::array<DescriptorT_, NumFeats>,
                     std::monostate> descs_gt_;
  

  // Input and Output containers for the Kernel Under Test
  Image<Rows, Cols, PixelType> img_;

  std::conditional_t<DoDetection_,
                     FeatureArray<KeypointT_, NumFeats>,
                     std::monostate> feats_;

  std::conditional_t<DoDescription_,
                     std::array<DescriptorT_, NumFeats>,
                     std::monostate> descs_;

  //===================================
  //--------- Constructors ------------

  FeatureRecognitionProblem(Kernel kernel)
    : kernel_(std::move(kernel)) {} 

  //===================================

  //===================================
  //--------- Problem Interface ------------
#ifdef NATIVE
  std::string serialize_impl() const;
  bool deserialize_impl(const std::string& line);
#else
  const char* serialize_impl();
#endif
  bool deserialize_impl(const char* line);
  bool validate_impl() const;
  void solve_impl();
  void clear_impl()
  {
    //descs_.clear();
    //if constexpr (DoDetection_)
    //  feats_gt_.clear();
    //if constexpr (DoDescription_)
    //  descs_gt_.clear();
  }

  static constexpr const char* header_impl() { return "Feature Recognition Problem"; }

  //===================================

private:
#ifdef NATIVE
  bool load_image(const std::string &path);
  bool load_features(const std::string &path);
  bool load_descriptors(const std::string &path);
#endif
  bool load_image(const char* path);
  bool load_features(const char* path);
  bool load_descriptors(const char* path);


};


#ifdef NATIVE
// The harness calls this to parse a line from the dataset
template <typename Kernel, size_t NumFeats,
         size_t Rows, size_t Cols, typename PixelType,
         bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
deserialize_impl(const std::string &line)
{
  std::istringstream iss(line);
  ENTO_DEBUG("Hello world");
  std::string image_path, feature_path;
  ENTO_DEBUG("Line: %s", line.c_str());
  char comma;
  if constexpr (!DoDescription)
  {
    // Get image path
    if (!std::getline(iss, image_path, ','))
    {
      ENTO_DEBUG("Failed to parse image_path");
      return false;
    }

    // Get feature path
    if (!std::getline(iss, feature_path, ','))
    {
      ENTO_DEBUG("Failed to parse feature_path");
      return false;
    }
    // Always load image
    if (!load_image(image_path))
    {
      ENTO_DEBUG("Failed to load image: %s", image_path.c_str());
      return false;
    }

    // If we are NOT detecting in the kernel, we rely on ground-truth coords
    // from feature_path. E.g. load x,y pairs into m_keypoints
    if (!load_features(feature_path))
    {
      ENTO_DEBUG("Failed to load ground-truth features from: %s", feature_path.c_str());
      return false;
    }
  }
  else
  {
    std::string desc_path;
    // Get image path
    if (!std::getline(iss, image_path, ','))
    {
      ENTO_DEBUG("Failed to parse image_path");
      return false;
    }

    // Get feature path
    if (!std::getline(iss, feature_path, ','))
    {
      ENTO_DEBUG("Failed to parse feature_path");
      return false;
    }

    // Get descriptor path
    if (!std::getline(iss, desc_path, ','))
    {
      ENTO_DEBUG("Failed to parse desc_path");
      return false;
    }

    // Always load image
    if (!load_image(image_path))
    {
      ENTO_DEBUG("Failed to load image: %s", image_path.c_str());
      return false;
    }

    // If we are NOT detecting in the kernel, we rely on ground-truth coords
    // from feature_path. E.g. load x,y pairs into m_keypoints
    if (!load_features(feature_path))
    {
      ENTO_DEBUG("Failed to load ground-truth features from: %s", feature_path.c_str());
      return false;
    }

    // Load descriptors
    if (!load_descriptors(desc_path))
    {
      ENTO_DEBUG("Failed to load ground-truth descriptors from: %s", desc_path.c_str());
      return false;
    }
  }
  return true;
}

template <typename Kernel, size_t NumFeats,
         size_t Rows, size_t Cols, typename PixelType,
         bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
load_image(const std::string& filename)
{
  int result = img_.image_from_pgm(filename.c_str());
  return (result == 1);
}

template <typename Kernel, size_t NumFeats,
         size_t Rows, size_t Cols, typename PixelType,
         bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
load_features(const std::string& gt_path)
{
  std::ifstream gt_file(gt_path);
  std::string line;

  if (!gt_file.is_open())
  {
    ENTO_ERROR("Failed to open gt file: %s", gt_path.c_str());
    return false;
  }

  int num_gt_feats = 0;
  
  // Read the first line to get num_feats
  if (std::getline(gt_file, line))
  {
    std::istringstream iss(line);
    iss >> num_gt_feats;
  }

  if (num_gt_feats <= 0 || num_gt_feats > NumFeats)
  {
    ENTO_ERROR("Invalid feature count in file: %d (file) vs %d (NumFeats template param)", num_gt_feats, NumFeats);
    return false;
  }

  // Read features
  for (int i = 0; i < num_gt_feats; ++i)
  {
    int x2_gt, y2_gt;
    char comma;
    
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

    if (!feats_gt_.add_keypoint(KeypointT_(x2_gt, y2_gt)) )
    {
      ENTO_ERROR("Feature GT next array capacity exceeded at index %d", i);
      return false;
    }

    ENTO_DEBUG("Added feature gt: %i, %i", x2_gt, y2_gt);
  }

  ENTO_DEBUG("Successfully deserialized %d features", num_gt_feats);
  return true;

}

template <typename Kernel, size_t NumFeats,
          size_t Rows, size_t Cols, typename PixelType,
          bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
load_descriptors(const std::string& filename)
{
  static_assert(DoDescription, "Problem must have DoDescription set to true.");

  std::ifstream infile(filename);
  if (!infile)
  {
    ENTO_ERROR("Unable to open descriptor file: %s", filename.c_str());
    return false;
  }

  std::string line;
  size_t num_descs_in_file = 0;

  // Read the first line to get num_descs_in_file
  if (std::getline(infile, line))
  {
    std::istringstream iss(line);
    iss >> num_descs_in_file;
  }

  if (num_descs_in_file > NumFeats)
  {
    ENTO_ERROR("Descriptor count mismatch: file has %zu descriptors, expected %zu",
               num_descs_in_file, NumFeats);
    return false;
  }

  ENTO_DEBUG("Number of descriptors: %zu", num_descs_in_file);

  if constexpr (std::is_same_v<DescriptorT_, BRIEFDescriptor>)
  {
    for (size_t i = 0; i < NumFeats; ++i)
    {
      if (!std::getline(infile, line))
      {
        ENTO_ERROR("Failed to read descriptor line %zu", i);
        return false;
      }

      ENTO_DEBUG("Line: %s", line.c_str());

      std::istringstream iss(line);
      std::string byte_str;
      for (size_t byte_idx = 0; byte_idx < 32; ++byte_idx)
      {
        if (!std::getline(iss, byte_str, ','))
        {
          ENTO_ERROR("Failed to parse byte %zu of descriptor %zu", byte_idx, i);
          return false;
        }

        int byte_val = std::stoi(byte_str);
        descs_gt_[i].data[byte_idx] = static_cast<uint8_t>(byte_val);
      }
    }
    ENTO_DEBUG("Successfully loaded %zu BRIEF descriptors", NumFeats);
  }
  //else if constexpr (std::is_same_v<DescriptorT_, SIFTDescriptor>)
  //{
  //  for (size_t i = 0; i < NumFeats; ++i)
  //  {
  //    if (!std::getline(infile, line))
  //    {
  //      ENTO_ERROR("Failed to read descriptor line %zu", i);
  //      return false;
  //    }

  //    std::istringstream iss(line);
  //    for (size_t dim = 0; dim < 128; ++dim)
  //    {
  //      float val;
  //      if (!(iss >> val))
  //      {
  //        ENTO_ERROR("Failed to parse dimension %zu of SIFT descriptor %zu", dim, i);
  //        return false;
  //      }
  //      descs_[i].data[dim] = val;
  //    }
  //  }
  //  ENTO_DEBUG("Successfully loaded %zu SIFT descriptors", NumFeats);
  //}
  else
  {
    ENTO_ERROR("Unsupported Descriptor Type!");
    return false;
  }

  return true;
}
#endif

template <typename Kernel, size_t NumFeats,
         size_t Rows, size_t Cols, typename PixelType,
         bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
load_image(const char* filename)
{
  int result = img_.image_from_pgm(filename);
  return (result == 1);
}

template <typename Kernel, size_t NumFeats,
          size_t Rows, size_t Cols, typename PixelType,
          bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
load_features(const char* gt_path)
{
  ENTO_DEBUG("Deserializing features const char*");
  FILE *file = fopen(gt_path, "r");
  if (!file)
  {
    ENTO_ERROR("Failed to open gt file: %s", gt_path);
    return false;
  }

  char line[1024];

  // Read first line for num_feats
  if (!fgets(line, sizeof(line), file))
  {
    ENTO_ERROR("Failed to read feature count line");
    fclose(file);
    return false;
  }

  int num_gt_feats = atoi(line);

  if (num_gt_feats <= 0 || num_gt_feats > NumFeats)
  {
    ENTO_ERROR("Invalid feature count in file: %d (file) vs %zu (NumFeats template param)",
               num_gt_feats, NumFeats);
    fclose(file);
    return false;
  }

  // Read each feature
  for (int i = 0; i < num_gt_feats; ++i)
  {
    if (!fgets(line, sizeof(line), file))
    {
      ENTO_ERROR("Error reading gt feature line at index %d", i);
      fclose(file);
      return false;
    }

    // Parse x,y coordinates separated by comma
    char* token = strtok(line, ",");
    if (!token)
    {
      ENTO_ERROR("Error parsing x coordinate at feature index %d", i);
      fclose(file);
      return false;
    }

    int x2_gt = atoi(token);

    token = strtok(nullptr, ",");
    if (!token)
    {
      ENTO_ERROR("Error parsing y coordinate at feature index %d", i);
      fclose(file);
      return false;
    }

    int y2_gt = atoi(token);

    if (!feats_gt_.add_keypoint(KeypointT_(x2_gt, y2_gt)))
    {
      ENTO_ERROR("Feature GT array capacity exceeded at index %d", i);
      fclose(file);
      return false;
    }

    ENTO_DEBUG("Added feature gt: %i, %i", x2_gt, y2_gt);
  }

  fclose(file);
  ENTO_DEBUG("Successfully deserialized %d features", num_gt_feats);
  return true;
}

template <typename Kernel, size_t NumFeats,
          size_t Rows, size_t Cols, typename PixelType,
          bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
load_descriptors(const char* filename)
{
  static_assert(DoDescription, "Problem must have DoDescription set to true.");

  FILE *file = fopen(filename, "r");
  if (!file)
  {
    ENTO_ERROR("Unable to open descriptor file: %s", filename);
    return false;
  }

  char line[1024];

  // Read the first line to get num_descs_in_file
  if (!fgets(line, sizeof(line), file))
  {
    ENTO_ERROR("Unable to read descriptor count line.");
    fclose(file);
    return false;
  }

  size_t num_descs_in_file = (size_t)atoi(line);

  if (num_descs_in_file > NumFeats)
  {
    ENTO_ERROR("Descriptor count mismatch: file has %zu descriptors, expected %zu",
               num_descs_in_file, NumFeats);
    fclose(file);
    return false;
  }

  ENTO_DEBUG("Number of descriptors: %zu", num_descs_in_file);

  if constexpr (std::is_same_v<DescriptorT_, BRIEFDescriptor>)
  {
    for (size_t i = 0; i < NumFeats; ++i)
    {
      if (!fgets(line, sizeof(line), file))
      {
        ENTO_ERROR("Failed to read descriptor line %zu", i);
        fclose(file);
        return false;
      }

      ENTO_DEBUG("Line: %s", line);

      // Parse comma-separated values using strtok
      char *token = strtok(line, ",");
      for (size_t byte_idx = 0; byte_idx < 32; ++byte_idx)
      {
        if (!token)
        {
          ENTO_ERROR("Failed to parse byte %zu of descriptor %zu", byte_idx, i);
          fclose(file);
          return false;
        }

        int byte_val = atoi(token);
        descs_gt_[i].data[byte_idx] = (uint8_t)byte_val;

        token = strtok(nullptr, ",");
      }
    }
    ENTO_DEBUG("Successfully loaded %zu BRIEF descriptors", NumFeats);
  }
  else
  {
    ENTO_ERROR("Unsupported Descriptor Type!");
    fclose(file);
    return false;
  }

  fclose(file);
  return true;
}

template <typename Kernel, size_t NumFeats,
         size_t Rows, size_t Cols, typename PixelType,
         bool DoDetection, bool DoDescription>
bool FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
deserialize_impl(const char* line)
{
  ENTO_DEBUG("Inside deserialize impl const char* line");
  if (!line || *line == '\0')
  {
    ENTO_ERROR("Empty or null input line.");
    return false;
  }

  ENTO_DEBUG("Line: %s", line);

  // Buffers to store extracted file paths
  char image_path[256], feature_path[256], desc_path[256];

  if constexpr (!DoDescription)
  {
    // Expect exactly two fields: image_path, feature_path
    int num_parsed = sscanf(line, "%255[^,],%255s", image_path, feature_path);

    if (num_parsed != 2)
    {
      ENTO_ERROR("Failed to parse expected fields (image_path, feature_path). Parsed: %d", num_parsed);
      return false;
    }

    ENTO_DEBUG("Image Path: %s", image_path);
    ENTO_DEBUG("Feature Path: %s", feature_path);

    char resolved_image[256], resolved_feature[256];
    EntoUtil::resolve_path(image_path, resolved_image, sizeof(resolved_image));
    EntoUtil::resolve_path(feature_path, resolved_feature, sizeof(resolved_feature));

    // Load image
    if (!load_image(resolved_image))
    {
      ENTO_ERROR("Failed to load image: %s", resolved_image);
      return false;
    }

    // Load ground-truth features
    if (!load_features(resolved_feature))
    {
      ENTO_ERROR("Failed to load ground-truth features: %s", resolved_feature);
      return false;
    }
  }
  else
  {
    // Expect exactly three fields: image_path, feature_path, desc_path
    int num_parsed = sscanf(line, "%255[^,],%255[^,],%255s", image_path, feature_path, desc_path);

    if (num_parsed != 3)
    {
      ENTO_ERROR("Failed to parse expected fields (image_path, feature_path, desc_path). Parsed: %d", num_parsed);
      return false;
    }

    ENTO_DEBUG("Image Path: %s", image_path);
    ENTO_DEBUG("Feature Path: %s", feature_path);
    ENTO_DEBUG("Descriptor Path: %s", desc_path);

    char resolved_image[256], resolved_feature[256], resolved_desc[256];
    EntoUtil::resolve_path(image_path, resolved_image, sizeof(resolved_image));
    EntoUtil::resolve_path(feature_path, resolved_feature, sizeof(resolved_feature));
    EntoUtil::resolve_path(desc_path, resolved_desc, sizeof(resolved_desc));

    // Load image
    if (!load_image(resolved_image))
    {
      ENTO_ERROR("Failed to load image: %s", resolved_image);
      return false;
    }

    // Load ground-truth features
    if (!load_features(resolved_feature))
    {
      ENTO_ERROR("Failed to load ground-truth features: %s", resolved_feature);
      return false;
    }

    // Load descriptors
    if (!load_descriptors(resolved_desc))
    {
      ENTO_ERROR("Failed to load ground-truth descriptors: %s", resolved_desc);
      return false;
    }
  }

  return true;
}


template <typename Kernel, size_t NumFeats,
         size_t Rows, size_t Cols, typename PixelType,
         bool DoDetection, bool DoDescription>
void FeatureRecognitionProblem<Kernel, NumFeats,
                               Rows, Cols, PixelType,
                               DoDetection, DoDescription>::
solve_impl()
{
  if constexpr (DoDetection && DoDescription)
  {
    // Both detection and description are performed
    kernel_(img_, feats_, descs_);
  }
  else if constexpr (DoDetection && !DoDescription)
  {
    // Only detection performed
    kernel_(img_, feats_);
  }
  else if constexpr (!DoDetection && DoDescription)
  {
    // Only description performed, using ground-truth keypoints
    kernel_(img_, feats_gt_, descs_);
  }
  else
  {
    static_assert(DoDetection || DoDescription, "Kernel must do detection, description, or both!");
  } 
}


} // namespace EntoFeature2D 



#endif // FEATURE_RECOGNITION_PROBLEM_H
