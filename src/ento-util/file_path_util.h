#ifndef ENTO_UTIL_FILE_PATH_UTIL_H
#define ENTO_UTIL_FILE_PATH_UTIL_H

#include <string>
#include <ento-util/debug.h>

#ifdef NATIVE
#include <sstream> 
#endif

#include <cstdio> 
#include <cstring>

namespace EntoUtil {

constexpr size_t MAX_PATH = 256;

#ifdef NATIVE
// Native build: Use std::string
inline std::string build_file_path(const std::string& base_path,
                                   const std::string& relative_path) {
  if (base_path.empty() || relative_path.empty()) 
  {
    return ""; // Return an empty string for invalid paths
  }

  std::ostringstream oss;
  oss << base_path;
  if (base_path.back() != '/')
  {
    oss << '/';
  }
  oss << relative_path;
  return oss.str();
}
#endif
// MCU build: Use char* with a fixed-size buffer
inline bool build_file_path(const char* base_path,
                            const char* relative_path,
                            char* output,
                            size_t output_size) {
  if (!base_path || !relative_path || !output || output_size == 0)
  {
    ENTO_DEBUG("Invalid input to build_file_path.");
    return false; // Invalid input
  }

  int result = snprintf(output, output_size, "%s/%s", base_path, relative_path);
  if (result < 0)
  {
    ENTO_DEBUG("Error formatting file path.");
    return false; // snprintf error
  }
  if (static_cast<size_t>(result) >= output_size)
  {
    ENTO_DEBUG("File path truncated. Increase buffer size.");
    return false; // Truncation
  }

  return true;
}

inline void build_file_paths(const char* base_path,
                             const char** relative_paths,
                             char** new_paths,
                             size_t output_size,
                             size_t num_paths)
{
  if (!base_path || !relative_paths || !new_paths || !output_size)
  {
    ENTO_DEBUG("Invalid input arrays in build_file_paths.");
    return;
  }

  for (size_t i = 0; i < num_paths; ++i)
  {
    if (!base_path || !relative_paths[i] || !new_paths[i] || output_size == 0)
    {
      ENTO_DEBUG("Skipping invalid path at index %zu.", i);
      continue;
    }

    int result = snprintf(new_paths[i], output_size, "%s/%s", base_path, relative_paths[i]);

    if (result < 0)
    {
      ENTO_DEBUG("Error formatting file path at index %zu.", i);
      continue;
    }
    if (static_cast<size_t>(result) >= output_size)
    {
      ENTO_DEBUG("File path truncated at index %zu. Increase buffer size.", i);
    }
  }
}

inline void get_file_directory(const char* filepath, size_t result_size, char* result)
{
  strncpy(result, filepath, result_size);
  result[result_size - 1] = '\0';  // Ensure null termination

  char* last_slash = strrchr(result, '/');  // POSIX
  if (!last_slash)
  {
    last_slash = strrchr(result, '\\');  // Windows
  }

  if (last_slash)
  {
    *last_slash = '\0';  // Truncate at the last slash
  }
}

inline bool is_absolute_path(const char* path)
{
  return path[0] == '/' || (isalpha(path[0]) && path[1] == ':');
}

inline bool is_absolute_path(const std::string& path)
{
  return is_absolute_path(path.c_str());
}

// Function to resolve paths (const char* overload)
inline std::string resolve_path(const char* path)
{
  if (!path || *path == '\0')
    return std::string(DATASET_PATH);

  if (is_absolute_path(path))
    return std::string(path);
  else
    return std::string(DATASET_PATH) + "/" + path;
}

inline void resolve_path(const char* path, char* resolved_path, size_t buffer_size)
{
  if (!path || *path == '\0')
  {
    strncpy(resolved_path, DATASET_PATH, buffer_size - 1);
    resolved_path[buffer_size - 1] = '\0';
    return;
  }

  if (is_absolute_path(path))
  {
    strncpy(resolved_path, path, buffer_size - 1);
    resolved_path[buffer_size - 1] = '\0';
  }
  else
  {
    snprintf(resolved_path, buffer_size, "%s/%s", DATASET_PATH, path);
  }
}

// Function to resolve paths (std::string overload)
inline std::string resolve_path(const std::string& path)
{
  // Delegate to the char* version
  return resolve_path(path.c_str()); 
}

} // namespace EntoUtil

#endif // ENTO_UTIL_FILE_PATH_UTIL_H
