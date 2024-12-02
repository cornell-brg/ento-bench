#ifndef ENTO_UTIL_FILE_PATH_UTIL_H
#define ENTO_UTIL_FILE_PATH_UTIL_H

#include <string>
#include <ento-util/debug.h>

#ifdef NATIVE
#include <sstream> 
#else
#include <cstdio> 
#endif

namespace EntoUtil {

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

} // namespace EntoUtil

#endif // ENTO_UTIL_FILE_PATH_UTIL_H
