#ifndef ENTO_UTIL_FILE_PATH_UTIL_H
#define ENTO_UTIL_FILE_PATH_UTIL_H

#include <string>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#ifdef NATIVE
#include <sstream> 
#endif

#include <cstdio> 
#include <cstring>

namespace EntoUtil {

constexpr size_t MAX_PATH = 256;

// Returns a literal format for a given scalar type.
template <typename Scalar>
constexpr const char* scalar_format() {
    if constexpr (std::is_same_v<Scalar, float>)
        return "%f";
    else if constexpr (std::is_same_v<Scalar, double>)
        return "%lf";
    else if constexpr (std::is_integral_v<Scalar> && std::is_signed_v<Scalar>)
        return "%d";
    else if constexpr (std::is_integral_v<Scalar> && !std::is_signed_v<Scalar>)
        return "%u";
    else
        static_assert(sizeof(Scalar) == 0, "Unsupported scalar type");
}

// Returns the length (number of characters, not counting the null terminator)
// of the format string for a given scalar type.
template <typename Scalar>
constexpr int scalar_format_length() {
    if constexpr (std::is_same_v<Scalar, float>)
        return 2;  // "%f"
    else if constexpr (std::is_same_v<Scalar, double>)
        return 3;  // "%lf"
    else if constexpr (std::is_integral_v<Scalar>)
        return 2;  // "%d" or "%u"
    else
        static_assert(sizeof(Scalar) == 0, "Unsupported scalar type");
}

//---------------------------------------------------------------------
// Compile-time builder for a format string for an array of N scalars.
// For an array with N elements, the format string is built as:
//   scalar_format [, scalar_format] ...
// For example, for N=4 and float, the result is "%f,%f,%f,%f".
template <typename Scalar, int N>
constexpr auto array_format_string_impl() {
    constexpr int sfmt_len = scalar_format_length<Scalar>();
    constexpr int fmt_len = (N == 0 ? 0 : sfmt_len + (N - 1) * (1 + sfmt_len));
    std::array<char, fmt_len + 1> arr = {}; // +1 for null terminator
    int pos = 0;
    constexpr const char* sfmt = scalar_format<Scalar>();
    // Write the first scalar format.
    for (int i = 0; i < sfmt_len; ++i) {
        arr[pos++] = sfmt[i];
    }
    // For each subsequent element, write a comma and then the scalar format.
    for (int e = 1; e < N; ++e) {
        arr[pos++] = ',';
        for (int i = 0; i < sfmt_len; ++i) {
            arr[pos++] = sfmt[i];
        }
    }
    arr[pos] = '\0';
    return arr;
}

//---------------------------------------------------------------------
// Instead of using a static variable inside a constexpr function,
// we define an inline variable template at namespace scope.
template <typename Scalar, int N>
inline constexpr auto array_format_string = array_format_string_impl<Scalar, N>();

// Helper function that returns a pointer to a compile-time format string
// for an array of N scalars of type Scalar.
template <typename Scalar, int N>
constexpr const char* get_array_format() {
    return array_format_string<Scalar, N>.data();
}

//---------------------------------------------------------------------
// get_format for fixed-size Eigen matrices.
// The matrix must have compile-time known dimensions.
template <typename Derived>
constexpr const char* get_format(const Eigen::MatrixBase<Derived>&) {
    constexpr int Rows = Derived::RowsAtCompileTime;
    constexpr int Cols = Derived::ColsAtCompileTime;
    static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,
                  "Matrix must have fixed compile-time dimensions");
    constexpr int N = Rows * Cols;
    return get_array_format<typename Derived::Scalar, N>();
}

//---------------------------------------------------------------------
// get_format for plain C-style arrays (non-Eigen)
// The array must have a compile-time known size.
template <typename Scalar, std::size_t N>
constexpr const char* get_format(const Scalar (&)[N]) {
    return get_array_format<Scalar, N>();
}

//---------------------------------------------------------------------
// Specialization for Eigen::Quaternion
template <typename Scalar>
constexpr const char* get_format(const Eigen::Quaternion<Scalar>&) {
    // A quaternion is composed of 4 scalars.
    return get_array_format<Scalar, 4>();
}

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

// Function to resolve paths that relative to dataset (const char* overload)
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
