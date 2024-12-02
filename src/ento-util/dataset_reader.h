#ifndef ENTO_UTIL_DATASET_READER_H
#define ENTO_UTIL_DATASET_READER_H

#include <string>
#include <array>
#include <ento-util/debug.h> // For ENTO_DEBUG

#ifdef NATIVE
#include <fstream>
#else
#include <cstdio>
#endif

namespace EntoUtil
{

template <typename ProblemInstance>
class DatasetReader
{
public:
  // Constructor for `std::string` (Native builds only)
#ifdef NATIVE
  explicit DatasetReader(const std::string &filename)
  {
    file_.open(filename);
    if (!file_.is_open())
    {
      fprintf(stderr, "Failed to open file for reading: %s\n", filename.c_str());
      return;
    }
    if (!validate_header())
    {
      fprintf(stderr, "Dataset header mismatch for file: %s\n", filename.c_str());
      file_.close();
    }
  }
#endif

  // Constructor for `char*` (MCU builds only)
#ifndef NATIVE
 explicit DatasetReader(const char *filename)
  {
    file_ = fopen(filename, "r");
    if (!file_)
    {
      ENTO_DEBUG("Failed to open file on microcontroller: %s", filename);
      return;
    }
    if (!validate_header())
    {
      ENTO_DEBUG("Dataset header mismatch for file: %s", filename);
      fclose(file_);
      file_ = nullptr;
    }
  }
#endif

  ~DatasetReader()
  {
#ifdef NATIVE
    if (file_.is_open())
    {
      file_.close();
    }
#else
    if (file_)
    {
      fclose(file_);
    }
#endif
  }

  // Reads the next instance
  bool read_next(ProblemInstance &instance)
  {
    instance.clear();
#ifdef NATIVE
    std::string line;
    if (std::getline(file_, line))
    {
      return ProblemInstance::deserialize(line.c_str(), &instance);
    }
#else
    char line[256];
    if (file_ && fgets(line, sizeof(line), file_))
    {
      return ProblemInstance::deserialize(line, &instance);
    }
#endif
    return false; // End of file or error
  }

private:
  bool validate_header()
  {
#ifdef NATIVE
    std::string header;
    std::getline(file_, header);
    if (header != ProblemInstance::header())
    {
      fprintf(stderr, "Expected header: %s, Found: %s\n", ProblemInstance::header().c_str(), header.c_str());
      return false;
    }
#else
    char header[256];
    if (file_ && fgets(header, sizeof(header), file_))
    {
      // Remove trailing newline
      char *newline = strchr(header, '\n');
      if (newline) *newline = '\0';

      if (std::string(header) != ProblemInstance::header())
      {
        ENTO_DEBUG("Expected header: %s, Found: %s", ProblemInstance::header().c_str(), header);
        return false;
      }
    }
    else
    {
      ENTO_DEBUG("Failed to read dataset header.");
      return false;
    }
#endif
    return true;
  }

#ifdef NATIVE
  std::ifstream file_;
#else
  FILE *file_;
#endif
};

} // namespace EntoUtil

#endif // ENTO_UTIL_DATASET_READER_H
