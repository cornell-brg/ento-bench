#ifndef ENTO_UTIL_DATASET_READER_H
#define ENTO_UTIL_DATASET_READER_H

#include <string>
#include <ento-util/debug.h> // For ENTO_DEBUG
#include <ento-bench/problem.h>

#ifdef NATIVE
#include <fstream>
#else
#include <cstdio>
#endif

namespace EntoUtil
{

class DatasetReader
{
public:
  // Constructor for `std::string` (Native builds only)
#ifdef NATIVE
  explicit DatasetReader(const std::string &filename) : header_validated_(false)
  {
    ENTO_DEBUG("DatasetReader opening file: %s", filename.c_str());
    file_.open(filename);
    if (!file_.is_open())
    {
      ENTO_DEBUG("Failed to open file for reading: %s\n", filename.c_str());
      return;
    }
  }
#else
  // Constructor for `char*` (MCU builds only)
  explicit DatasetReader(const char *filename) : header_validate_(false)
  {
    ENTO_DEBUG("DatasetReader opening file: %s", filename);
    file_ = fopen(filename, "r");
    if (!file_)
    {
      ENTO_DEBUG("Failed to open file on microcontroller: %s", filename);
      return;
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
  template <EntoBench::ProblemConcept Problem>
  bool read_next(Problem &problem_instance)
  {
    if (!header_validated_)
    {

      if (!validate_header<Problem>())
      {
        ENTO_DEBUG("Dataset header mismatch!");
#ifdef NATIVE
        file_.close();
#else
        fclose(file_);
#endif
        return false;
      }
    }

    problem_instance.clear();
#ifdef NATIVE
    std::string line;
    if (std::getline(file_, line))
    {
      return problem_instance.deserialize(line.c_str());
    }
#else
    char line[256];
    if (file_ && fgets(line, sizeof(line), file_))
    {
      return problem_instance.deserialize(line);
    }
#endif
    return false; // End of file or error
  }

private:
  bool header_validated_; 

  template <EntoBench::ProblemConcept Problem>
  bool validate_header()
  {
#ifdef NATIVE
    std::string header;
    std::getline(file_, header);
    if (header != Problem::header())
    {
      ENTO_DEBUG("Expected header: %s, Found: %s\n", Problem::header(), header.c_str());
      return false;
    }
#else
    char header[256];
    if (file_ && fgets(header, sizeof(header), file_))
    {
      // Remove trailing newline
      char *newline = strchr(header, '\n');
      if (newline) *newline = '\0';

      if (std::string(header) != Problem::header())
      {
        ENTO_DEBUG("Expected header: %s, Found: %s", Problem::header(), header);
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
