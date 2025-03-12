#ifndef ENTO_UTIL_DATASET_WRITER_H
#define ENTO_UTIL_DATASET_WRITER_H

#include <string>
#include <array>
#include <ento-util/debug.h> 
#include <ento-bench/problem.h>

#ifdef NATIVE
#include <fstream>
#else
#include <cstdio>
#endif

namespace EntoUtil
{

class DatasetWriter
{
public:
  // Constructor for `std::string` (Native builds only)
#ifdef NATIVE
  explicit DatasetWriter(const std::string &filename) : header_written_(false)
  {
    file_.open(filename);
    if (!file_.is_open())
    {
      ENTO_DEBUG("Failed to open file for writing: %s\n", filename.c_str());
      return;
    }
  }
#else
  // Constructor for `char*` (MCU builds only)
  explicit DatasetWriter(const char *filename)
  {
    file_ = fopen(filename, "w");
    if (!file_)
    {
      ENTO_DEBUG("Failed to open file on microcontroller: %s", filename);
      return;
    }
  }
#endif

  ~DatasetWriter()
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

  // Writes a single problem instance
  template <EntoBench::ProblemConcept Problem>
  void write(const Problem &problem_instance)
  {
    if (!file_)
    {
      ENTO_DEBUG("Attempted to write to an unopened file.");
    }
    if (!header_written_)
    {
      write_header<Problem>();
      header_written_ = true;
    }
#ifdef NATIVE
    file_ << problem_instance.serialize() << "\n";
#else
      fprintf(file_, "%s\n", instance.serialize());
#endif
  }

private:
  bool header_written_;

  template <EntoBench::ProblemConcept Problem>
  void write_header()
  {
#ifdef NATIVE
    file_ << Problem::header() << "\n";
#else
    if (file_)
    {
      fprintf(file_, "%s\n", ProblemInstance::header());
    }
    else
    {
      ENTO_DEBUG("Attempted to write header to an unopened file.");
    }
#endif
  }

#ifdef NATIVE
  std::ofstream file_;
#else
  FILE *file_;
#endif
};

} // namespace EntoUtil

#endif // ENTO_UTIL_DATASET_WRITER_H
