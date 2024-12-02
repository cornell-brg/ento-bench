#ifndef ENTO_UTIL_DATASET_WRITER_H
#define ENTO_UTIL_DATASET_WRITER_H

#include <string>
#include <array>

#ifdef NATIVE
#include <fstream>
#else
#include <cstdio>
#include <ento-util/debug.h> // For ENTO_DEBUG
#endif

namespace EntoUtil
{

template <typename ProblemInstance>
class DatasetWriter
{
public:
  // Constructor for `std::string` (Native builds only)
#ifdef NATIVE
  explicit DatasetWriter(const std::string &filename)
  {
    file_.open(filename);
    if (!file_.is_open())
    {
      fprintf(stderr, "Failed to open file for writing: %s\n", filename.c_str());
      return;
    }
    write_header();
  }
#endif

  // Constructor for `char*` (MCU builds only)
#ifndef NATIVE
  explicit DatasetWriter(const char *filename)
  {
    file_ = fopen(filename, "w");
    if (!file_)
    {
      ENTO_DEBUG("Failed to open file on microcontroller: %s", filename);
      return;
    }
    write_header();
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
  void write_instance(const ProblemInstance &instance)
  {
#ifdef NATIVE
    file_ << instance.serialize() << "\n";
#else
    if (file_)
    {
      fprintf(file_, "%s\n", instance.serialize().c_str());
    }
    else
    {
      ENTO_DEBUG("Attempted to write to an unopened file.");
    }
#endif
  }

private:
  void write_header()
  {
#ifdef NATIVE
    file_ << ProblemInstance::header() << "\n";
#else
    if (file_)
    {
      fprintf(file_, "%s\n", ProblemInstance::header().c_str());
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
