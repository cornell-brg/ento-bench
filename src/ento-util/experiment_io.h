#ifndef EXPERIMENT_IO_H
#define EXPERIMENT_IO_H

#include "file_path_util.h"
#include <ento-util/debug.h> 
#include <ento-bench/problem.h>

#ifdef NATIVE
#include <string>
#include <fstream>
#else
#include <cstring>
#include <cstdio>
#endif

#include <optional>

namespace EntoUtil
{

class ExperimentIO
{
private:
  // @TODO: Add separate headers for input and output files
  bool header_written_;
  bool header_validated_; 
  bool setup_finished_;

#ifdef NATIVE
  std::ifstream ifile_;
  std::ofstream ofile_;
#else
  FILE *ifile_ = nullptr;
  FILE *ofile_ = nullptr;
#endif

public:
  // Delete copy constructor and copy assignment operator
  ExperimentIO(const ExperimentIO&) = delete;
  ExperimentIO& operator=(const ExperimentIO&) = delete;

  // Default constructor
  ExperimentIO() : header_written_(false), header_validated_(false), setup_finished_(false),
#ifdef NATIVE
                   ifile_(), ofile_()
#else
                   ifile_(nullptr), ofile_(nullptr)
#endif
  {}

  // Move constructor
  ExperimentIO(ExperimentIO&& other) noexcept
    : header_written_(other.header_written_),
      header_validated_(other.header_validated_),
      setup_finished_(setup_finished_)
  {
#ifdef NATIVE
    ifile_ = std::move(other.ifile_);
    ofile_ = std::move(other.ofile_);
#else
    ifile_ = other.ifile_;
    ofile_ = other.ofile_;
    other.ifile_ = nullptr;
    other.ofile_ = nullptr;
#endif
  }

  // Move assignment operator
  ExperimentIO& operator=(ExperimentIO&& other) noexcept
  {
    if (this != &other)
    {
      header_written_ = other.header_written_;
      header_validated_ = other.header_validated_;
      setup_finished_ = other.setup_finished_;

#ifdef NATIVE
      ifile_ = std::move(other.ifile_);
      ofile_ = std::move(other.ofile_);
#else
      if (ifile_) fclose(ifile_);
      if (ofile_) fclose(ofile_);

      ifile_ = other.ifile_;
      ofile_ = other.ofile_;
      other.ifile_ = nullptr;
      other.ofile_ = nullptr;
#endif
    }
    return *this;
  }

  // Constructor for `std::string` (Native builds only)
#ifdef NATIVE
  explicit ExperimentIO(const std::optional<std::string>& input_filepath = std::nullopt,
                        const std::optional<std::string>& output_filepath = std::nullopt)
    : header_written_(false), header_validated_(false), setup_finished_(false)
  {
    if (input_filepath && !input_filepath->empty())
    {
      std::string resolved_input = resolve_path(*input_filepath);
      ifile_.open(resolved_input);
      if (!ifile_.is_open())
      {
        ENTO_DEBUG("Failed to input file: %s\n", input_filepath->c_str());
        return;
      }
    }
    if (output_filepath && !output_filepath->empty())
    {
      std::string resolved_output = resolve_path(*output_filepath);
      ofile_.open(resolved_output, std::ios::out | std::ios::trunc);
      if (!ofile_.is_open())
      {
        ENTO_DEBUG("Failed to open output file: %s\n", output_filepath->c_str());
        return;
      }
    }
  }
#else
  // Constructor for `char*` (MCU builds only)
  explicit ExperimentIO(const char *input_filepath,
                        const char *output_filepath)
  {
    char resolved_input[MAX_PATH];
    char resolved_output[MAX_PATH];

    if (input_filepath && input_filepath[0] != '\0')
    {
      resolve_path(input_filepath, resolved_input, sizeof(resolved_input));
      ifile_ = fopen(resolved_input, "r");
      if (!ifile_)
      {
        ENTO_DEBUG("Failed to open input file: %s\n", resolved_input);
        printf("Failed to open input file: %s\n", input_filepath);
        printf("Failed to open resolved input file: %s\n", resolved_input);
      }
    }
    if (output_filepath && output_filepath[0] != '\0')
    {
      resolve_path(output_filepath, resolved_output, sizeof(resolved_output));
      ofile_ = fopen(resolved_output, "w");
      if (!ofile_)
      {
        ENTO_DEBUG("Failed to open output file: %s\n", resolved_output);
      }
    }
  }
#endif

  ~ExperimentIO()
  {
#ifdef NATIVE
    if (ifile_.is_open()) ifile_.close();
    if (ofile_.is_open()) ofile_.close();
#else
    if (ifile_) fclose(ifile_);
    if (ofile_) fclose(ofile_);
#endif
  }

  // Writes a single problem instance
  template <EntoBench::ProblemConcept Problem>
  void write_results(const Problem &problem_instance)
  {
    if (!ofile_)
    {
      ENTO_DEBUG("Attempted to write to an unopened file.");
    }
    if (!header_written_)
    {
      write_output_header<Problem>();
      header_written_ = true;
    }
#ifdef NATIVE
    ofile_ << problem_instance.serialize() << "\n";
#else
      fprintf(ofile_, "%s\n", problem_instance.serialize());
#endif
  }

  // Reads the next instance
  template <EntoBench::ProblemConcept Problem>
  bool read_next(Problem &problem_instance)
  {
    __asm__ volatile("" ::: "memory");
    ENTO_DEBUG("ExperimentIO: read_next called");
    
    if (!header_validated_)
    {
      ENTO_DEBUG("ExperimentIO: validating header");
      if (!validate_input_header<Problem>())
      {
        ENTO_DEBUG("ExperimentIO: header validation failed");
#ifdef NATIVE
        ifile_.close();
#else
        fclose(ifile_);
#endif
        return false;
      }
      ENTO_DEBUG("ExperimentIO: header validation passed");
    }

    if constexpr (Problem::RequiresSetup_)
    {
      if (!setup_finished_)
      {
        ENTO_DEBUG("ExperimentIO: setting up problem");
        if (!setup_problem<Problem>(problem_instance))
        {
          ENTO_DEBUG("ExperimentIO: problem setup failed");
#ifdef NATIVE
          ifile_.close();
#else
          fclose(ifile_);
#endif
        }
      } 
    }

    problem_instance.clear();
    ENTO_DEBUG("ExperimentIO: cleared problem instance");
    
#ifdef NATIVE
    std::string line;
    if (std::getline(ifile_, line))
    {
      ENTO_DEBUG("ExperimentIO: read line (native): %.50s", line.c_str());
      bool result = problem_instance.deserialize(line);
      ENTO_DEBUG("ExperimentIO: deserialize result: %d", result);
      return result;
    }
    else
    {
      ENTO_DEBUG("ExperimentIO: failed to read line (native)");
    }
#else
    char line[4096];
    if (ifile_ && fgets(line, sizeof(line), ifile_))
    {
      ENTO_DEBUG("ExperimentIO: read line (MCU): %.50s", line);
      bool result = problem_instance.deserialize(line);
      ENTO_DEBUG("ExperimentIO: deserialize result: %d", result);
      return result;
    }
    else
    {
      ENTO_DEBUG("ExperimentIO: failed to read line (MCU)");
    }
#endif
    __asm__ volatile("" ::: "memory");
    ENTO_DEBUG("ExperimentIO: returning false (end of file)");
    return false; // End of file or error
  }

  bool is_input_open()
  {
#ifdef NATIVE
    if (ifile_.is_open())
      return true;
    else
      return false;
#else
    if (ifile_)
      return true;
    else
      return false;
#endif
  }
  
  bool is_output_open()
  {
#ifdef NATIVE
    if (ofile_.is_open())
      return true;
    else
      return false;
#else
    if (ofile_)
      return true;
    else
      return false;
#endif
  }

private:
  template <EntoBench::ProblemConcept Problem>
  void write_output_header()
  {
    if constexpr (Problem::SaveResults_)
    {
#ifdef NATIVE
      ofile_ << Problem::header() << "\n";
#else
      if (ifile_)
      {
        fprintf(ofile_, "%s\n", Problem::header());
      }
      else
      {
        ENTO_DEBUG("Attempted to write header to an unopened file.");
      }
#endif
    }
  }


  template <EntoBench::ProblemConcept Problem>
  bool validate_input_header()
  {
    if constexpr (Problem::RequiresDataset_)
    {
#ifdef NATIVE
      std::string header;
      std::getline(ifile_, header);
      if (header != Problem::header())
      {
        ENTO_ERROR("Expected header for: %s, Found: %s\n", Problem::header(), header.c_str());
        return false;
      }
#else
      char header[256];
      if (ifile_ && fgets(header, sizeof(header), ifile_))
      {
        // Remove trailing newline
        char *newline = strchr(header, '\n');
        if (newline) *newline = '\0';

        if (strcmp(header, Problem::header()) != 0)
        {
          ENTO_DEBUG("Expected header: %s\nFound: %s", Problem::header(), header);
          return false;
        }
      }
      else
      {
        ENTO_DEBUG("Failed to read dataset header.");
        return false;
      }
#endif
      header_validated_ = true;
      return true;
    }
  }

  template <EntoBench::ProblemConcept Problem>
  bool setup_problem(Problem &problem_instance)
  {
    for ( int i = 0; i < Problem::SetupLines_; i++ ) {
#ifdef NATIVE
      std::string line;
      if (std::getline(ifile_, line))
      {
        problem_instance.deserialize(line);
      }
#else
      char line[256];
      if (ifile_ && fgets(line, sizeof(line), ifile_))
      {
        problem_instance.deserialize(line);
      }
#endif
    }
    setup_finished_ = true;
    return true;
  }

};

} // namespace EntoUtil

#endif // EXPERIMENT_IO_H

