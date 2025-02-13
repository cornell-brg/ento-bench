#ifndef EXPERIMENT_IO_H
#define EXPERIMENT_IO_H

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
  ExperimentIO() : header_written_(false), header_validated_(false),
#ifdef NATIVE
                   ifile_(), ofile_()
#else
                   ifile_(nullptr), ofile_(nullptr)
#endif
  {}

  // Move constructor
  ExperimentIO(ExperimentIO&& other) noexcept
    : header_written_(other.header_written_),
      header_validated_(other.header_validated_)
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
    : header_written_(false), header_validated_(false)
  {
    if (input_filepath && !input_filepath->empty())
    {
      ifile_.open(*input_filepath);
      if (!ifile_.is_open())
      {
        ENTO_DEBUG("Failed to input file: %s\n", input_filepath->c_str());
        return;
      }
    }
    if (output_filepath && !output_filepath->empty())
    {
      ofile_.open(*output_filepath, std::ios::out | std::ios::trunc);
      if (!ofile_.is_open())
      {
        ENTO_DEBUG("Failed to open output file: %s\n", output_filepath->c_str());
        return;
      }
    }
  }
#else
  // Constructor for `char*` (MCU builds only)
  explicit ExperimentIO(const char *input_filepath = "",
                        const char *output_filepath = "")
  {
    if (input_filepath && input_filepath[0] != '\0')
    {
      ifile_ = fopen(input_filepath, "r");
      if (!ifile_)
      {
        ENTO_DEBUG("Failed to open input file: %s\n", input_filepath);
      }
    }
    if (output_filepath && output_filepath[0] != '\0')
    {
      ofile_ = fopen(output_filepath, "w");
      if (!ofile_)
      {
        ENTO_DEBUG("Failed to open output file: %s\n", output_filepath);
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
    if (!header_validated_)
    {
      if (!validate_input_header<Problem>())
      {
#ifdef NATIVE
        ifile_.close();
#else
        fclose(ifile_);
#endif
        return false;
      }
    }

    problem_instance.clear();
#ifdef NATIVE
    std::string line;
    if (std::getline(ifile_, line))
    {
      return problem_instance.deserialize(line);
    }
#else
    char line[256];
    if (ifile_ && fgets(line, sizeof(line), ifile_))
    {
      return problem_instance.deserialize(line);
    }
#endif
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
        ENTO_DEBUG("Expected header: %s, Found: %s\n", Problem::header(), header.c_str());
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
      header_validated_ = true;
      return true;
    }
  }

};

} // namespace EntoUtil

#endif // EXPERIMENT_IO_H

