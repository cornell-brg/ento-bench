#ifndef ENTO_UTIL_MATRIX_READER_H
#define ENTO_UTIL_MATRIX_READER_H

#include <Eigen/Dense>
#include <cstdio>
#include <cstring>
#include "ento-util/debug.h"


enum MatrixFileType {
  FLOAT = 0,
  DOUBLE,
  INT32,
  UINT32,
  UNKNOWN
};

// Convert type string to enum
MatrixFileType get_matrix_type(const char* type_str)
{
  if (strcmp(type_str, "FLOAT") == 0) return FLOAT;
  if (strcmp(type_str, "DOUBLE") == 0) return DOUBLE;
  if (strcmp(type_str, "INT32") == 0) return INT32;
  if (strcmp(type_str, "UINT32") == 0) return UINT32;
  return UNKNOWN;
}

template <typename Derived>
bool matrix_from_file(const char* file_path, Eigen::DenseBase<Derived>& matrix)
{
  FILE* file = fopen(file_path, "r");
  if (!file)
  {
    printf("Error opening file: %s\n", file_path);
    return false;
  }

  char magic[7], type_str[10];
  if (fscanf(file, "%6s %9s", magic, type_str) != 2 || std::string(magic) != "MATRIX")
  {
    printf("Invalid matrix file format: %s\n", file_path);
    fclose(file);
    return false;
  }

  MatrixFileType matrix_type = get_matrix_type(type_str);
  if (matrix_type == UNKNOWN)
  {
    printf("Unknown matrix type: %s\n", type_str);
    fclose(file);
    return false;
  }

  int rows, cols;
  if (fscanf(file, "%d %d", &rows, &cols) != 2)
  {
    printf("Error reading matrix dimensions\n");
    fclose(file);
    return false;
  }

  // If the matrix dimensions being read in don't match return false
  if ((rows != matrix.rows()) || (cols != matrix.cols()))
  {
    printf("Matrix dimensions do not match.");
    return false;
  }
  DPRINTF("File has matrix dimensions of %i rows and %i cols", rows, cols);
  //matrix.derived().resize(rows, cols);

  // Read matrix data based on type
  for (int row = 0; row < rows; ++row)
  {
    for (int col = 0; col < cols; ++col)
    {
      if (matrix_type == FLOAT)
      {
        float value;
        if (fscanf(file, "%f", &value) != 1)
        {
          printf("Error reading value at (%d, %d)\n", row, col);
          printf("Value: %d\n", value);
          fclose(file);
          return false;
        }
        matrix(row, col) = static_cast<typename Derived::Scalar>(value);
      }
      else if (matrix_type == DOUBLE)
      {
        double value;
        if (fscanf(file, "%lf", &value) != 1)
        {
          fprintf(stderr, "Error reading value at (%d, %d)\n", row, col);
          fclose(file);
          return false;
        }
        matrix(row, col) = static_cast<typename Derived::Scalar>(value);
      }
      else if (matrix_type == INT32)
      {
        int32_t value;
        if (fscanf(file, "%d", &value) != 1) {
          fprintf(stderr, "Error reading value at (%d, %d)\n", row, col);
          fclose(file);
          return false;
        }
        matrix(row, col) = static_cast<typename Derived::Scalar>(value);
      }
      else if (matrix_type == UINT32)
      {
        uint32_t value;
        if (fscanf(file, "%u", &value) != 1) {
          fprintf(stderr, "Error reading value at (%d, %d)\n", row, col);
          fclose(file);
          return false;
        }
        matrix(row, col) = static_cast<typename Derived::Scalar>(value);
      }
    }
  }

  fclose(file);
  return true;
}

#endif // ENTO_UTIL_MATRIX_READER_H
