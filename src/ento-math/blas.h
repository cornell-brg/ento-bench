#ifndef BLAS_H
#define BLAS_H

#include <array>

template <typename Scalar, std::size_t Rows, std::size_t Cols>
class EntoMatrix
{
public:
  // Default Constructor
  constexpr EntoMatrix()
  {
    static_assert(Rows > 0 && Cols > 0, "Matrix dimensions must be greater than 0.");
    data_.fill(static_cast<Scalar>(0)); // Initialize with zeros
  }

  // Access Element
  Scalar &operator()(std::size_t row, std::size_t col)
  {
    // @TODO: Add assert capabilities that fall back to error handler for mcu builds.
    //  Probably add to ento-util/debug.h?
    //  assert(row < Rows && col < Cols && "Index out of bounds.");
    return data_[row * Cols + col];
  }

  // Const Access
  const Scalar &operator()(std::size_t row, std::size_t col) const
  {
    // @TODO: Add assert capabilities that fall back to error handler for mcu builds.
    //  Probably add to ento-util/debug.h?
    //  assert(row < Rows && col < Cols && "Index out of bounds.");
    return data_[row * Cols + col];
  }

  // Fill the matrix with a value
  void fill(Scalar value)
  {
    data_.fill(value);
  }

  // Copy a matrix
  constexpr void copy(const EntoMatrix &other)
  {
    static_assert(Rows == Rows && Cols == Cols, "Matrix dimensions must match for copy.");
    data_ = other.data_;
  }

  // Addition of two matrices
  constexpr EntoMatrix operator+(const EntoMatrix &other) const
  {
    EntoMatrix result;
    for (std::size_t i = 0; i < data_.size(); ++i)
    {
      result.data_[i] = data_[i] + other.data_[i];
    }
    return result;
  }

  // Subtraction of two matrices
  constexpr EntoMatrix operator-(const EntoMatrix &other) const
  {
    EntoMatrix result;
    for (std::size_t i = 0; i < data_.size(); ++i)
    {
      result.data_[i] = data_[i] - other.data_[i];
    }
    return result;
  }

  // Scalar multiplication
  constexpr EntoMatrix operator*(Scalar scalar) const
  {
    EntoMatrix result;
    for (std::size_t i = 0; i < data_.size(); ++i)
    {
      result.data_[i] = data_[i] * scalar;
    }
    return result;
  }

  // EntoMatrix-vector multiplication
  template <std::size_t VecSize>
  constexpr std::array<Scalar, Rows> operator*(const std::array<Scalar, VecSize> &vec) const
  {
    static_assert(Cols == VecSize, "Matrix columns must match vector size.");
    std::array<Scalar, Rows> result{};
    for (std::size_t row = 0; row < Rows; ++row)
    {
      for (std::size_t col = 0; col < Cols; ++col)
      {
        result[row] += (*this)(row, col) * vec[col];
      }
    }
    return result;
  }

  // EntoMatrix-matrix multiplication
  template <std::size_t OtherCols>
  constexpr EntoMatrix<Scalar, Rows, OtherCols> operator*(const EntoMatrix<Scalar, Cols, OtherCols> &other) const
  {
    EntoMatrix<Scalar, Rows, OtherCols> result;
    for (std::size_t row = 0; row < Rows; ++row)
    {
      for (std::size_t col = 0; col < OtherCols; ++col)
      {
        for (std::size_t k = 0; k < Cols; ++k)
        {
          result(row, col) += (*this)(row, k) * other(k, col);
        }
      }
    }
    return result;
  }

  // Transpose the matrix
  constexpr EntoMatrix<Scalar, Cols, Rows> transpose() const
  {
    EntoMatrix<Scalar, Cols, Rows> result;
    for (std::size_t row = 0; row < Rows; ++row)
    {
      for (std::size_t col = 0; col < Cols; ++col)
      {
        result(col, row) = (*this)(row, col);
      }
    }
    return result;
  }

  // Size information
  constexpr std::size_t rows() const { return Rows; }
  constexpr std::size_t cols() const { return Cols; }

private:
  std::array<Scalar, Rows * Cols> data_;
};

#endif
