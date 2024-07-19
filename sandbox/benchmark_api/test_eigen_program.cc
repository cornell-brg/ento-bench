
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>

#ifndef NATIVE
extern "C" void* __dso_handle = nullptr;
#endif

constexpr int MAX_ROWS = 100;
constexpr int MAX_COLS = 100;

template <typename MatrixType>
void computeAndPrintDecompositions(const MatrixType& matrix, const std::string& matrixName) {
    std::cout << "Matrix " << matrixName << ":\n" << matrix << "\n\n";

    // SVD decomposition
    Eigen::JacobiSVD<MatrixType> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << "Singular values of " << matrixName << ":\n" << svd.singularValues() << "\n\n";
    std::cout << "Matrix U of " << matrixName << ":\n" << svd.matrixU() << "\n\n";
    std::cout << "Matrix V of " << matrixName << ":\n" << svd.matrixV() << "\n\n";

    // Cholesky decomposition (only for positive definite matrices)
    if(matrix.isApprox(matrix.transpose())) {
        Eigen::LLT<MatrixType> cholesky(matrix);
        if (cholesky.info() == Eigen::Success) {
            std::cout << "Cholesky decomposition (L) of " << matrixName << ":\n" << Eigen::Matrix<typename MatrixType::Scalar, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime>(cholesky.matrixL()) << "\n\n";
        } else {
            std::cout << "Cholesky decomposition failed for " << matrixName << "\n\n";
        }
    } else {
        std::cout << "Matrix " << matrixName << " is not symmetric, skipping Cholesky decomposition.\n\n";
    }
}

int main() {
    // Different matrix sizes
    //Eigen::Matrix3d fixedMatrix3x3;
    //Eigen::Matrix4d fixedMatrix4x4;
    //Eigen::Matrix<float, 10, 10> fixedMatrix10x10;
    //Eigen::Matrix<float, 100, 100> fixedMatrix100x100;
    //Eigen::Matrix<float, 50, 50> fixedMatrix1000x1000;
    Eigen::Matrix<float, 8, 3> fixedMatrix8x3;

    // Randomly initialize fixed-size matrices
    //fixedMatrix3x3.setRandom();
    //fixedMatrix4x4.setRandom();
    //fixedMatrix10x10.setRandom();
    //fixedMatrix100x100.setRandom();
    //fixedMatrix1000x1000.setRandom();
    fixedMatrix8x3.setRandom();

    Eigen::JacobiSVD<Eigen::Matrix<float, 8, 3>, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(8, 3);
    //svd.compute(fixedMatrix8x3);
    //std::cout << "Singular values of fixed10x10 in main" << ":\n" << svd.singularValues() << "\n\n";


    auto svd_lambda = [&svd, &fixedMatrix8x3]() -> void {
      svd.compute(fixedMatrix8x3);
      std::cout << "Singular values of fixed10x10 in main" << ":\n" << svd.singularValues() << "\n\n";
    };
    svd_lambda();
    // Perform computations and print results for fixed-size matrices
    /*computeAndPrintDecompositions(fixedMatrix3x3, "3x3 matrix");
    computeAndPrintDecompositions(fixedMatrix4x4, "4x4 matrix");
    computeAndPrintDecompositions(fixedMatrix10x10, "10x10 matrix");
    computeAndPrintDecompositions(fixedMatrix100x100, "100x100 matrix");
    computeAndPrintDecompositions(fixedMatrix1000x1000, "1000x1000 matrix");*/

    // Dynamic-size matrices with maximum size defined at compile time
    //Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_ROWS, MAX_COLS> dynamicMatrix;

    // Example sizes for dynamic matrices
    /*for (int size : {5, 10, 50, 100} ) {
        dynamicMatrix.resize(size, size);
        dynamicMatrix.setRandom();
        computeAndPrintDecompositions(dynamicMatrix, std::to_string(size) + "x" + std::to_string(size) + " dynamic matrix");
    }*/

    return 0;
}
