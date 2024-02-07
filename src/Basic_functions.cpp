#include "Basic_functions.h"

Eigen::MatrixXd inverseDiagonal(const Eigen::MatrixXd& A){
    Eigen::MatrixXd A_inv = A; // Copy A to A_inv
    for (int i = 0; i < A.rows(); i++) {
        if (A(i, i) != 0) // Check to avoid division by zero
            A_inv(i, i) = 1 / A(i, i);
        else
            A_inv(i, i) = 0; // Optionally handle division by zero case
    }
    return A_inv; // Return by value
}