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

double huberWeight(double e, double delta) {
	if (std::abs(e) <= delta)
		return 1;
	else
		return delta / std::abs(e);
}

Eigen::VectorXd robustifyError(Eigen::VectorXd& errorVec, double delta){

	Eigen::VectorXd weights(errorVec.size());
    double temp = 0;
    for (int i = 0; i < errorVec.size(); i++) {
        temp = huberWeight(errorVec(i), delta);
		weights(i) = temp;
        errorVec(i) = temp * errorVec(i);
	}
	return weights;
}

void robustifyJacobianVertex(Eigen::MatrixXd& JVertex, const Eigen::VectorXd& weights) {
    for (int i = 0; i < JVertex.rows(); i++) {
		JVertex.row(i) *= weights(i);
	}
}