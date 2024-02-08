#ifndef Basic_Functions_H
#define Basic_Functions_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>


Eigen::MatrixXd inverseDiagonal(const Eigen::MatrixXd& A);

double huberWeight(double e, double delta);

Eigen::VectorXd robustifyError(Eigen::VectorXd& errorVec, double delta);

void robustifyJacobianVertex(Eigen::MatrixXd& JVertex, const Eigen::VectorXd& weights);

#endif // !Basic_Functions_H