#ifndef Basic_Functions_H
#define Basic_Functions_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;


Eigen::MatrixXd inverseDiagonal(const Eigen::MatrixXd& A);

double huberWeight(double e, double delta);

Eigen::VectorXd robustifyError(Eigen::VectorXd& errorVec, double delta, double cov);

void robustifyJacobianVertex(Eigen::MatrixXd& JVertex, const Eigen::VectorXd& weights);

Eigen::Matrix3d hat(const Eigen::Vector3d& v);
Matrix4d expSE3Map(const Vector6d& xi);

Vector6d logMapSE3(const Matrix4d& T);



#endif // !Basic_Functions_H