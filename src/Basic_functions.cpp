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
    double dsqr = delta * delta;
    if (e <= delta) {
        return 1;
    }
    else {
		return delta / std::sqrt(e);
	}
}

Eigen::VectorXd robustifyError(Eigen::VectorXd& errorVec, double delta,double cov){

	Eigen::VectorXd weights(errorVec.size());
    double temp = 0;
    for (int i = 0; i < errorVec.size(); i++) {
        temp = huberWeight(errorVec(i) * errorVec(i) / cov, delta);
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

Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
	Eigen::Matrix3d v_hat;
	v_hat <<  0  , -v(2),  v(1),
		     v(2),     0, -v(0),
		    -v(1),  v(0),     0;
	return v_hat;
}

/*
 * This function computes the exponential map of a 6D vector xi
 * * and returns the corresponding 4x4 matrix
*/
Matrix4d expSE3Map(const Vector6d& xi) {
    // Extract rotation (omega) and translation (v) from xi
    Eigen::Vector3d omega = xi.tail<3>();
    Eigen::Vector3d v = xi.head<3>();

    double theta = omega.norm(); // Magnitude of omega
    Eigen::Matrix3d omega_hat = hat(omega); // Skew-symmetric matrix for omega
    Eigen::Matrix3d R; // Rotation matrix
    Eigen::Matrix3d V; // Jacobian of the translation

    if (theta < 1e-10) {
        // For small theta, use approximations
        R = Eigen::Matrix3d::Identity() + omega_hat;
        V = Eigen::Matrix3d::Identity() + 0.5 * omega_hat;
    }
    else {
        // For larger theta, compute exact values
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double one_minus_cos_theta = 1.0 - cos_theta;
        R = Eigen::Matrix3d::Identity() + sin_theta / theta * omega_hat +
            one_minus_cos_theta / (theta * theta) * omega_hat * omega_hat;

        V = Eigen::Matrix3d::Identity() + one_minus_cos_theta / (theta * theta) * omega_hat +
            (theta - sin_theta) / (theta * theta * theta) * omega_hat * omega_hat;
    }

    // Construct the SE3 matrix
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = V * v;

    return T;
}


/*
 * This function computes the logarithmic map of a 4x4 matrix T
 * * and returns the corresponding 6D vector
 */
Vector6d logMapSE3(const Matrix4d& T) {
    Vector6d upsilon_omega;
    auto translation = T.block<3, 1>(0, 3);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);

    // Compute the angle theta and rotation axis
    double theta = acos(std::min(std::max((R.trace() - 1) / 2.0, -1.0), 1.0));
    Eigen::Vector3d omega;
    if (theta < 1e-6) {
        omega = Eigen::Vector3d::Zero();
    }
    else {
        double scale = theta / (2.0 * sin(theta));
        omega << (R(2, 1) - R(1, 2)) * scale,
            (R(0, 2) - R(2, 0))* scale,
            (R(1, 0) - R(0, 1))* scale;
    }

    Eigen::Matrix3d Omega = Eigen::Matrix3d::Zero();
    Omega(0, 1) = -omega(2);
    Omega(0, 2) = omega(1);
    Omega(1, 0) = omega(2);
    Omega(1, 2) = -omega(0);
    Omega(2, 0) = -omega(1);
    Omega(2, 1) = omega(0);

    Eigen::Matrix3d V_inv;
    if (theta < 1e-6) {
        V_inv = Eigen::Matrix3d::Identity() - 0.5 * Omega + (1.0 / 12.0) * Omega * Omega;
    }
    else {
        double half_theta = 0.5 * theta;
        V_inv = Eigen::Matrix3d::Identity();
        V_inv -= 0.5 * Omega;
        V_inv += (1 - theta * cos(half_theta) / (2 * sin(half_theta))) / (theta * theta) * Omega * Omega;
    }

    Eigen::Vector3d v = V_inv * translation;
    upsilon_omega.head<3>() = v;
    upsilon_omega.tail<3>() = omega;

    return upsilon_omega;
}