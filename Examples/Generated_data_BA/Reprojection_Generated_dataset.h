//Reprojection_Generated_dataset.h
#include <iostream>
#include "ReprojectionBase.h"

/**
 * @brief Class for reprojection using a generated dataset.
 *
 * This class provides a reprojection function specific to a generated dataset.
 */
class Reprojection_generated_dataset : public ReprojectionBase
{
public:
    Reprojection_generated_dataset() {};
    ~Reprojection_generated_dataset() {};

    /**
     * @brief Perform reprojection using the provided estimates.
     *
     * This method computes the reprojection using the provided estimates and outputs the result.
     *
     * @param est1 Estimate 1 as a map of Eigen vector.
     * @param est2 Estimate 2 as a map of Eigen vector.
     * @param output Output Eigen vector to store the result.
     */
    void reproject(const Eigen::Map<Eigen::VectorXd>& est1, const Eigen::Map<Eigen::VectorXd>& est2, Eigen::VectorXd& output)
    {
        const double& p = est1[0];
        const double& q = est1[1];
        const double& r = est1[2];
        const double& s = est1[3];
        const double& t = est1[4];
        const double& u = est1[5];

        const double& l = est2[0];
        const double& m = est2[1];
        const double& n = est2[2];

        output.resize(2);

        output[0] = ((p + l) * (q + m) * (r + s * n) * std::log(u * u) + t) / (p * p + q * q + r * r);
        output[1] = u * (r * m + q * l + t * n) * std::log((p + s) * (p + s)) / (s * s + t * t + u * u);
    }
};
