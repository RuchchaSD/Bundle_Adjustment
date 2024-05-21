// ReprojectionBase.h
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

/**
 * @class ReprojectionBase
 * @brief Base class for reprojection operations.
 */
class ReprojectionBase {
protected:

public:
    /**
     * @brief Default constructor.
     */
    ReprojectionBase();

    /**
     * @brief Destructor.
     */
    ~ReprojectionBase();

    /**
     * @brief Pure virtual function to reproject estimates.
     * @param est1 Estimate 1.
     * @param est2 Estimate 2.
     * @param output Output vector for the reprojected values.
     */
    virtual void reproject(const Eigen::Map<Eigen::VectorXd>& est1, const Eigen::Map<Eigen::VectorXd>& est2, Eigen::VectorXd& output) = 0;
};