// RobustKernels.h
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "BaseDataStructure.h"

/**
 * @class RobustKernelBase
 * @brief Base class for robust kernel functions.
 */
class RobustKernelBase : public BaseDataStructure {
protected:
    bool isInitialized;
    std::shared_ptr<Eigen::VectorXd> weightsVec;
    bool isUpdated_;
    double delta_;
    double sqrt_delta_;

    /**
     * @brief Calculates the weight for a given residual.
     * @param residual Residual value.
     * @return Weight value.
     */
    virtual double calculateWeight(double residual);

public:
    /**
     * @brief Default constructor.
     */
    RobustKernelBase();

    /**
     * @brief Destructor.
     */
    ~RobustKernelBase();

    /**
     * @brief Initializes the robust kernel.
     */
    virtual void initialize() override;

    /**
     * @brief Finalizes the robust kernel.
     */
    void finalize() override;

    /**
     * @brief Applies the robust kernel to residuals.
     * @param residuals Vector of residuals.
     */
    virtual void robustifyResiduals(Eigen::VectorXd& residuals);

    /**
     * @brief Applies the robust kernel to the Jacobian matrix.
     * @param J Jacobian matrix.
     */
    virtual void robustifyJacobian(Eigen::MatrixXd& J);

    /**
     * @brief Gets the vector of weights.
     * @return Shared pointer to the vector of weights.
     */
    std::shared_ptr<const Eigen::VectorXd> getWeightsVec() {
        return weightsVec;
    }

    /**
     * @brief Checks if the robust kernel is updated.
     * @return True if updated, false otherwise.
     */
    bool isUpdated() const { return isUpdated_; }

    /**
     * @brief Sets the updated state of the robust kernel.
     * @param updated Updated state to set.
     */
    void setUpdated(bool updated) { isUpdated_ = updated; }
};

/**
 * @class HuberKernel
 * @brief Huber robust kernel function.
 */
class HuberKernel : public RobustKernelBase {
protected:
    /**
     * @brief Calculates the weight for a given residual using the Huber kernel.
     * @param residual Residual value.
     * @return Weight value.
     */
    double calculateWeight(double residual) override;

public:
    /**
     * @brief Default constructor.
     */
    HuberKernel() : RobustKernelBase() {};

    /**
     * @brief Destructor.
     */
    ~HuberKernel() {};
};

