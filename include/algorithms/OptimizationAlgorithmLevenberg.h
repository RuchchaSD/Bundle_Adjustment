// OptimizationAlgorithmLevenberg.h
#pragma once
#include "OptimizationAlgorithmBase.h"

/**
 * @class OptimizationAlgorithmLevenberg
 * @brief Implementation of the Levenberg-Marquardt optimization algorithm.
 */
class OptimizationAlgorithmLevenberg : public OptimizationAlgorithmBase {
private:
    std::shared_ptr<JacobianCompressed> jacobian; ///< Jacobian matrix.
    std::shared_ptr<ResidualsCompressed> residuals; ///< Residuals vector.
    std::shared_ptr<HessianOrb> Hessian; ///< Hessian matrix.
    std::shared_ptr<bVectorOrb> bVector; ///< 'b' vector.
    std::shared_ptr<ParameterVectorBase> parameters; ///< Parameters for the optimization.
    std::shared_ptr<SolverOrb> solver; ///< Solver for the optimization.

public:
    /**
     * @brief Constructor for OptimizationAlgorithmLevenberg.
     */
    OptimizationAlgorithmLevenberg();

    /**
     * @brief Destructor for OptimizationAlgorithmLevenberg.
     */
    ~OptimizationAlgorithmLevenberg();

    /**
     * @brief Initializes the algorithm.
     */
    void initialize() override;

    /**
     * @brief Finalizes the algorithm.
     */
    void finalize() override;

    /**
     * @brief Performs the optimization using the Levenberg-Marquardt algorithm.
     *
     * This function implements the Levenberg-Marquardt optimization algorithm. It iterates over the
     * optimization steps, adjusting the parameters and minimizing the residuals. The algorithm uses
     * a combination of gradient descent and Gauss-Newton methods, controlled by a damping factor lambda.
     *
     * @param maxIterations The maximum number of iterations to run the optimization.
     *
     * Inside the function:
     * - The function begins by checking if the optimizer has been initialized.
     * - Various parameters and thresholds for the optimization are set, including the lambda factor,
     *   update norms, and residual norms.
     * - The main optimization loop iterates up to maxIterations times, updating the parameter vectors
     *   and calculating the residuals.
     * - Within the main loop, there is an inner loop to adjust the lambda factor if the update does not
     *   lead to a reduction in residuals (controlled by the rho value).
     * - The function checks for convergence based on the norms of the update and residual vectors.
     * - If the residuals are reduced successfully, the lambda factor is decreased; otherwise, it is increased.
     * - Detailed information about the optimization process is printed if verbosity is enabled.
     *
     * Key variables:
     * - `lamda`: Damping factor controlling the blend between gradient descent and Gauss-Newton methods.
     * - `b_max`: Maximum coefficient of the b vector, used for convergence checking.
     * - `update_norm`: Norm of the parameter update vector, used for convergence checking.
     * - `rho`: Ratio used to determine if the update step was successful.
     * - `current_residuals_norm` and `prev_residuals_norm`: Norms of the residual vectors before and after updates.
     */
    void Optimize(int maxIterations) override;
};
