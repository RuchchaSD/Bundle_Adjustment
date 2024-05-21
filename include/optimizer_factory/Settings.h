//Settings.h
#pragma once
#include <iostream>
#include "RobustKernels.h"
#include "ReprojectionBase.h"
#include "ParameterUpdateBase.h"

/**
 * @brief Class for configuring optimizer settings.
 *
 * This class holds various settings and parameters required for configuring the optimizer.
 * It includes options for robust kernels, reprojection functions, parameter updates, and several
 * other configurations related to the optimization process.
 */
class OptimizerSettings
{
protected:
    bool isValid = false; ///< Indicates if the settings are valid.

    std::shared_ptr<RobustKernelBase> robustKernel; ///< Pointer to the robust kernel.
    std::shared_ptr<ReprojectionBase> reprojection; ///< Pointer to the reprojection function.
    std::unique_ptr<ParameterUpdateBase> parameterUpdate; ///< Unique pointer to the parameter update.

public:
    int vertexType1Size = 0; ///< Size of vertex type 1.
    int vertexType2Size = 0; ///< Size of vertex type 2.
    int edgeSize = 0; ///< Size of the edge.

    std::string Algorithm = "LM"; ///< Algorithm type: "LM" (Levenberg-Marquardt) or "GN" (Gauss-Newton).
    int MaxIterations = 100; ///< Maximum number of iterations for the optimizer.
    int maxRepeats = 10; ///< Maximum number of repeats after failing to converge in a single iteration.

    bool Robust = true; ///< Flag to use a robust cost function.
    double RobustParameter = 10; ///< Parameter for the robust cost function.
    std::string RobustType = "Huber"; ///< Type of robust kernel: "Huber", "Cauchy", "PseudoHuber", or "custom".

    int VerbosityLvl = 1; ///< Verbosity level for output.
    bool DebugMode = false; ///< Debug mode for more checks inside operations.

    bool isSparse = true; ///< Flag to use sparse matrices.
    bool isMalginalized = false; ///< Flag to marginalize the second vertex type.
    bool isFixedAvailable = false; ///< Flag to indicate if fixed vertices are available.
    bool needRemove = false; ///< Flag to indicate if some vertices need to be removed.

    OptimizerSettings() = default;

    /**
     * @brief Check if the settings are valid.
     * @return True if the settings are valid, otherwise false.
     */
    bool getIsValid() const { return isValid; }

    /**
     * @brief Set the robust kernel.
     * @param robustKernel Pointer to the robust kernel.
     */
    void setRobustKernel(std::shared_ptr<RobustKernelBase> robustKernel) { this->robustKernel = robustKernel; }

    /**
     * @brief Get the robust kernel.
     * @return Pointer to the robust kernel.
     */
    std::shared_ptr<RobustKernelBase> getRobustKernel() const { return robustKernel; }

    /**
     * @brief Set the reprojection function.
     * @param reprojection Pointer to the reprojection function.
     */
    void setReprojection(std::shared_ptr<ReprojectionBase> reprojection) { this->reprojection = reprojection; }

    /**
     * @brief Get the reprojection function.
     * @return Pointer to the reprojection function.
     */
    std::shared_ptr<ReprojectionBase> getReprojection() const { return reprojection; }

    /**
     * @brief Set the parameter update function.
     * @param parameterUpdate Unique pointer to the parameter update function.
     */
    void setParameterUpdate(std::unique_ptr<ParameterUpdateBase> parameterUpdate) { this->parameterUpdate = std::move(parameterUpdate); }

    /**
     * @brief Get the parameter update function.
     * @return Unique pointer to the parameter update function.
     */
    std::unique_ptr<ParameterUpdateBase> getParameterUpdate() { return std::move(parameterUpdate); }

    /**
     * @brief Validate the settings.
     * @return True if the settings are valid, otherwise false.
     */
    bool validate();
};
