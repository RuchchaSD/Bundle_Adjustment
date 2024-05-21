// ProblemStructure.h
#pragma once
#include <Eigen/core>
#include <Eigen/Dense>
#include "DataStructureBases.h"

/**
 * @class ProblemStructure
 * @brief A class that encapsulates the entire problem structure for SLAM optimization, including parameters, Jacobians, residuals, Hessians, and solvers.
 */
class ProblemStructure : public BaseDataStructure {
protected:
    std::shared_ptr<ParameterVectorBase> parameters; ///< Parameters for the optimization problem.
    std::shared_ptr<JacobianCompressed> jacobian; ///< Jacobian matrix, may need to be changed to base class for polymorphism.
    std::shared_ptr<ResidualsCompressed> residual; ///< Residual vector, may need to be changed to base class for polymorphism.
    std::shared_ptr<HessianOrb> Hessian; ///< Hessian matrix.
    std::shared_ptr<bVectorOrb> bVector; ///< 'b' vector for the optimization problem.
    std::shared_ptr<SolverOrb> solver; ///< Solver for the optimization problem.

public:
    /**
     * @brief Constructor for ProblemStructure.
     */
    ProblemStructure();

    /**
     * @brief Destructor for ProblemStructure.
     */
    ~ProblemStructure();

    // Setters

    /**
     * @brief Sets the parameters for the optimization problem.
     * @param parameters Shared pointer to the parameter vector base.
     */
    void setParameters(std::shared_ptr<ParameterVectorBase> parameters) { this->parameters = parameters; }

    /**
     * @brief Sets the Jacobian for the optimization problem.
     * @param jacobian Shared pointer to the Jacobian.
     */
    void setJacobian(std::shared_ptr<JacobianCompressed> jacobian) { this->jacobian = jacobian; }

    /**
     * @brief Sets the residual for the optimization problem.
     * @param residual Shared pointer to the residual.
     */
    void setResidual(std::shared_ptr<ResidualsCompressed> residual) { this->residual = residual; }

    /**
     * @brief Sets the Hessian for the optimization problem.
     * @param Hessian Shared pointer to the Hessian.
     */
    void setHessian(std::shared_ptr<HessianOrb> Hessian) { this->Hessian = Hessian; }

    /**
     * @brief Sets the 'b' vector for the optimization problem.
     * @param bVector Shared pointer to the 'b' vector.
     */
    void setbVector(std::shared_ptr<bVectorOrb> bVector) { this->bVector = bVector; }

    /**
     * @brief Sets the solver for the optimization problem.
     * @param solver Shared pointer to the solver.
     */
    void setSolver(std::shared_ptr<SolverOrb> solver) { this->solver = solver; }

    // Getters

    /**
     * @brief Gets the parameters for the optimization problem.
     * @return Shared pointer to the parameter vector base.
     */
    std::shared_ptr<ParameterVectorBase> getParameters() { return parameters; }

    /**
     * @brief Gets the Jacobian for the optimization problem.
     * @return Shared pointer to the Jacobian.
     */
    std::shared_ptr<JacobianCompressed> getJacobian() { return jacobian; }

    /**
     * @brief Gets the residual for the optimization problem.
     * @return Shared pointer to the residual.
     */
    std::shared_ptr<ResidualsCompressed> getResidual() { return residual; }

    /**
     * @brief Gets the Hessian for the optimization problem.
     * @return Shared pointer to the Hessian.
     */
    std::shared_ptr<HessianOrb> getHessian() { return Hessian; }

    /**
     * @brief Gets the 'b' vector for the optimization problem.
     * @return Shared pointer to the 'b' vector.
     */
    std::shared_ptr<bVectorOrb> getbVector() { return bVector; }

    /**
     * @brief Gets the solver for the optimization problem.
     * @return Shared pointer to the solver.
     */
    std::shared_ptr<SolverOrb> getSolver() { return solver; }

    /**
     * @brief Initializes the problem structure, setting properties and timers, and initializing all components.
     */
    void initialize() override;

    /**
     * @brief Finalizes the problem structure, finalizing all components.
     */
    void finalize() override;
};