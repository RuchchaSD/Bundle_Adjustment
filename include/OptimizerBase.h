// OptimizerBase.h
#pragma once
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "ReprojectionBase.h"
#include "ProblemStructure.h"
#include "Misc.h"
#include "OptimizationAlgorithmBase.h"
#include "DataStructureBases.h"
#include "Timer.h"

/**
 * @class OptimizerBase
 * @brief Base class for optimizers, managing the optimization algorithm and problem structure.
 */
class OptimizerBase : public BaseDataStructure {
protected:
    std::unique_ptr<OptimizationAlgorithmBase> optimizerAlgorithm; ///< Optimization algorithm.
    std::unique_ptr<ProblemStructure> structure; ///< Problem structure.

public:
    /**
     * @brief Constructor for OptimizerBase.
     */
    OptimizerBase();

    /**
     * @brief Destructor for OptimizerBase.
     */
    ~OptimizerBase();

    /**
     * @brief Sets the problem structure.
     * @param str Unique pointer to the problem structure.
     */
    virtual void SetStructure(std::unique_ptr<ProblemStructure> str) { structure = std::move(str); }

    /**
     * @brief Sets the optimization algorithm.
     * @param opt Unique pointer to the optimization algorithm.
     */
    virtual void SetOptimizer(std::unique_ptr<OptimizationAlgorithmBase> opt) { optimizerAlgorithm = std::move(opt); }

    /**
     * @brief Adds a vertex of type 1 to the optimizer.
     * @param id Identifier for the vertex.
     * @param parameters Parameters of the vertex.
     * @param isFixed Boolean indicating if the vertex is fixed.
     */
    virtual void addVertex_type1(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false) = 0;

    /**
     * @brief Gets the parameters of a vertex of type 1.
     * @param id Identifier for the vertex.
     * @return Shared pointer to the parameters of the vertex.
     */
    virtual std::shared_ptr<Eigen::VectorXd> getVertex_type1_Parameters(int id) = 0;

    /**
     * @brief Adds a vertex of type 2 to the optimizer.
     * @param id Identifier for the vertex.
     * @param parameters Parameters of the vertex.
     * @param isFixed Boolean indicating if the vertex is fixed.
     */
    virtual void addVertex_type2(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false) = 0;

    /**
     * @brief Gets the parameters of a vertex of type 2.
     * @param id Identifier for the vertex.
     * @return Shared pointer to the parameters of the vertex.
     */
    virtual std::shared_ptr<Eigen::VectorXd> getVertex_type2_Parameters(int id) = 0;

    /**
     * @brief Adds an edge to the optimizer.
     * @param id Identifier for the edge.
     * @param vertex1_id Identifier for the first vertex.
     * @param vertex2_id Identifier for the second vertex.
     * @param observations Observations for the edge.
     * @param sigma Information matrix for the edge.
     */
    virtual void addEdge(int id, int vertex1_id, int vertex2_id, std::shared_ptr<Eigen::VectorXd> observations, std::shared_ptr<Eigen::VectorXd> sigma) = 0;

    /**
     * @brief Optimizes the problem for a given number of iterations.
     * @param n Number of iterations.
     */
    virtual void Optimize(int n) = 0;
};
