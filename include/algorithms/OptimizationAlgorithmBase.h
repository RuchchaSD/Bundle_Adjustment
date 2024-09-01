// OptimizationAlgorithmBase.h
#pragma once
#include "DataStructureBases.h"
#include "ProblemStructure.h"
#include "Timer.h"



/**
 * @class OptimizationAlgorithmBase
 * @brief Base class for optimization algorithms, handling problem structures and optimization processes.
 */
class OptimizationAlgorithmBase : public BaseDataStructure {
protected:
    std::unique_ptr<ProblemStructure> structure; ///< Problem structure for the optimization algorithm.

public:
    /**
     * @brief Constructor for OptimizationAlgorithmBase.
     */
    OptimizationAlgorithmBase();

    /**
     * @brief Destructor for OptimizationAlgorithmBase.
     */
    ~OptimizationAlgorithmBase();

    /**
     * @brief Sets the problem structure.
     * @param structure Unique pointer to the problem structure.
     */
    virtual void SetProblem(std::unique_ptr<ProblemStructure> structure) { this->structure = std::move(structure); }

    /**
     * @brief Sets the structure.
     * @param str Unique pointer to the structure.
     */
    virtual void SetStructure(std::unique_ptr<ProblemStructure> str) { structure = std::move(str); }

    /**
     * @brief Optimizes the problem for a given number of iterations.
     * @param maxIterations Maximum number of iterations.
     */
    virtual void Optimize(int maxIterations) = 0;

    /**
     * @brief Optimizes the problem using default maxIterations from properties.
     */
    virtual void Optimize() {
        if (this->isInitialized)
            this->Optimize(p->maxIterations);
    }
};
