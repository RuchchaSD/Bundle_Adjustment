//OptimizerFactory.h
#pragma once
#include <iostream>
#include "OptimizerBase.h"
#include "OptimizerOrbSlam.h"
#include "Settings.h"

/**
 * @brief Factory class for creating optimizers.
 *
 * This class provides an interface for creating different types of optimizers based on the provided settings.
 * 
 * Example usage:
 * @code
 * #include "OptimizerFactory.h"
 * #include "FactoryProducer.h"
 *
 * int main() {
 *     // Create an instance of the optimizer factory producer
 *     OptimizerFactoryProducer factoryProducer;
 *
 *     // Get the optimizer factory
 *     auto& optimizerFactory = factoryProducer.getOptimizerFactory("ORB New");
 * 
 *     //make Settings object
 *     std::unique_ptr<Settings> settings = makeSettings();
 * 
 *     // move the settings object to the optimizer factory to create the optimizer
 *     std::unique_ptr<OptimizerBase> optimizerAlgorithm = optimizerFactory->getOptimizer(std::move(settings));
 *
 *     // add vertices and edges to the optimizer
 *      optimizerAlgorithm->addVertex_type1(index, parameters, isFixed);
 *      optimizerAlgorithm->addVertex_type2(index, parameters, isFixed);
 *      optimizerAlgorithm->addEdge(index, vertex_1_index, vertex_2_index, Observations, sigmaValues);
 *
 * 
 *     //optimize 
 *     optimizer->optimize();
 * 
 *     //get optimized parameters
 *     optimizerAlgorithm->getVertex_type1_Parameters(index);
 *     optimizerAlgorithm->getVertex_type2_Parameters(index);
 * 
 *     return 0;
 * }
 * @endcode
 *
 * @example Generated_dataSet_BA.cpp
 */
class OptimizerFactory
{
public:
    OptimizerFactory() {};
    ~OptimizerFactory() {};

    /**
     * @brief Create an optimizer based on the provided settings.
     * @param settings Unique pointer to the optimizer settings.
     * @return Unique pointer to the created optimizer.
     */
    virtual std::unique_ptr<OptimizerBase> getOptimizer(std::unique_ptr<OptimizerSettings> settings) = 0;
};

/**
 * @brief Factory class for creating ORB-SLAM optimizers.
 *
 * This class creates instances of ORB-SLAM optimizers based on the provided settings.
 */
class OptimizerFactoryOrbSlamNew : public OptimizerFactory
{
public:
    OptimizerFactoryOrbSlamNew() {};
    ~OptimizerFactoryOrbSlamNew() {};

    /**
     * @brief Create an ORB-SLAM optimizer based on the provided settings.
     * @param settings Unique pointer to the optimizer settings.
     * @return Unique pointer to the created ORB-SLAM optimizer.
     */
    std::unique_ptr<OptimizerBase> getOptimizer(std::unique_ptr<OptimizerSettings> settings) override;
};
