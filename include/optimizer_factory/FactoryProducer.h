
//FactoryProducer.h
#pragma once
#include <iostream>
#include "OptimizerFactory.h"

/**
 * @brief Factory producer class to generate different types of optimizer factories.
 *
 * This class provides a method to create different optimizer factories based on the specified type.
 */
class OptimizerFactoryProducer
{
public:
    /**
     * @brief Get an optimizer factory based on the specified type.
     * @param factoryType The type of the factory.
     * @return Unique pointer to the created optimizer factory.
     */
    std::unique_ptr<OptimizerFactory> getOptimizerFactory(std::string factoryType);
};
