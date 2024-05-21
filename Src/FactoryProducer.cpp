//FactoryProducer.cpp
#include "FactoryProducer.h"

/**
 * @brief Get an optimizer factory based on the specified type.
 *
 * This method creates and returns an optimizer factory of the specified type.
 * Currently, it supports "Orb New" and "Orb Old" types.
 *
 * @param factoryType The type of the factory.
 * @return Unique pointer to the created optimizer factory.
 */
std::unique_ptr<OptimizerFactory> OptimizerFactoryProducer::getOptimizerFactory(std::string factoryType)
{
    if (factoryType == "Orb New") {
        return std::make_unique<OptimizerFactoryOrbSlamNew>();
    }
    else if (factoryType == "Orb Old") {
        return nullptr;
    }
    else {
        std::cerr << "Error: OptimizerFactoryProducer::getOptimizerFactory factoryType: " << factoryType << " not recognized" << std::endl;
        return nullptr;
    }
}
