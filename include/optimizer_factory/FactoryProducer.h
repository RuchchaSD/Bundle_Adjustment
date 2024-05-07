#pragma once
#include <iostream>
#include "OptimizerFactory.h"
class OptimizerFactoryProducer
{
public:
	std::unique_ptr<OptimizerFactory> getOptimizerFactory(std::string factoryType);
};

