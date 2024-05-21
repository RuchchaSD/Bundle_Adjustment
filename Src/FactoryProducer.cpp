#include "FactoryProducer.h"

std::unique_ptr<OptimizerFactory> OptimizerFactoryProducer::getOptimizerFactory(std::string factoryType)
{
	if (factoryType == "Orb New") {
		return std::make_unique<OptimizerFactoryOrbSlamNew>();
	}
	else if (factoryType == "Orb Old"){
		//share the one I made before
		// return new OptimizerOrbSlamOld();
		return nullptr;
	} 
	else{
		std::cerr << "Error: OptimizerFactoryProducer::getOptimizerFactory factoryType: " << factoryType << " not recognized" << std::endl;
		return nullptr;
	}
}
  