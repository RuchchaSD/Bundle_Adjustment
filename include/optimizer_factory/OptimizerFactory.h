#pragma once
#include <iostream>
#include "OptimizerBase.h"
#include "OptimizerOrbSlam.h"
#include "Settings.h"

class OptimizerFactory
{
protected:


public:
	OptimizerFactory() {};
	~OptimizerFactory() {};


	virtual std::unique_ptr<OptimizerBase> getOptimizer(std::unique_ptr<OptimizerSettings> settings) = 0;

};


class OptimizerFactoryOrbSlamNew : public OptimizerFactory
{
private:

public:
	OptimizerFactoryOrbSlamNew() {};
	~OptimizerFactoryOrbSlamNew() {};

	std::unique_ptr<OptimizerBase> getOptimizer(std::unique_ptr<OptimizerSettings> settings) override;

};