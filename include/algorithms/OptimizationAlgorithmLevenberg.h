#pragma once
#include "OptimizationAlgorithmBase.h"
class OptimizationAlgorithmLevenberg : public OptimizationAlgorithmBase
{
private:
	std::shared_ptr<JacobianCompressed> jacobian;
	std::shared_ptr<residualsCompressed> residuals;
	std::shared_ptr<HessianOrb> Hessian;
	std::shared_ptr<bVectorOrb> bVector;
	std::shared_ptr<parameterVectorBase> parameters;
	std::shared_ptr<SolverOrb> solver;
public:
	OptimizationAlgorithmLevenberg();
	~OptimizationAlgorithmLevenberg();

	void initialize() override;
	void finalize() override;
	void Optimize(int maxIterations) override;
};

 