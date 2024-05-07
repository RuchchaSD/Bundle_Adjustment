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

class OptimizerBase : public BaseDataStructure
{
protected:
	std::unique_ptr<OptimizationAlgorithmBase> optimizerAlgorithm;
	std::unique_ptr<ProblemStructure> structure;

public:
	OptimizerBase();
	~OptimizerBase();

	virtual void SetStructure(std::unique_ptr<ProblemStructure> str) { structure = std::move(str); }
	virtual void SetOptimizer(std::unique_ptr<OptimizationAlgorithmBase> opt) { optimizerAlgorithm = std::move(opt); }

	virtual void addVertex_type1(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false) = 0;
	virtual std::shared_ptr<Eigen::VectorXd> getVertex_type1_Parameters(int id) = 0;
	virtual void addVertex_type2(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false) = 0;
	virtual std::shared_ptr<Eigen::VectorXd> getVertex_type2_Parameters(int id) = 0;
	virtual void addEdge(int id, int vertex1_id, int vertex2_id, std::shared_ptr<Eigen::VectorXd> observations, std::shared_ptr<Eigen::VectorXd> sigma) = 0;

	virtual void Optimize(int n) = 0;
};

