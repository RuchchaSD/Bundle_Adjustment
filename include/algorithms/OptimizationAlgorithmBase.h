#pragma once
#include "DataStructureBases.h"
#include "ProblemStructure.h"
#include "Timer.h"

class OptimizationAlgorithmBase : public BaseDataStructure
{
protected:
	std::unique_ptr<ProblemStructure> structure;

public:
	OptimizationAlgorithmBase();
	~OptimizationAlgorithmBase();

	virtual void SetProblem(std::unique_ptr<ProblemStructure> structure) { this->structure = std::move(structure); }
	virtual void SetStructure(std::unique_ptr<ProblemStructure> str) { structure = std::move(str); }
	//use initialize to set up temporary data structures and validate the settings given by the user
	virtual void Optimize(int maxIterations) = 0;
	virtual void Optimize() {
		if(this->isInitialized)
			this->Optimize(p->maxIterations);
	}

};