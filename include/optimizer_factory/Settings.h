#pragma once
#include <iostream>
#include "RobustKernels.h"
#include "ReprojectionBase.h"
#include "ParameterUpdateBase.h"

class OptimizerSettings
{
protected:
	bool isValid = false;

	std::shared_ptr<RobustKernelBase> robustKernel;
	std::shared_ptr<ReprojectionBase> reprojection;
	std::unique_ptr<ParameterUpdateBase> parameterUpdate;


public:
	int vertexType1Size = 0;
	int vertexType2Size = 0;
	int edgeSize = 0;



	std::string  Algorithm = "LM"; // "LM" or "GN
	int          MaxIterations = 100; // Maximum number of iterations for optimizerAlgorithm
	int maxRepeats = 10; // Maximum number of repeats after failing to converge in a single iteration
	
	bool         Robust = true; // Use robust cost function
	double       RobustParameter = 10; // Parameter for robust cost function
	std::string RobustType = "Huber"; // "Huber" implemented | "Cauchy" or "PseudoHuber" not implemented yet, "custom" for custom robust kernel

	int         VerbosityLvl = 1; // 0: No output | 1: Basic output after optimization and initilization | 2: 1 + output after each iteration | 3: 2 + output with more details | 4: 3 + output with debug information
	bool DebugMode = false; // Debug mode for more checks inside operations
	//bool        UseAnalyticJacobian = true; // Use analytic jacobian if available | still not implemented

	bool isSparse = true; // Use sparse matrices of the data structure | only sparse is implemented yet
	bool isMalginalized = false; // Marginalize the second vertices type
	bool isFixedAvailable = false; // tell if the fixed vertices are available
	bool needRemove = false; // tell if some vertices need to be removed - makes the initialization slow | still not implemented

	OptimizerSettings() = default;

	bool getIsValid() const { return isValid; }

	void setRobustKernel(std::shared_ptr<RobustKernelBase> robustKernel) { this->robustKernel = robustKernel; }
	std::shared_ptr<RobustKernelBase> getRobustKernel() const { return robustKernel; }

	void setReprojection(std::shared_ptr<ReprojectionBase> reprojection) { this->reprojection = reprojection; }
	std::shared_ptr<ReprojectionBase> getReprojection() const { return reprojection; }

	void setParameterUpdate(std::unique_ptr<ParameterUpdateBase> parameterUpdate) { this->parameterUpdate = std::move(parameterUpdate); }
	std::unique_ptr<ParameterUpdateBase> getParameterUpdate() { return std::move(parameterUpdate); }


	bool validate();

};