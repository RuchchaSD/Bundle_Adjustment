#pragma once
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "BaseDataStructure.h"

class RobustKernelBase : public BaseDataStructure
{ 
protected:
	bool isInitialized;
	std::shared_ptr<Eigen::VectorXd> weightsVec;

	bool isUpdated_;

	double delta_;
	double sqrt_delta_;
	virtual double calculateWeight(double residual);

public:
	RobustKernelBase();
	~RobustKernelBase();

	//virtual void initialize(const Eigen::VectorXi data,double delta);
	//virtual void initialize(const int totalNumOfResiduals);
	virtual void initialize() override;
	void finalize() override;


	virtual void robustifyResiduals(Eigen::VectorXd& residuals) ;
	virtual void robustifyJacobian(Eigen::MatrixXd& J);
	std::shared_ptr<const Eigen::VectorXd> getWeightsVec() { 
		return weightsVec; 
	}

	bool isUpdated() const { return isUpdated_; }
	void setUpdated(bool updated) { isUpdated_ = updated; }
};

class HuberKernel : public RobustKernelBase 
{
protected:
	double calculateWeight(double residual) override;

public:
	HuberKernel() : RobustKernelBase() {};
	~HuberKernel() {};
};