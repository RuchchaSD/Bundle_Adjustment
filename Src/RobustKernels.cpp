#include "RobustKernels.h"

RobustKernelBase::RobustKernelBase() : BaseDataStructure()
{
	this->delta_ = 0.0;
	this->sqrt_delta_ = 0.0;
	weightsVec = std::make_shared<Eigen::VectorXd>();
	isUpdated_ = false;

}

RobustKernelBase::~RobustKernelBase()
{
}

void RobustKernelBase::initialize()
{	
#ifndef NDEBUG
	assert(p->delta > 0.0 || isInitialized);
#endif // !NDEBUG

	this->delta_ = p->delta;
	this->sqrt_delta_ = sqrt(p->delta);

	this->weightsVec->resize(p->totalObservations);
	isInitialized = true;
	isUpdated_ = false;
}

void RobustKernelBase::finalize()
{
	if (isInitialized)
	{
		weightsVec.reset();
		isInitialized = false;
	}
	else {
		std::cerr << "RobustKernelBase::finalize: Not initialized" << std::endl;
	}
}

void RobustKernelBase::robustifyResiduals(Eigen::VectorXd& residuals)
{
	for (int i = 0; i < residuals.size(); i++) {
		(*this->weightsVec)[i] = calculateWeight(residuals[i]);
		residuals[i] *= (*this->weightsVec)[i];
	}
}

void RobustKernelBase::robustifyJacobian(Eigen::MatrixXd& J)
{
	isUpdated_ = true;
	for (int i = 0; i < J.rows(); i++) {
		J.row(i) *= (*this->weightsVec)[i];
	}
}

double RobustKernelBase::calculateWeight(double residual)
{
	return 1.0;
}



double HuberKernel::calculateWeight(double residual)
{
	residual = abs(residual);
	if (residual < this->sqrt_delta_) {
		return 1.0;
	}
	else {
		return this->delta_ / residual;
	}
}


