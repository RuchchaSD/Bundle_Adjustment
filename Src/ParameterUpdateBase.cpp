#include "ParameterUpdateBase.h"

ParameterUpdateBase::ParameterUpdateBase()
{
}

ParameterUpdateBase::~ParameterUpdateBase()
{
}

void ParameterUpdateBase::update(Eigen::VectorXd& parameters, const Eigen::VectorXd& update) 
{
	parameters += update;
}
