#pragma once
#include <Eigen/core>
#include <Eigen/Dense>
class ParameterUpdateBase
{
	protected: 

	public:
		ParameterUpdateBase();
		~ParameterUpdateBase();

		virtual void update(Eigen::VectorXd& parameters, const Eigen::VectorXd& update);
};

