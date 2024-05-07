#pragma once
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
class ReprojectionBase
{
	protected:


	public:
	ReprojectionBase();
	~ReprojectionBase();

	virtual void reproject(const Eigen::Map<Eigen::VectorXd>& est1, const Eigen::Map<Eigen::VectorXd>& est2, Eigen::VectorXd& output) = 0;
};

