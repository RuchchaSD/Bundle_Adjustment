#pragma once
#include <Eigen/core>
#include <Eigen/Dense>
#include "DataStructureBases.h"


class ProblemStructure : public BaseDataStructure
{
protected:
	std::shared_ptr<parameterVectorBase> parameters;
	std::shared_ptr<JacobianCompressed> jacobian; // will need to change to the base class to achieve polymorphism
	std::shared_ptr<residualsCompressed> residual;// will need to change to the base class to achieve polymorphism
	std::shared_ptr<HessianOrb> Hessian;
	std::shared_ptr<bVectorOrb> bVector;
	std::shared_ptr<SolverOrb> solver;

public:
	ProblemStructure();
	~ProblemStructure();

	//setters
	void setParameters(std::shared_ptr<parameterVectorBase> parameters) { this->parameters = parameters; }
	void setJacobian(std::shared_ptr<JacobianCompressed> jacobian) { this->jacobian = jacobian; }
	void setResidual(std::shared_ptr<residualsCompressed> residual) { this->residual = residual; }
	void setHessian(std::shared_ptr<HessianOrb> Hessian) { this->Hessian = Hessian; }
	void setbVector(std::shared_ptr<bVectorOrb> bVector) { this->bVector = bVector; }
	void setSolver(std::shared_ptr<SolverOrb> solver) { this->solver = solver; }
	
	//getters
	std::shared_ptr<parameterVectorBase> getParameters() { return parameters; }
	std::shared_ptr<JacobianCompressed> getJacobian() { return jacobian; }
	std::shared_ptr<residualsCompressed> getResidual() { return residual; }
	std::shared_ptr<HessianOrb> getHessian() { return Hessian; }
	std::shared_ptr<bVectorOrb> getbVector() { return bVector; }
	std::shared_ptr<SolverOrb> getSolver() { return solver; }

	void initialize() override;
	void finalize() override;

};
 
