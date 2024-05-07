#pragma once
#include <iostream>
#include <Eigen/core>

class Vertex
{
protected:
	int globalId;//global id that the user assigns to the vertex
	int id;//id that the graph assigns to the vertex
	bool fixed = false;
	//std::unique_ptr<Eigen::Map<Eigen::VectorXd>> parameters;
	std::shared_ptr<Eigen::VectorXd> parameters;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> parametersMap;


public:
	Vertex() : fixed(false), parameters(nullptr), parametersMap(nullptr)
	{
		this->globalId =-1;
		this->id = -1;
	}
	Vertex(std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed) : parameters(parameters), fixed(isFixed), parametersMap(nullptr)
	{
		this->globalId = -1;
		this->id = -1;
	}
	~Vertex() {}

	void setId(int id) { this->id = id; }
	int getId() { return id; }

	void setGlobalId(int gId) { this->globalId = gId; }
	int setGlobalId() { return globalId; }

	void setFixed(bool fixed) { this->fixed = fixed; }
	bool isFixed() { return fixed; }

	void setParameters(Eigen::VectorXd& parameters) { this->parameters = std::make_shared<Eigen::VectorXd>(parameters); }
	void setParameters(std::shared_ptr<Eigen::VectorXd> parameters) { this->parameters = parameters; }
	std::shared_ptr<Eigen::VectorXd> getParameters() { return parameters; }

	void setParametersMap(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> parametersMap) { 
		this->parametersMap = parametersMap; 
		*this->parametersMap = *parameters;
	}

	void updarteParameters() { *parameters = *parametersMap; }

	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getParametersMap() { return parametersMap; }

};


class edge {
protected:
	int globalId;
	int id;
	std::shared_ptr<Vertex> v1;
	std::shared_ptr<Vertex> v2;
	std::shared_ptr<Eigen::VectorXd> observations;
	std::shared_ptr<Eigen::VectorXd> covariance;
	std::shared_ptr<Eigen::VectorXd> sigma;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> observationsMap;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> covarianceMap;


public:
	edge() 
	{
		this->globalId = -1;
		this->id = -1;
	}
	~edge() {}

	void setId(int id) { this->id = id; }
	int getId() { return id; }

	void setGlobalId(int id) { this->globalId = id; }
	int getGlobalId() { return globalId; }


	void setVertices(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2) {
		this->v1 = v1;
		this->v2 = v2;
	}

	void setObservations(Eigen::VectorXd& observations) { this->observations = std::make_shared<Eigen::VectorXd>(observations); }
	void setObservations(std::shared_ptr<Eigen::VectorXd> observations) { this->observations = observations; }
	std::shared_ptr<Eigen::VectorXd> getObservations() { return observations; }

	void setCovariance(Eigen::VectorXd& covariance) { this->covariance = std::make_shared<Eigen::VectorXd>(covariance); }
	void setCovariance(std::shared_ptr<Eigen::VectorXd> covariance) { this->covariance = covariance; }
	std::shared_ptr<Eigen::VectorXd> getCovariance() { return covariance; }

	void setSigma(Eigen::VectorXd& sigma) { this->sigma = std::make_shared<Eigen::VectorXd>(sigma); }
	void setSigma(std::shared_ptr<Eigen::VectorXd> sigma) { this->sigma = sigma; }
	std::shared_ptr<Eigen::VectorXd> getSigma() { return sigma; }
	std::shared_ptr<Eigen::VectorXd> getinvSigma() { 
		std::shared_ptr<Eigen::VectorXd> invSigma = std::make_shared<Eigen::VectorXd>(sigma->size());
		*invSigma = sigma->cwiseInverse();
		return invSigma;
	}

	void setObservationsMap(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> observationsMap) { 
		this->observationsMap = observationsMap; 
		*this->observationsMap = *observations;
	}

	void setCovarianceMap(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> covarianceMap) { 
		this->covarianceMap = covarianceMap; 
		*this->covarianceMap = *covariance;
	}

	std::shared_ptr<Vertex> getVertex1() { return v1; }
	std::shared_ptr<Vertex> getVertex2() { return v2; }
};