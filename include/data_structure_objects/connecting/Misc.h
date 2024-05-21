#pragma once
#include <Eigen/core>
struct Propertise
{
	bool verticesSetted;

	int numEdges;
	int edgeSize;
	int totalObservations; // numEdges * edgeSize

	int numActiveVertices1; // active vertices are the vertices that are involved in the optimization
	int vertex1Size; // size of the vertex1
	int totalType1Parameters; // numActiveVertices1 * vertex1Size
	int numFixedVertices1; // fixed vertices are the vertices that are not involved in the optimization

	int numActiveVertices2; // active vertices are the vertices that are involved in the optimization
	int vertex2Size; // size of the vertex2
	int totalType2Parameters; // numActiveVertices2 * vertex2Size
	int numFixedVertices2; // fixed vertices are the vertices that are not involved in the optimization

	int totalWidth; // totalType1Parameters + totalType2Parameters

	bool isInitialized;
	bool DebugMode;
	int verbosityLevel;

	//Algorithm settings
	bool isLM;
	int maxIterations;
	int maxRepeats;

	//Robust kernel settings
	double delta;
	bool isRobust;

	bool isSparse;

	bool isMarginalized;
	bool hasFixedVertices;

	//std::vector<std::map<int,Eigen::VectorXd>> edgeToVertex2; //edgeToVertex2[edgeLocation][vertex2Index] = observationVec

	Eigen::VectorXi numOberservationsPerVertex1;
	Eigen::VectorXi numOberservationsPerVertex2;

	//for solvers usage
	double Lamda;
	bool isRepeatAttempt;


	void initialize()
	{
		totalObservations = numEdges * edgeSize; 
		totalType1Parameters = numActiveVertices1 * vertex1Size;
		totalType2Parameters = numActiveVertices2 * vertex2Size;
		totalWidth = totalType1Parameters + totalType2Parameters;
		isInitialized = true;
	}

	void setNumObservationVectors() {
		this->numOberservationsPerVertex1.resize(this->numActiveVertices1);
		this->numOberservationsPerVertex2.resize(this->numActiveVertices2);
		this->numOberservationsPerVertex1.setZero();
		this->numOberservationsPerVertex2.setZero();
	}
};