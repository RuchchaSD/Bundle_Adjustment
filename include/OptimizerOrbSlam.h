#include "OptimizerBase.h"
#include <Eigen/Dense>
#include <Eigen/core>
#include "ORBData.h"
#include "OptimizationAlgorithmGN.h"
#include "OptimizationAlgorithmLevenberg.h"

#pragma once
class OptimizerOrbSlam : public OptimizerBase
{ 
private:
	//global vertex map is used when the user wants to get data after optimization
	std::map<int, std::shared_ptr<Vertex>> globalVerticesType1;
	std::map<int, std::shared_ptr<Vertex>> globalVerticesType2;

	//vertex map is used for internal use
	std::map<int, std::shared_ptr<Vertex>> verticesType1;
	std::map<int, std::shared_ptr<Vertex>> verticesType2;

	std::map<int, std::shared_ptr<std::map<int, std::shared_ptr<edge>>>> edges; //vertex2 id, vertex1 id 
	std::pair<int, std::shared_ptr<std::map<int, std::shared_ptr<edge>>>> *lastEdgePair;
	std::shared_ptr<edge> tempEdge;

	void addVertex(int globalId, int, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed, std::map<int, std::shared_ptr<Vertex>>& globalVertexMap, std::map<int, std::shared_ptr<Vertex>>& vertexMap);
	void makeEdge(int globalId, std::shared_ptr<Eigen::VectorXd>& observations, std::shared_ptr<Eigen::VectorXd>& sigma, int& vertex1_id, int& vertex2_id);
public:
	OptimizerOrbSlam();
	~OptimizerOrbSlam();

	void addVertex_type1(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false);
	std::shared_ptr<Eigen::VectorXd> getVertex_type1_Parameters(int id) override;
	void addVertex_type2(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false);
	std::shared_ptr<Eigen::VectorXd> getVertex_type2_Parameters(int id) override;
	void addEdge(int id, int vertex1_id, int vertex2_id, std::shared_ptr<Eigen::VectorXd> observations, std::shared_ptr<Eigen::VectorXd> sigma);
	
	void initialize() override;
	void finalize() override;
	void Optimize(int n) override;
};