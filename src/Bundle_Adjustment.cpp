#include "Bundle_Adjustment.h"



void Bundle_Adjustment::estimateY(std::vector<std::reference_wrapper<Eigen::VectorXd>>& input, Eigen::VectorXd& output) {
	Eigen::VectorXd est1 = input[0].get();
	Eigen::VectorXd est2 = input[1].get();

	int len1, len2;
	len1 = est1.size();
	len2 = est2.size();

	output.resize(2);

	if (len1 == 6 && len2 == 3) {
		output = projectXY(est1, est2);
	}
	else if(len1 == 3 && len2 == 6){
		output = projectXY(est2, est1);
	}
	else {
		std::cout << "Invalid input to estimateY" << std::endl;
	}
	return;
}

Eigen::Vector2d Bundle_Adjustment::projectXY(const Vector6d pose, const Eigen::Vector3d& point) {

	Matrix4d T = expSE3Map(pose);
	Eigen::Vector4d q;
	q << point, 1;

	Eigen::Vector4d q_prime = T * q;

	////using sophus
	//Sophus::SE3d T = Sophus::SE3d::exp(pose);
	//Eigen::Vector3d q_prime = T * point;

	Eigen::Vector2d xy;
	xy << q_prime(0) / q_prime(2), q_prime(1) / q_prime(2);

}

void Bundle_Adjustment::addVertex(int id, Eigen::VectorXd vertex_data, int vertex_type, bool isFixed = false) {

	assert(vertex_data.size() == 6 || vertex_data.size() == 3);

	if (vertex_data.size() == 6 && vertex_type == 0) {
		PoseVertex* v = new PoseVertex;
		v->setParameters(vertex_data);//head(3) is translation, tail(3) is rotation
		v->setFixed(isFixed);
		general_vertices[vertex_type].push_back(v);
	}
	else if (vertex_data.size() == 3 && vertex_type == 1) {
		PointVertex* v = new PointVertex;
		v->setParameters(vertex_data);//X,Y,Z coordinates of the map point
		v->setFixed(isFixed);
		general_vertices[vertex_type].push_back(v);
	}
	else {
		std::cout << "vertex type and size mismatch" << std::endl;
	}

}

void Bundle_Adjustment::addEdge(int id, Eigen::VectorXd measurement, double w_sigma, int first_vertex_id, int second_vertex_id) {
	assert(measurement.size() == 2);
	Edge* e = new Edge;
	e->setMeasurement(measurement, w_sigma);
	e->setVertexIds(first_vertex_id, second_vertex_id);
	general_edges.push_back(e);

}