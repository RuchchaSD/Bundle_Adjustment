#pragma once
#include <iostream>
#include <chrono>
#include <sophus/se3.hpp>
#include "Optimization_General.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;


class Bundle_Adjustment : public Optimization_General {

protected:
	void estimateY(std::vector<std::reference_wrapper<Eigen::VectorXd>>& input, Eigen::VectorXd& output) override;
	Eigen::Vector2d projectXY(const Vector6d pose,const Eigen::Vector3d& point);
	
	
public:
	void addVertex(int id, Eigen::VectorXd vertex_data, int vertex_type, bool isFixed = false) override;
	void addEdge(int id, Eigen::VectorXd measurement, double w_sigma, int first_vertex_id, int second_vertex_id) override;
};