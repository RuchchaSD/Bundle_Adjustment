#ifndef VERTEX_TYPES_H
#define VERTEX_TYPES_H
#include "general_vertex.h"
#include <sophus/se3.hpp>
#include "Basic_functions.h"

class PoseVertex : public general_vertex {

	void setParameters(const Eigen::VectorXd& parameters) override;
	void setParameters(const Sophus::SE3f& parameters);
	void updateParameters(const Eigen::VectorXd& parametersUpdate) override;
	Eigen::VectorXd getParameters()override;
	

};

class PointVertex : public general_vertex {

	void setParameters(const Eigen::VectorXd& parameters) override;
	void updateParameters(const Eigen::VectorXd& parametersUpdate) override;
	Eigen::VectorXd getParameters() override;

};



#endif // !VERTEX_TYPES_H


