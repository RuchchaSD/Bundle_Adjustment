#include "vertex_types.h"

void PoseVertex::setParameters(const Eigen::VectorXd& parameters) {
	assert(parameters.size() == 6);
	this->parameters = parameters;
}
void PoseVertex::setParameters(const Sophus::SE3f& parameters) {
	Sophus::SE3d parameters_d = parameters.cast<double>();
	this->parameters = parameters_d.log();
}
void PoseVertex::updateParameters(const Eigen::VectorXd& parametersUpdate) { // first three are translation, last three are rotation
	//Sophus::SE3d update = Sophus::SE3d::exp(parametersUpdate);
	//Sophus::SE3d current = Sophus::SE3d::exp(this->parameters);
	//this->parameters = (update  * current).log();
	Eigen::MatrixXd update = expSE3Map(parametersUpdate); // from Basic_functions.cpp
	Eigen::MatrixXd current = expSE3Map(this->parameters);
	this->parameters = logMapSE3(update * current);// from Basic_functions.cpp
}
Eigen::VectorXd PoseVertex::getParameters() {
	return this->parameters;
}




void PointVertex::setParameters(const Eigen::VectorXd& parameters) {
	assert(parameters.size() == 3);
	this->parameters = parameters;
}
void PointVertex::updateParameters(const Eigen::VectorXd& parametersUpdate) {
	assert(parametersUpdate.size() == 3);
	this->parameters += parametersUpdate;
}
Eigen::VectorXd PointVertex::getParameters() {
	return this->parameters;
}