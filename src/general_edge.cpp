#include "../include/general_edge.h"

general_edge::general_edge(int global_id) : global_id(global_id) {
    this->id = -1;
	this->first_vertex_ptr = nullptr;
    this->first_vertex_id = 0;
	this->second_vertex_ptr = nullptr;
    this->second_vertex_id = 0;
	this->measurement = Eigen::VectorXd::Zero(0);
	this->w_sigma = 0;
	this->isInitialized = false;

}

general_edge::general_edge(int global_id, int id) : id(id), global_id(global_id) {
	this->first_vertex_ptr = nullptr;
	this->first_vertex_id = 0;
	this->second_vertex_ptr = nullptr;
    this->second_vertex_id = 0;
	this->measurement = Eigen::VectorXd::Zero(0);
	this->w_sigma = 0;
	this->isInitialized = false;

}

int general_edge::getId()
{
    return this->id;
}

int general_edge::getGlobalId()
{
    return this->global_id;
}

void general_edge::setFirstVertex(general_vertex* first_vertex_ptr)
{
    this->first_vertex_ptr = first_vertex_ptr;
    this->first_vertex_id = first_vertex_ptr->getGlobalId();
}

void general_edge::setSecondVertex(general_vertex* second_vertex_ptr)
{
    this->second_vertex_ptr = second_vertex_ptr;
    this->second_vertex_id = second_vertex_ptr->getGlobalId();
}

general_vertex* general_edge::getFirstVertex()
{
    return this->first_vertex_ptr;
}

general_vertex* general_edge::getSecondVertex()
{
    return this->second_vertex_ptr;
}

void general_edge::setMeasurement(const Eigen::VectorXd& measurement, const double& w_sigma)
{
    this->measurement = measurement;
    this->w_sigma = w_sigma;
}

Eigen::VectorXd general_edge::getMeasurement()
{
    if (isInitialized)
    {
        return this->measurement;
    }
    else
    {
        std::cout << "Edge not initialized" << std::endl;
        throw std::runtime_error("Edge not initialized");
    }
}

double general_edge::getCovariance()
{
    if (isInitialized)
    {
        return this->w_sigma * w_sigma;
    }
    else
    {
        std::cout << "Edge not initialized" << std::endl;
        throw std::runtime_error("Edge not initialized");
    }
}

void general_edge::initialize(int id)
{
    if (!this->isInitialized)
    {
        this->id = id;
        if (this->first_vertex_ptr == nullptr || this->second_vertex_ptr == nullptr || this->measurement.size() == 0 || this->w_sigma == 0)
        {
            if (this->first_vertex_ptr == nullptr)
            {
                throw std::runtime_error("Edge " + std::to_string(id) + " : First vertex not set");
            }
            else if (this->second_vertex_ptr == nullptr)
            {
                throw std::runtime_error("Edge " + std::to_string(id) + " : Second vertex not set");
            }
            else if (this->measurement.size() == 0)
            {
                throw std::runtime_error("Edge " + std::to_string(id) + " : Measurement not set");
            }
            else if (this->w_sigma == 0)
            {
                throw std::runtime_error("Edge " + std::to_string(id) + " : Covariance not set");
            }
        }
        else
        {
            this->first_vertex_ptr->addEdge(this, this->second_vertex_ptr);
            this->second_vertex_ptr->addEdge(this, this->first_vertex_ptr);
            this->isInitialized = true;
        }
    }
    else
    {
        std::cout << "Edge " << id << " : already initialized" << std::endl;
    }
}

bool general_edge::getIsInitialized()
{
    return this->isInitialized;
}