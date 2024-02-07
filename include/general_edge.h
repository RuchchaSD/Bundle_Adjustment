#ifndef GENERAL_EDGE_H
#define GENERAL_EDGE_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "general_vertex.h"

class general_vertex;

class general_edge
{
private:
    Eigen::VectorXd measurement;
    double w_sigma;
    int id;
    int global_id;
    int first_vertex_id;
    int second_vertex_id;
    bool isInitialized = false;

    general_vertex* first_vertex_ptr;
    general_vertex* second_vertex_ptr;

public:


    general_edge(int global_id);
    general_edge(int global_id, int id);


    //Id
    int getId();

    int getGlobalId();
    //Vertices
    void setFirstVertex(general_vertex* first_vertex_ptr);

    void setSecondVertex(general_vertex* second_vertex_ptr);

    general_vertex* getFirstVertex();

    general_vertex* getSecondVertex();

    //Measurement
    void setMeasurement(const Eigen::VectorXd& measurement, const double& w_sigma);

    Eigen::VectorXd getMeasurement();

    //return w_sigma^2
    double getCovariance();

    //Initialize
    void initialize(int id);

    bool getIsInitialized();

};

#endif