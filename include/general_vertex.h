#ifndef GENERAL_VERTEX_H
#define GENERAL_VERTEX_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>

#include <iostream>

#include "general_edge.h"

class general_edge;

class general_vertex
{
private:
    int vertex_type; // 0,1,2,3 ......
    Eigen::VectorXd parameters;
    Eigen::VectorXd previous_parameters;
    int id;
    int global_id;
    std::map<int, general_edge*> attached_edges;
    std::map<int, general_vertex*> attached_vertices;
    bool isInitialized = false;
    bool isFixed = false;
    int temp_id;

    void initialize(int id);

public:
    general_vertex(int g_id, int id, int vertex_type);
    general_vertex(int g_id);

    //Type
    void setType(int vertex_type);
    int getType();
    //Id
    int getId();

    int getGlobalId();
    //Parameters
    void setParameters(const Eigen::VectorXd& parameters);

    void updateParameters(const Eigen::VectorXd& parametersUpdate);

    void revertParameters();

    Eigen::VectorXd getParameters();

    //Edges
    void addEdge(general_edge* edge, general_vertex* general_vertex_ptr);

    const std::map<int, general_edge*> getAttachedEdges();

    const std::map<int, general_vertex*> getAttachedVertices();

    //Fixedness
    void setFixed(bool isFixed);

    bool getFixed();

    //Initialization
    bool getInitialized();

    void setId(int id);

};


#endif