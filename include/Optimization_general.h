#ifndef Optmization_General_H
#define Optmization_General_H

#include <chrono>
//#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
 #include "Basic_functions.h"

class general_vertex;
class general_edge;

#include "general_vertex.h"
#include "general_edge.h"

class Optimization_General{

private:
    size_t general_edge_count;
    size_t vertex_count;
    size_t fixed_vertex_count;
    size_t vertex_size; // size of the vertex vector

    bool bRobust;

    std::vector<int> vertex_sizes; // stores number of parameters for each vertex type
    std::vector<int> vertex_types;// stores how much of each vertex type is present
    int edge_size; // size of the edge vector
    std::vector<int> edge_sizes;

    std::vector<std::vector<general_vertex*>> general_vertices;
    std::map<int, general_vertex*> temp_vertices;
    std::vector<general_vertex*> fixed_vertices;

    std::vector<general_edge*> general_edges;
    std::map<int, general_edge*> temp_edges;

    // std::unordered_map<int,int> general_vertices_map;//map to store the id of the vertex and its index in the vertices vector
    // std::unordered_map<int,int> general_edges_map;//map to store the id of the edge and its index in the edges vector

    Eigen::VectorXd* errorVec;
    Eigen::MatrixXd* Cov;
    Eigen::MatrixXd* Jacobian;

    Eigen::VectorXd* deltaX;
    Eigen::MatrixXd* A;
    Eigen::VectorXd* b;

    //function to estimate the measurements from the pose and landmark vertices, this function is passed to the optimization class
    //last element of the vector should contain the reference to the output vector
    void estimateY(std::vector<std::reference_wrapper<Eigen::VectorXd>>& input, Eigen::VectorXd& output);
    void computeJacobianVertex(const Eigen::VectorXd& parameters, const Eigen::VectorXd& constants, const double step_size, Eigen::MatrixXd& J);
    //compute the error between the estimated parameters and the actual measurements
    void computeError(const Eigen::VectorXd& estimatedParameters1, const Eigen::VectorXd& estimatedParameters2, const Eigen::VectorXd& Measurements, Eigen::VectorXd& errorVec);
    void buildJacobian();//take pose_vertices and landmark_vertices and build the jacobian
    void buildErrorVector();//take pose_vertices and landmark_vertices and build the error vector
    void buildCovarianceMatrix();//make the covariance matrix from w_sigma in the edges
    void updateEstimates(Eigen::VectorXd& deltaX);//update the pose and landmark vertices with the new estimates
    void RobustKernel(Eigen::VectorXd& estimateVec, Eigen::VectorXd& measurementVec, Eigen::VectorXd& Error);


public:

    Optimization_General();
    Optimization_General(std::vector<int> edge_sizes, std::vector<int> vertex_sizes);
    ~Optimization_General();

    void setVertexSize(int vertex_size);
    void setVertexSizes(std::vector<int> vertex_sizes);
    void setRobust(bool robust);
    bool getRobust();
    void setEdgeSize(int edge_size);
    void setEdgeSizes(std::vector<int> edge_sizes);

    void addVertex(int id, Eigen::VectorXd vertex_data, int vertex_type, bool isFixed = false);
    void removeVertex(int id);
    Eigen::VectorXd getVertexParameters(int id);

    void addEdge(int id, Eigen::VectorXd measurement, double w_sigma, int first_vertex_id, int second_vertex_id);
    void removeEdge(int id);
    Eigen::VectorXd getEdgeMeasurement(int id);

    void initialize();


    void optimize(int iterations);

};
#endif
