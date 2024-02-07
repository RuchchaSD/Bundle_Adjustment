#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
//#include <opencv2/core/core.hpp>
#include <data_generate.h>
#include <cmath>
#include <chrono>
using namespace std;

// Vertex for the curve model, template parameters: optimization variable dimension and data type
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        virtual void setToOriginImpl() // Reset
    {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update) // Update
    {
        _estimate += Eigen::Vector3d(update);
    }
    // Save and load: left empty
    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}
};

// Error model, template parameters: observation dimension, type, connected vertex type
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
    // Calculate the error of the curve model
    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }
    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}
public:
    double _x;  // x value, y value is _measurement
};

int main(int argc, char** argv)
{
    //double a = 1.0, b = 2.0, c = 1.0;         // True parameter values
    //int N = 100;                          // Data points
    //double w_sigma = 1.0;                 // Noise Sigma value
    //cv::RNG rng;                        // OpenCV random number generator
    //double abc[3] = { 0,0,0 };            // Estimated values of abc parameters

    //vector<double> x_data, y_data;      // Data

    //cout << "generating data: " << endl;
    //for (int i = 0; i < N; i++)
    //{
    //    double x = i / 100.0;
    //    x_data.push_back(x);
    //    y_data.push_back(
    //        exp(a * x * x + b * x + c) + rng.gaussian(w_sigma)
    //    );
    //    cout << x_data[i] << " " << y_data[i] << endl;
    //}

        // Parameteres and data for the problem
    std::string filePath = "..\\data\\data.csv"; // Update this to your actual file path
    auto [col1, col2] = readCsvColumns(filePath);

    int N = col1.size(); // Number of data points
    double w_sigma = 1.0; // Noise of the data
    vector<double> x_data, y_data;      // Data
    //std::vector<Eigen::VectorXd> x_data(N), y_data(N); // Data

    for (int i = 0; i < N; i++) {
        double x = std::stod(col1[i]);
        x_data.push_back(x);
        double y = std::stod(col2[i]);
        y_data.push_back(y);
    }




    // Build the graph optimization, first set g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 1> > Block;  // Each error term has 3 optimization variables and 1 error value
    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // Linear equation solver
    // Block* solver_ptr = new Block( linearSolver );      // Block solver
    std::unique_ptr<Block::LinearSolverType> linearSolver = std::make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>();
    std::unique_ptr<Block> solver_ptr = std::make_unique<Block>(std::move(linearSolver));

    // Gradient descent method, choose from GN, LM, DogLeg
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    // std::unique_ptr<g2o::OptimizationAlgorithmLevenberg> solver = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(std::move(solver_ptr));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer;     // Graph model
    optimizer.setAlgorithm(solver);   // Set the solver
    optimizer.setVerbose(true);       // Enable debug output

    // Add vertices to the graph
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0, 0, 0));
    v->setId(0);
    optimizer.addVertex(v);

    // Add edges to the graph
    for (int i = 0; i < N; i++)
    {
        CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);                // Set the connected vertex
        edge->setMeasurement(y_data[i]);      // Observation value
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // Information matrix: inverse of covariance matrix
        optimizer.addEdge(edge);
    }

    // Perform optimization
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // Output the optimized values
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;

    return 0;
}