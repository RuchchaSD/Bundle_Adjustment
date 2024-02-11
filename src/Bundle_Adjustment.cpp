// Bundle_Adjustment.cpp : Defines the entry point for the application.
//

#include "Bundle_Adjustment.h"

using namespace std;

int main(int argc, char** argv) {

    // Parameteres and data for the problem
    std::string filePath = "..\\..\\..\\..\\Other\\data\\data.csv"; // Update this to your actual file path
    auto [col1, col2] = readCsvColumns(filePath);

    int N = col1.size(); // Number of data points
    double w_sigma = 1; // Noise of the data
    std::vector<Eigen::VectorXd> x_data(N), y_data(N); // Data

    for (int i = 0; i < N; i++) {
        x_data[i].resize(1);
        y_data[i].resize(1);
		x_data[i][0] = (std::stod(col1[i]));
		y_data[i][0] = (std::stod(col2[i]));
	}
    

    std::vector<int> edge_sizes = { 1 };
    std::vector<int> vertex_sizes = { 3,1 };
    Optimization_General* optimizer = new Optimization_General(edge_sizes, vertex_sizes);

    Eigen::VectorXd initial_est(3);
    initial_est <<1, 2, 1;


    optimizer->addVertex(N, initial_est, 0, false);

    for (int i = 0; i < N; i++) {
        //add fixed vertex
        optimizer->addVertex(i, x_data[i], 1, true);
        //add edge
        optimizer->addEdge(i, y_data[i], w_sigma, N, i);
    }

    //optimizer->removeEdge(54);
    //optimizer->removeVertex(54);

    //optimizer->setRobust(true,100);
    optimizer->initialize();

    Eigen::VectorXd parameters;

    //std::cout << "Before optimization:" << std::endl;

    //measure the time before the optimization
    auto start = std::chrono::high_resolution_clock::now();
     
    //optimizer->optimize(100);
    optimizer->optimizeWithLM(100);

     
     //measure the time after the optimization
     auto finish = std::chrono::high_resolution_clock::now();
     std::chrono::duration<double> elapsed = finish - start;

     //print the elapsed time in seconds
     std::cout << "\nElapsed time: " << elapsed.count() << " s\n";

     //std::cout << "\nAfter optimization:" << std::endl;

     std::cout << "\nOptimized parameters: " << optimizer->getVertexParameters(N).transpose()<<"\n\n";


    
    return 0;
}


//compare with g2o - potentially step by step
//improving accuracy
//Bundle adjustment problem 
//
//
//
//
