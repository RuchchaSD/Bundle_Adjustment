#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "data_generate.h"

using namespace std;
using namespace Eigen;

void optimize(int iterations, vector<double>& estimate, vector<vector<vector<double>>>& data, double (*estimateY)(vector<double>&, double),
    void (*calcJacobian)(VectorXd&, vector<double>&, vector<double>&));

void calcJac(VectorXd& J, vector<double>& estimate, vector<double>& xi);
void clacJac(VectorXd& J, vector<double>& estimate, vector<double>& xi,
    vector<double>& yi, double (*estimateY)(vector<double>&, double), double step);
double estimateY(vector<double>& estimate, double xi);


int main(int argc, char** argv) {

    std::string filePath = "..\\..\\..\\..\\..\\Other\\data\\data.csv"; // Update this to your actual file path
    auto [col1, col2] = readCsvColumns(filePath);

    int N = col1.size(); // Number of data points
    double w_sigma = 1.0; // Noise of the data
    vector<vector<double>> x_data, y_data;      // Data
    //std::vector<Eigen::VectorXd> x_data(N), y_data(N); // Data

    for (int i = 0; i < N; i++) {
        vector<double> x = { std::stod(col1[i]) };
        x_data.push_back(x);
        vector<double> y = { std::stod(col2[i]) };
        y_data.push_back(y);
    }




    // Start Gauss-Newton iteration
    int iterations = 100;    // Number of iterations
    double ae = 2.0, be = -1.0, ce = 5.0;        // Estimated parameter values to start iteration

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    vector<double> estimate = { ae,be,ce };
    vector<vector<vector<double>>> data = { x_data,y_data };

    optimize(iterations, estimate, data, estimateY, calcJac);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
    std::cout << "estimated abc = " << estimate[0] << ", " << estimate[1] << ", " << estimate[2] << endl;

    return 0;
}



void optimize(int iterations, vector<double>& estimate, vector<vector<vector<double>>>& data, double (*estimateY)(vector<double>&, double),
    void (*calcJacobian)(VectorXd&, vector<double>&, vector<double>&)) { //double &ae, double &be, double &ce, vector<double> x_data, vector<double> y_data){

    int num_of_est = estimate.size();

    vector<vector<double>> x_data = data[0];
    vector<vector<double>> y_data = data[1];
    int num_of_data = x_data.size();

    //calculate standard deviation of y_data
    // double w_sigma = calculateStandardDeviation(y_data);
    // double inv_sigma = 1.0 / w_sigma;

    MatrixXd H;             // Hessian = J^T W^{-1} J in Gauss-Newton
    VectorXd b;             // Bias
    double cost = 0, lastCost = 0;  // Cost of current iteration and cost of last iteration

    //jacobian matrix
    VectorXd J(num_of_est), K(num_of_est);

    //to be used in the loop
    vector<double> xi, yi;

    VectorXd error(y_data[0].size());

    for (int i = 0; i < iterations; i++) {

        H = MatrixXd::Zero(num_of_est, num_of_est);
        b = VectorXd::Zero(num_of_est);
        cost = 0;

        for (int j = 0; j < num_of_data; j++) {
            xi = x_data[j];
            yi = y_data[j];
            // error = yi - exp(estimate[0] * xi * xi + estimate[1] * xi + estimate[2]);//implement inside a function

            for (int k = 0; k < xi.size(); k++) {
                error[k] = yi[k] - estimateY(estimate, xi[0]);
            }

            // error = yi - estimateY(estimate,xi);

            //update jacobian matrix - implement inside a function
            // calcJacobian(J,estimate,xi); //this needs to be passed as a function pointer
            clacJac(J, estimate, xi, yi, estimateY, 1e-6);// no need to pass as a function pointer use K for dummy variable

            //cout << "Iteration: " << i << " data_point: " << j << " | Jacobian: " << J.transpose() << "| error: " << error.transpose() << "\n";

            // compare the two jacobian matrices
            // cout << "Iteration"<< i << "data piont:" << j << "|| J: " << J.transpose() << " , K: " << K.transpose() << endl;
            // cout << "K: " << K << endl;

            //update Hessian matrix


            // H += inv_sigma * inv_sigma * J * J.transpose();
            // b += -inv_sigma * inv_sigma * error * J; //update bias vector : Hx = b -> Hx / inv_sigma**2 = b / inv_sigma ** 2
            H += J * J.transpose();
            b += -J * error.transpose();

            cost += error.norm() * error.norm();
        }

        // Solve linear equation Hx=b
        VectorXd dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            std::cout << "result is nan!" << endl;
            break;
        }

        if (i > 0 && cost + 1e-4 >= lastCost) {
            std::cout << endl << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
            //error vector
            for (int j = 0; j < error.size(); j++) {
                std::cout << error[j] << ",";
            }
            std::cout << endl;
            //estimate vector
            for (int j = 0; j < num_of_est; j++) {
                std::cout << estimate[j] << ",";
            }
            std::cout << endl;
            break;
        }

        //update estimate
        for (int j = 0; j < num_of_est; j++) {
            estimate[j] += dx[j];
        }

        lastCost = cost;

        std::cout <<"Iteration: " << i <<"total cost: " << cost << ", | update: " << dx.transpose() <<
            " | estimated params : ";

        for (int j = 0; j < num_of_est; j++) {
            std::cout << estimate[j] << ",";
        }
        std::cout << "\n\n";
    }
}

// double calculateStandardDeviation(const vector<double>& data) { //make this function to accept multiple y_data vectors
//     // Check if the vector is empty or contains only one element
//     if (data.empty() || data.size() == 1) {
//         return 0.0; // Standard deviation is undefined in these cases
//     }
//     double sum = 0.0;
//     double mean = 0.0;
//     // Calculate the mean (average) of the elements in the vector
//     for (const double& value : data) {
//         sum += value;
//     }
//     mean = sum / static_cast<double>(data.size());
//     // Calculate the sum of squared differences from the mean
//     double sumSquaredDifferences = 0.0;
//     for (const double& value : data) {
//         double difference = value - mean;
//         sumSquaredDifferences += difference * difference;
//     }
//     // Calculate the variance and then the standard deviation
//     double variance = sumSquaredDifferences / static_cast<double>(data.size() - 1);
//     double standardDeviation = std::sqrt(variance);
//     return standardDeviation;
// }

void calcJac(VectorXd& J, vector<double>& estimate, vector<double>& xVec) {

    double xi = xVec[0];
    J[0] = -xi * xi * exp(estimate[0] * xi * xi + estimate[1] * xi + estimate[2]);  // de/da
    J[1] = -xi * exp(estimate[0] * xi * xi + estimate[1] * xi + estimate[2]);  // de/db
    J[2] = -exp(estimate[0] * xi * xi + estimate[1] * xi + estimate[2]);  // de/dc
}

double estimateY(vector<double>& estimate, double xi) {
    return std::exp(estimate[0] * xi * xi + estimate[1] * xi + estimate[2]);
}

void clacJac(VectorXd& J, vector<double>& estimate, vector<double>& xi, vector<double>& yi, double (*estimateY)(vector<double>&, double), double step) {
    vector<double> tempEstimate = estimate;

    double y0 = estimateY(estimate, xi[0]);

    for (int i = 0; i < estimate.size(); i++) {
        tempEstimate[i] += step;
        // J[i] = ((yi[0] - estimateY(tempEstimate,xi[0])) - (yi[0] - estimateY(estimate,xi[0]))) / step;

        J[i] = ((y0 - estimateY(tempEstimate, xi[0]))) / step;

        tempEstimate[i] = estimate[i];
    }
}