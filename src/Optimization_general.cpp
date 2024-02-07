﻿#include "Optimization_General.h"


// private:

void Optimization_General::computeError(const Eigen::VectorXd& estimatedParameters1, const Eigen::VectorXd& estimatedParameters2, const Eigen::VectorXd& Measurements, Eigen::VectorXd& errorVec) {

    Eigen::VectorXd y_est;
    Eigen::VectorXd estimatedParameters_ = estimatedParameters1;
    Eigen::VectorXd estimatedParameters2_ = estimatedParameters2;


    std::vector<std::reference_wrapper<Eigen::VectorXd>> arg_vec;
    arg_vec.push_back(estimatedParameters_);
    arg_vec.push_back(estimatedParameters2_);

    estimateY(arg_vec, y_est);
    errorVec.resize(Measurements.size());

    errorVec = y_est - Measurements;

}

void Optimization_General::computeJacobianVertex(const Eigen::VectorXd& parameters, const Eigen::VectorXd& constants, double step_size, Eigen::MatrixXd& J) {

    Eigen::VectorXd y0; // y(estimate)
    Eigen::VectorXd y1;// y(estimate + h)
    Eigen::VectorXd temp_parameters = parameters; //copy assignment
    Eigen::VectorXd temp_constants = constants; //copy assignment

    //make a vector of arguments to pass to the estimateY function
    std::vector<std::reference_wrapper<Eigen::VectorXd>> arguments_vec;
    arguments_vec.push_back(temp_parameters);
    arguments_vec.push_back(temp_constants);

    estimateY(arguments_vec, y0);// calculate y(estimate)

    //∂e/∂x = (de(estimate + step_size) - de(estimate)) / dstep_size => ( (y(estimate + h) - y(actual)) - (y(estimate) - y(actual)) )  ) /h => (y(estimate + h) - y(estimate)) / h
    for (int i = 0; i < parameters.size(); i++) {

        //update the parameter
        temp_parameters[i] += step_size;

        // calculate y(estimate + h)
        estimateY(arguments_vec, y1); // calculate y(estimate + h)

        //calculate the partial derivative
        J.col(i) = (y1 - y0) / step_size;

        temp_parameters[i] = parameters[i]; //reset the parameter
    }
}

void Optimization_General::buildJacobian() {

    Eigen::MatrixXd* J = new Eigen::MatrixXd();

    //structure of the jacobian matrix -> rows - number of measurements(observations in a measurement) * measurement count, cols - n of parameters in a vertex x number of vertices
    //Order - rows -> order of the edges in the edges vector, cols -> order of the vertices in the vertices vector

    size_t jacobian_column_size = 0;

    for (int i = 0; i < this->vertex_sizes.size(); i++)
        jacobian_column_size += this->general_vertices[i].size() * this->vertex_sizes[i];


    J->resize(this->edge_size * this->general_edge_count, jacobian_column_size);//change

    Eigen::MatrixXd J_vertex;

    general_vertex* first_vertex_ptr;
    general_vertex* second_vertex_ptr;

    int column_location,row_location;
    int vertex_size;


    for (auto edge_ptr : this->general_edges) { //iterate for all edges in the edges vector

        first_vertex_ptr = edge_ptr->getFirstVertex();
        second_vertex_ptr = edge_ptr->getSecondVertex();

        int first_vertex_id = first_vertex_ptr->getId();
        int second_vertex_id = second_vertex_ptr->getId();

        //check if the vertex is fixed and skip it if it is withouth calculating the jacobian

        if (!first_vertex_ptr->getFixed()) {
            //resize the jvertex here
            vertex_size = this->vertex_sizes[first_vertex_ptr->getType()];
            J_vertex.resize(this->edge_size, vertex_size);

            //calculate the first vertex jacobian
            this->computeJacobianVertex(first_vertex_ptr->getParameters(), second_vertex_ptr->getParameters(), 1e-6, J_vertex);

            //calculate the position of the first vertex in the jacobian matrix
            column_location = 0;
            row_location = edge_ptr->getId() * this->edge_size;

            for (int i = 0; i < first_vertex_ptr->getType(); i++) {
                column_location += this->vertex_types[i] * this->vertex_sizes[i];
            }
            column_location += vertex_size * first_vertex_ptr->getId();

            //add the first vertex jacobian to the jacobian matrix
            //std::cout << "Row location: " << row_location << " | Column location: " << column_location << " | J_vertex: " << J_vertex<< std::endl;
            J->block(row_location, column_location, J_vertex.rows(), J_vertex.cols()) = J_vertex;

        }
        if (!second_vertex_ptr->getFixed()) {

            vertex_size = this->vertex_sizes[second_vertex_ptr->getType()];
            J_vertex.resize(this->edge_size, vertex_size);

            //calculate the first vertex jacobian
            this->computeJacobianVertex(second_vertex_ptr->getParameters(), first_vertex_ptr->getParameters(), 1e-6, J_vertex);

            //calculate the position of the first vertex in the jacobian matrix
            column_location = 0;
            row_location = edge_ptr->getId() * this->edge_size;

            for (int i = 0; i < second_vertex_ptr->getType(); i++) {
                column_location += this->vertex_types[i] * this->vertex_sizes[i];
            }
            column_location += vertex_size * second_vertex_ptr->getId();

            //add the first vertex jacobian to the jacobian matrix
            J->block(row_location, column_location, J_vertex.rows(), J_vertex.cols()) = J_vertex;

        }
    }
    delete this->Jacobian;
    
    this->Jacobian = J;
    //std::cout << "Jacobian matrix: " << *Jacobian << std::endl;
}

void Optimization_General::buildCovarianceMatrix() {
    Eigen::MatrixXd* Cov = new Eigen::MatrixXd();

    //structure of the covariance matrix -> rows & cols - number of measurements(observations in a measurement) * measurement count
    Cov->resize(this->edge_size * this->general_edge_count, this->edge_size * this->general_edge_count);
    Cov->setZero();

    Eigen::MatrixXd Cov_edge;
    Cov_edge.resize(this->edge_size, this->edge_size);

    double sigma_squared;
    int location;

    for (auto edge_ptr : this->general_edges) {

        sigma_squared = edge_ptr->getCovariance();
        location = edge_ptr->getId() * this->edge_size;

        Cov_edge.setIdentity();
        Cov_edge *= sigma_squared;
        //std::cout << "Location: " << location << "\n";
        //std::cout << "Cov_edge: " << Cov_edge << "\n";

        Cov->block(location, location, edge_size, edge_size) = Cov_edge;
    }
    delete this->Cov;
    this->Cov = Cov;
    //std::cout << "Covariance matrix built" << std::endl;
    //std::cout << *Cov << std::endl;
}

void Optimization_General::updateEstimates(Eigen::VectorXd& deltaX) {
    //update the pose and landmark vertices with the new estimates
//    for(auto vertex_ptr : this->general_vertices){
//         vertex_ptr->updateParameters(deltaX.segment(vertex_ptr->getId() * this->vertex_size, this->vertex_size));
//     }

    int location = 0;
    int l = 0;
    int size = 0;

    for (int i = 0; i < this->general_vertices.size(); i++) {
        for (int k = 0; k < i; k++)
            location += this->vertex_sizes[k] * this->vertex_sizes[k];
        size = this->vertex_sizes[i];

        for (int j = 0; j < this->general_vertices[i].size(); j++) {
            l = location + j * size;
            this->general_vertices[i][j]->updateParameters(deltaX.segment(l, size));
        }
    }
}

void Optimization_General::buildErrorVector() {
    Eigen::VectorXd* eVec = new Eigen::VectorXd();
    //structure of the error vector -> rows - number of measurements(observations in a measurement) * measurement count, cols - 1
    eVec->resize(this->edge_size * this->general_edge_count);

    Eigen::VectorXd errorVec_edge;
    errorVec_edge.resize(this->edge_size);

    //double w_sigma;

    auto processEdge = [&](general_edge* edge_ptr) {
        general_vertex* first_vertex_ptr = edge_ptr->getFirstVertex();
        general_vertex* second_vertex_ptr = edge_ptr->getSecondVertex();

        Eigen::VectorXd estimatedParameters1 = first_vertex_ptr->getParameters();
        Eigen::VectorXd estimatedParameters2 = second_vertex_ptr->getParameters();

        Eigen::VectorXd Measurements = edge_ptr->getMeasurement();

        this->computeError(estimatedParameters1, estimatedParameters2, Measurements, errorVec_edge);

        eVec->segment(edge_ptr->getId() * this->edge_size, this->edge_size) = errorVec_edge;
        };
    //calculate the error vector for each edge and add the error to the error vector

    for (auto edge_ptr : this->general_edges) {
        processEdge(edge_ptr);
    }

    delete this->errorVec;
    this->errorVec = eVec;
}

void Optimization_General::estimateY(std::vector<std::reference_wrapper<Eigen::VectorXd>>& input, Eigen::VectorXd& output) {
    Eigen::VectorXd est1 = input[0].get(); // a,b,c in ax^2 + bx + c
    Eigen::VectorXd est2 = input[1].get();// x in ax^2 + bx + c

    double a = est1[0];
    double b = est1[1];
    double c = est1[2];

    double x = est2[0];

    output.resize(1);

    output[0] = (a * x * x * x + b * x + c);
}
// public:
Optimization_General::Optimization_General(std::vector<int> edge_sizes, std::vector<int> vertex_sizes) {
    this->edge_sizes = edge_sizes;
    this->vertex_sizes = vertex_sizes;
    this->general_edge_count = 0;
    this->vertex_count = 0;
    this->edge_size = edge_sizes[0];
    this->vertex_size = vertex_sizes[0];
    this->fixed_vertex_count = 0;
    this->errorVec = new Eigen::VectorXd();
    this->Cov = new Eigen::MatrixXd();
    this->Jacobian = new Eigen::MatrixXd();
    this->deltaX = new Eigen::VectorXd();
    this->A = new Eigen::MatrixXd();
    this->b = new Eigen::VectorXd();
    this->bRobust = false;
    //this->vertex_types.resize(vertex_sizes.size());
    //this->general_vertices.resize(vertex_sizes.size());
    //this->general_edges.resize(edge_sizes.size());
}

Optimization_General::Optimization_General() {
    std::cout << "Please specify the size of the vertices and edges vectors" << std::endl;
    this->edge_size = 0;
    this->vertex_size = 0;
    this->general_edge_count = 0;
    this->vertex_count = 0;
    this->fixed_vertex_count = 0;
    this->errorVec = new Eigen::VectorXd();
    this->Cov = new Eigen::MatrixXd();
    this->Jacobian = new Eigen::MatrixXd();
    this->deltaX = new Eigen::VectorXd();
    this->A = new Eigen::MatrixXd();
    this->b = new Eigen::VectorXd();
    this->bRobust = false;
}

void Optimization_General::setVertexSize(int vertex_size) {
    this->vertex_size = vertex_size;
}

void Optimization_General::setVertexSizes(std::vector<int> vertex_sizes) {
    this->vertex_sizes = vertex_sizes;
}

void Optimization_General::setRobust(bool bRobust) {
	this->bRobust = bRobust;
}

bool Optimization_General::getRobust() {
	return this->bRobust;
}

void Optimization_General::setEdgeSize(int edge_size) {
    this->edge_size = edge_size;
}

void Optimization_General::setEdgeSizes(std::vector<int> edge_sizes) {
    this->edge_sizes = edge_sizes;
}

Optimization_General::~Optimization_General() {}

void Optimization_General::addVertex(int id, Eigen::VectorXd vertex_data, int vertex_type, bool isFixed)
{
    general_vertex* vertex_ptr = new general_vertex(id);
    vertex_ptr->setType(vertex_type);
    vertex_ptr->setParameters(vertex_data);
    vertex_ptr->setFixed(isFixed);
    this->temp_vertices.insert(std::make_pair(id, vertex_ptr));
}

void Optimization_General::removeVertex(int id)
{
    this->temp_vertices.erase(id);
}

Eigen::VectorXd Optimization_General::getVertexParameters(int id) {
    if (temp_vertices.count(id) == 0) {
        std::cout << "Vertex with ID " << id << " does not exist." << std::endl;
        return Eigen::VectorXd(); // Return an empty vector or handle the error accordingly.
    }
    return temp_vertices[id]->getParameters();
}

void Optimization_General::addEdge(int id, Eigen::VectorXd measurement, double w_sigma, int first_vertex_id, int second_vertex_id) {

    general_edge* edge_ptr = new general_edge(id);
    edge_ptr->setMeasurement(measurement, w_sigma);

    //check if the vertices exist
    if (temp_vertices.count(first_vertex_id) == 0) {
        std::cout << "Vertex with ID " << first_vertex_id << " does not exist." << std::endl;
        return;
    }
    else if (temp_vertices.count(second_vertex_id) == 0) {
        std::cout << "Vertex with ID " << second_vertex_id << " does not exist." << std::endl;
        return;
    }

    edge_ptr->setFirstVertex(this->temp_vertices[first_vertex_id]);
    edge_ptr->setSecondVertex(this->temp_vertices[second_vertex_id]);

    temp_edges.insert(std::make_pair(id, edge_ptr));
}

void Optimization_General::removeEdge(int id) {
    this->temp_edges.erase(id);
}

Eigen::VectorXd Optimization_General::getEdgeMeasurement(int id) {
    if (temp_edges.count(id) == 0) {
        std::cout << "Edge with ID " << id << " does not exist." << std::endl;
        return Eigen::VectorXd();
    }

    return temp_edges[id]->getMeasurement();
}

//normal optimazation process 
void Optimization_General::optimize(int iterations) {
    double th1 = 1e-12;
    double th2 = 1e-12;
    double update_norm = 0;
    double b_max = 0;
    int current_iteration = 0;
    double cost = 0;
    double last_cost = std::numeric_limits<double>::infinity();
    Eigen::VectorXd poseUpdate;

    std::cout << "Optimization started! \n" << std::endl;

    //build the covariance matrix
    buildCovarianceMatrix();
    Eigen::MatrixXd* Cov = this->Cov;
    //std::cout << "Covariance matrix built" << std::endl;
    //std::cout << *Cov << std::endl;
    //Eigen::MatrixXd Cov_inv = Cov->inverse(); // this takes a lot of time
    Eigen::MatrixXd Cov_inv = inverseDiagonal(*Cov);


    //build the jacobian matrix
    buildJacobian();
    Eigen::MatrixXd* J = this->Jacobian;
    //std::cout << *J << std::endl;


    //build the A matrix
    Eigen::MatrixXd A = J->transpose() * Cov_inv * *J;
    //std::cout << "A matrix:\n " << A << std::endl;
    //build the error vector
    buildErrorVector();
    Eigen::VectorXd* errorVec = this->errorVec;
    //std::cout << "Error vector & Measurement Vector \n" << *errorVec << std::endl;
    
    cost = (errorVec->transpose() * *errorVec).norm() / errorVec->size() ;
    

    //build the b vector
    Eigen::VectorXd b =-1 * J->transpose() * Cov_inv * *errorVec;
    
    b_max = abs(b.maxCoeff());

    //std::cout << "Iterative Step \n";
    while (b_max > th1 && current_iteration < iterations) {
        current_iteration++;
        //solve the linear system
        poseUpdate = A.ldlt().solve(b);
		//std::cout << "i: "<< current_iteration << "| Pose update: \n" << poseUpdate.transpose() << std::endl;
        update_norm = poseUpdate.norm();

        std::cout << "cur_iter: " << current_iteration << " | cost: " << cost << " | update_norm: " << update_norm << " | b_max: " << b_max << std::endl;

        if (update_norm < th2){
            std::cout << "Update norm is less than threshold: " << update_norm << " < " << th2 << std::endl;
            break;
        }
        

        //update the pose and landmark vertices
        this->updateEstimates(poseUpdate);
        

        //build the jacobian matrix
        buildJacobian();
        J = this->Jacobian;

        //build the A matrix
        A = J->transpose() * Cov_inv * *J;

        //build the error vector
        buildErrorVector();
        errorVec = this->errorVec;

        //build the b vector
        b = J->transpose() * Cov_inv * *errorVec;

        cost = (errorVec->transpose() * *errorVec).norm() / errorVec->size();

        if (cost >= last_cost) {
            std::cout << "\ncost: cur_cost " << cost << " >= last_cost " << last_cost <<"| cur - last: "<< cost - last_cost << std::endl;
            break;
        }
        last_cost = cost;
        b_max = abs(b.maxCoeff());
    }
    std::cout << "\nOptimization finished\n"<<"b max :" << b_max << "| Iterations: " << current_iteration;
    std::cout << " Final cost: " << cost << " | update_norm: " << update_norm << std::endl;
}

void Optimization_General::initialize() {

    std::cout << "edge_sizes: ";
    for (const auto& edge_size : this->edge_sizes) {
        std::cout << edge_size << ", ";
    }
    std::cout << " | ";
    std::cout << "vertex_sizes: ";
    for (const auto& vertex_size : this->vertex_sizes) {
        std::cout << vertex_size << ", ";
    }
    std::cout << "\n";

    std::cout << "Total Vertices: " << temp_vertices.size()<< "\n";

    vertex_types.resize(vertex_sizes.size());
    general_vertices.resize(vertex_sizes.size());

    int new_vertex_id;
    int fixed_vertex_id;
    int vertex_type;
    general_vertex* vertex_ptr;

    for (auto& pair : temp_vertices) {
        vertex_ptr = pair.second;
        vertex_type = vertex_ptr->getType();
        vertex_types[vertex_type]++;

        if (vertex_ptr->getFixed()) {
            fixed_vertex_id = -1 * (static_cast<int>(this->fixed_vertices.size())) - 1;

            vertex_ptr->setId(fixed_vertex_id);
            this->fixed_vertices.push_back(vertex_ptr);

            // this->general_vertices_map.insert(std::make_pair(vertex_ptr->getGlobalId(), fixed_vertex_id));
            // this->fixed_vertex_count = -1 * (fixed_vertex_id + 1) + 1;

            this->fixed_vertex_count = this->fixed_vertices.size();

        }
        else {
            new_vertex_id = static_cast<int>(this->general_vertices[vertex_ptr->getType()].size());
            vertex_ptr->setId(new_vertex_id);
            this->general_vertices[vertex_type].push_back(vertex_ptr);

            // this->general_vertices_map.insert(std::make_pair(vertex_ptr->getGlobalId(), new_vertex_id));

            this->vertex_count += 1;
        }
    }

    int new_edge_id;
    general_edge* edge_ptr;

    for (auto& pair : temp_edges) {
        edge_ptr = pair.second;
        new_edge_id = static_cast<int>(this->general_edges.size());
        edge_ptr->initialize(new_edge_id);
        this->general_edges.push_back(edge_ptr);
        // this->general_edges_map.insert(std::make_pair(edge_ptr->getGlobalId(), new_edge_id));
        this->general_edge_count = new_edge_id + 1;
    }

    std::cout << "Fixed vertices: " << this->fixed_vertex_count << "   |   Total Edges: " << this->general_edge_count << "\n";
    std::cout << "#Vertex types: "<< this->general_vertices.size()<<" \n";
    for (int i = 0; i < this->general_vertices.size(); i++) {
		std::cout << "type " <<i << ": " << this->general_vertices[i].size() << "\n";
	}
    std::cout << "\n";
}