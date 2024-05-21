// ORBData.h
#pragma once
#include <iostream>
#include <Eigen/Core>

/**
 * @class Vertex
 * @brief Represents a vertex in the optimization problem.
 */
class Vertex {
protected:
    int globalId; // global id that the user assigns to the vertex
    int id; // id that the graph assigns to the vertex
    bool fixed = false;
    std::shared_ptr<Eigen::VectorXd> parameters;
    std::shared_ptr<Eigen::Map<Eigen::VectorXd>> parametersMap;

public:
    /**
     * @brief Default constructor.
     */
    Vertex() : fixed(false), parameters(nullptr), parametersMap(nullptr)
    {
        this->globalId = -1;
        this->id = -1;
    }

    /**
     * @brief Parameterized constructor.
     * @param parameters Parameters for the vertex.
     * @param isFixed Indicates if the vertex is fixed.
     */
    Vertex(std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed) : parameters(parameters), fixed(isFixed), parametersMap(nullptr)
    {
        this->globalId = -1;
        this->id = -1;
    }

    /**
     * @brief Destructor.
     */
    ~Vertex() {}

    /**
     * @brief Sets the ID of the vertex.
     * @param id ID to set.
     */
    void setId(int id) { this->id = id; }

    /**
     * @brief Gets the ID of the vertex.
     * @return ID of the vertex.
     */
    int getId() { return id; }

    /**
     * @brief Sets the global ID of the vertex.
     * @param gId Global ID to set.
     */
    void setGlobalId(int gId) { this->globalId = gId; }

    /**
     * @brief Gets the global ID of the vertex.
     * @return Global ID of the vertex.
     */
    int getGlobalId() { return globalId; }

    /**
     * @brief Sets whether the vertex is fixed.
     * @param fixed True if fixed, false otherwise.
     */
    void setFixed(bool fixed) { this->fixed = fixed; }

    /**
     * @brief Checks if the vertex is fixed.
     * @return True if fixed, false otherwise.
     */
    bool isFixed() { return fixed; }

    /**
     * @brief Sets the parameters of the vertex.
     * @param parameters Parameters to set.
     */
    void setParameters(Eigen::VectorXd& parameters) { this->parameters = std::make_shared<Eigen::VectorXd>(parameters); }

    /**
     * @brief Sets the parameters of the vertex.
     * @param parameters Shared pointer to parameters to set.
     */
    void setParameters(std::shared_ptr<Eigen::VectorXd> parameters) { this->parameters = parameters; }

    /**
     * @brief Gets the parameters of the vertex.
     * @return Shared pointer to the parameters.
     */
    std::shared_ptr<Eigen::VectorXd> getParameters() { return parameters; }

    /**
     * @brief Sets the parameter map of the vertex.
     * @param parametersMap Shared pointer to the parameter map.
     */
    void setParametersMap(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> parametersMap) {
        this->parametersMap = parametersMap;
        *this->parametersMap = *parameters;
    }

    /**
     * @brief Updates the parameters with the parameter map.
     */
    void updarteParameters() { *parameters = *parametersMap; }

    /**
     * @brief Gets the parameter map of the vertex.
     * @return Shared pointer to the parameter map.
     */
    std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getParametersMap() { return parametersMap; }
};

/**
 * @class edge
 * @brief Represents an edge in the optimization problem.
 */
class edge {
protected:
    int globalId;
    int id;
    std::shared_ptr<Vertex> v1;
    std::shared_ptr<Vertex> v2;
    std::shared_ptr<Eigen::VectorXd> observations;
    std::shared_ptr<Eigen::VectorXd> covariance;
    std::shared_ptr<Eigen::VectorXd> sigma;
    std::shared_ptr<Eigen::Map<Eigen::VectorXd>> observationsMap;
    std::shared_ptr<Eigen::Map<Eigen::VectorXd>> covarianceMap;

public:
    /**
     * @brief Default constructor.
     */
    edge()
    {
        this->globalId = -1;
        this->id = -1;
    }


    /**
     * @brief Destructor.
     */
    ~edge() {}

    /**
     * @brief Sets the ID of the edge.
     * @param id ID to set.
     */
    void setId(int id) { this->id = id; }

    /**
     * @brief Gets the ID of the edge.
     * @return ID of the edge.
     */
    int getId() { return id; }

    /**
     * @brief Sets the global ID of the edge.
     * @param id Global ID to set.
     */
    void setGlobalId(int id) { this->globalId = id; }

    /**
     * @brief Gets the global ID of the edge.
     * @return Global ID of the edge.
     */
    int getGlobalId() { return globalId; }

    /**
     * @brief Sets the vertices associated with the edge.
     * @param v1 First vertex.
     * @param v2 Second vertex.
     */
    void setVertices(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2) {
        this->v1 = v1;
        this->v2 = v2;
    }

    /**
     * @brief Sets the observations of the edge.
     * @param observations Observations to set.
     */
    void setObservations(Eigen::VectorXd& observations) { this->observations = std::make_shared<Eigen::VectorXd>(observations); }

    /**
     * @brief Sets the observations of the edge.
     * @param observations Shared pointer to observations to set.
     */
    void setObservations(std::shared_ptr<Eigen::VectorXd> observations) { this->observations = observations; }

    /**
     * @brief Gets the observations of the edge.
     * @return Shared pointer to the observations.
     */
    std::shared_ptr<Eigen::VectorXd> getObservations() { return observations; }

    /**
     * @brief Sets the covariance of the edge.
     * @param covariance Covariance to set.
     */
    void setCovariance(Eigen::VectorXd& covariance) { this->covariance = std::make_shared<Eigen::VectorXd>(covariance); }

    /**
     * @brief Sets the covariance of the edge.
     * @param covariance Shared pointer to covariance to set.
     */
    void setCovariance(std::shared_ptr<Eigen::VectorXd> covariance) { this->covariance = covariance; }

    /**
     * @brief Gets the covariance of the edge.
     * @return Shared pointer to the covariance.
     */
    std::shared_ptr<Eigen::VectorXd> getCovariance() { return covariance; }

    /**
     * @brief Sets the sigma of the edge.
     * @param sigma Sigma to set.
     */
    void setSigma(Eigen::VectorXd& sigma) { this->sigma = std::make_shared<Eigen::VectorXd>(sigma); }

    /**
     * @brief Sets the sigma of the edge.
     * @param sigma Shared pointer to sigma to set.
     */
    void setSigma(std::shared_ptr<Eigen::VectorXd> sigma) { this->sigma = sigma; }

    /**
     * @brief Gets the sigma of the edge.
     * @return Shared pointer to the sigma.
     */
    std::shared_ptr<Eigen::VectorXd> getSigma() { return sigma; }

    /**
     * @brief Gets the inverse of the sigma of the edge.
     * @return Shared pointer to the inverse sigma.
     */
    std::shared_ptr<Eigen::VectorXd> getinvSigma() {
        std::shared_ptr<Eigen::VectorXd> invSigma = std::make_shared<Eigen::VectorXd>(sigma->size());
        *invSigma = sigma->cwiseInverse();
        return invSigma;
    }

    /**
     * @brief Sets the observation map of the edge.
     * @param observationsMap Shared pointer to the observation map.
     */
    void setObservationsMap(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> observationsMap) {
        this->observationsMap = observationsMap;
        *this->observationsMap = *observations;
    }

    /**
     * @brief Sets the covariance map of the edge.
     * @param covarianceMap Shared pointer to the covariance map.
     */
    void setCovarianceMap(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> covarianceMap) {
        this->covarianceMap = covarianceMap;
        *this->covarianceMap = *covariance;
    }

    /**
     * @brief Gets the first vertex associated with the edge.
     * @return Shared pointer to the first vertex.
     */
    std::shared_ptr<Vertex> getVertex1() { return v1; }

    /**
     * @brief Gets the second vertex associated with the edge.
     * @return Shared pointer to the second vertex.
     */
    std::shared_ptr<Vertex> getVertex2() { return v2; }
};














