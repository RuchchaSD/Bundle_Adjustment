// OptimizerOrbSlam.h
#pragma once
#include "OptimizerBase.h"
#include <Eigen/Dense>
#include <Eigen/core>
#include "ORBData.h"
#include "OptimizationAlgorithmGN.h"
#include "OptimizationAlgorithmLevenberg.h"

/**
 * @class OptimizerOrbSlam
 * @brief Specific optimizer implementation for ORB-SLAM, managing vertices and edges.
 */
class OptimizerOrbSlam : public OptimizerBase {
private:
    std::map<int, std::shared_ptr<Vertex>> globalVerticesType1; ///< Global vertex map for type 1 vertices.
    std::map<int, std::shared_ptr<Vertex>> globalVerticesType2; ///< Global vertex map for type 2 vertices.
    std::map<int, std::shared_ptr<Vertex>> verticesType1; ///< Internal vertex map for type 1 vertices.
    std::map<int, std::shared_ptr<Vertex>> verticesType2; ///< Internal vertex map for type 2 vertices.
    std::map<int, std::shared_ptr<std::map<int, std::shared_ptr<edge>>>> edges; ///< Map of edges.
    std::pair<int, std::shared_ptr<std::map<int, std::shared_ptr<edge>>>>* lastEdgePair; ///< Last edge pair for adding edges.
    std::shared_ptr<edge> tempEdge; ///< Temporary edge for edge creation.

    /**
     * @brief Adds a vertex to the optimizer.
     * @param globalId Global identifier for the vertex.
     * @param id Identifier for the vertex.
     * @param parameters Parameters of the vertex.
     * @param isFixed Boolean indicating if the vertex is fixed.
     * @param globalVertexMap Global vertex map.
     * @param vertexMap Internal vertex map.
     */
    void addVertex(int globalId, int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed, std::map<int, std::shared_ptr<Vertex>>& globalVertexMap, std::map<int, std::shared_ptr<Vertex>>& vertexMap);

    /**
     * @brief Creates an edge between two vertices.
     * @param globalId Global identifier for the edge.
     * @param observations Observations for the edge.
     * @param sigma Information matrix for the edge.
     * @param vertex1_id Identifier for the first vertex.
     * @param vertex2_id Identifier for the second vertex.
     */
    void makeEdge(int globalId, std::shared_ptr<Eigen::VectorXd>& observations, std::shared_ptr<Eigen::VectorXd>& sigma, int& vertex1_id, int& vertex2_id);

public:
    /**
     * @brief Constructor for OptimizerOrbSlam.
     */
    OptimizerOrbSlam();

    /**
     * @brief Destructor for OptimizerOrbSlam.
     */
    ~OptimizerOrbSlam();

    /**
     * @brief Adds a vertex of type 1 to the optimizer.
     * @param id Identifier for the vertex.
     * @param parameters Parameters of the vertex.
     * @param isFixed Boolean indicating if the vertex is fixed.
     */
    void addVertex_type1(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false) override;

    /**
     * @brief Gets the parameters of a vertex of type 1.
     * @param id Identifier for the vertex.
     * @return Shared pointer to the parameters of the vertex.
     */
    std::shared_ptr<Eigen::VectorXd> getVertex_type1_Parameters(int id) override;

    /**
     * @brief Adds a vertex of type 2 to the optimizer.
     * @param globalId Global identifier for the vertex.
     * @param parameters Parameters of the vertex.
     * @param isFixed Boolean indicating if the vertex is fixed.
     */
    void addVertex_type2(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed = false) override;

    /**
     * @brief Gets the parameters of a vertex of type 2.
     * @param id Identifier for the vertex.
     * @return Shared pointer to the parameters of the vertex.
     */
    std::shared_ptr<Eigen::VectorXd> getVertex_type2_Parameters(int id) override;

    /**
     * @brief Adds an edge to the optimizer.
     * @param id Identifier for the edge.
     * @param vertex1_id Identifier for the first vertex.
     * @param vertex2_id Identifier for the second vertex.
     * @param observations Observations for the edge.
     * @param sigma Information matrix for the edge.
     */
    void addEdge(int id, int vertex1_id, int vertex2_id, std::shared_ptr<Eigen::VectorXd> observations, std::shared_ptr<Eigen::VectorXd> sigma) override;

    /**
     * @brief Initializes the optimizer with the required structures and data.
     *
     * This function sets up the optimizer by initializing the necessary data structures,
     * setting the problem properties, and configuring the optimization algorithm. It also
     * prepares the parameter vectors, Jacobians, residuals, Hessians, and solver components
     * for the optimization process.
     *
     * Inside the function:
     * - Initializes the ProblemStructure and sets properties and timer if available.
     * - Retrieves shared pointers to key components (Jacobian, residuals, Hessian, bVector, parameters, solver).
     * - Creates and configures the optimization algorithm (Levenberg-Marquardt).
     * - Maps parameters of vertices to the optimizer's parameter vectors.
     * - Configures the Jacobian, residuals, Hessian, and bVector with observation and parameter sets.
     * - Sets up memory locations for Jacobian blocks in the Hessian and residuals.
     * - Configures the solver with Hessian blocks, bVector, and update vector.
     * - Iterates over edges to set up their parameter mappings and residuals.
     * - Finalizes the initialization by clearing temporary structures and setting the initialized flag.
     *
     * Key data structures:
     * - `structure`: A unique pointer to the ProblemStructure, holding the problem's data.
     * - `parameterSets`: A vector of parameter pairs (maps) for the vertices connected by edges.
     * - `jacobiansToType1`, `jacobiansToType2`: Vectors of Jacobian blocks for the vertices.
     * - `residualsToType1`, `residualsToType2`: Vectors of residual maps for the vertices.
     * - `observationVector`: A vector holding the observations for the residuals.
     * - `informationVector`: A vector holding the information (inverse sigma) for the residuals.
     *
     * Equations/operations:
     * - Updates parameter vectors and observation vectors based on edge connections.
     * - Sets Jacobian blocks in the Hessian.
     * - Configures residual mappings for the bVector.
     */
    void initialize() override;

    /**
     * @brief Finalizes the optimizer.
     */
    void finalize() override;

    /**
     * @brief Optimizes the problem for a given number of iterations.
     * @param n Number of iterations.
     */
    void Optimize(int n) override;
};
