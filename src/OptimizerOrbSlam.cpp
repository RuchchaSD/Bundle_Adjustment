// OptimizerOrbSlam.cpp
#include "OptimizerOrbSlam.h"

void OptimizerOrbSlam::addVertex(int globalId, int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed, std::map<int, std::shared_ptr<Vertex>>& globalVertexMap, std::map<int, std::shared_ptr<Vertex>>& vertexMap) {
    if (isFixed && (!p->hasFixedVertices)) {
        std::cerr << "If there are fixed vertices you should give it to the optimizer in the settings object.";
    }
    std::shared_ptr<Vertex> vertex(new Vertex(parameters, isFixed));
    vertex->setGlobalId(globalId);
    vertex->setId(id);
    globalVertexMap.insert(std::make_pair(globalId, vertex));
    vertexMap.insert(std::make_pair(id, vertex));
}

OptimizerOrbSlam::OptimizerOrbSlam() : OptimizerBase() {
    lastEdgePair = new std::pair<int, std::shared_ptr<std::map<int, std::shared_ptr<edge>>>>(-1, std::make_shared<std::map<int, std::shared_ptr<edge>>>());
    this->structure = std::make_unique<ProblemStructure>();
}

OptimizerOrbSlam::~OptimizerOrbSlam() {
}

void OptimizerOrbSlam::addVertex_type1(int id, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed) {
    if (parameters->size() == this->p->vertex1Size) {
        addVertex(id, p->numActiveVertices1, parameters, isFixed, globalVerticesType1, verticesType1);
        p->numActiveVertices1++;
    }
    else {
        std::cerr << "Error: OptimizerOrbSlam::addVertex_type1 id: " << id << " parameters size is not correct" << std::endl;
    }
}

std::shared_ptr<Eigen::VectorXd> OptimizerOrbSlam::getVertex_type1_Parameters(int id) {
    if (globalVerticesType1.count(id)) {
        return globalVerticesType1[id]->getParameters();
    }
    else {
        std::cerr << "Error: OptimizerOrbSlam::getVertex_type1_Parameters id: " << id << " does not exist" << std::endl;
        return nullptr;
    }
}

void OptimizerOrbSlam::addVertex_type2(int globalId, std::shared_ptr<Eigen::VectorXd> parameters, bool isFixed) {
    if (parameters->size() == this->p->vertex2Size) {
        addVertex(globalId, p->numActiveVertices2, parameters, isFixed, globalVerticesType2, verticesType2);
        p->numActiveVertices2++;
    }
    else {
        std::cerr << "Error: OptimizerOrbSlam::addVertex_type2 id: " << globalId << " parameters size is not correct" << std::endl;
    }
}

std::shared_ptr<Eigen::VectorXd> OptimizerOrbSlam::getVertex_type2_Parameters(int id) {
    if (globalVerticesType2.count(id)) {
        return globalVerticesType2[id]->getParameters();
    }
    else {
        std::cerr << "Error: OptimizerOrbSlam::getVertex_type2_Parameters id: " << id << " does not exist" << std::endl;
        return nullptr;
    }
}

void OptimizerOrbSlam::makeEdge(int globalId, std::shared_ptr<Eigen::VectorXd>& observations, std::shared_ptr<Eigen::VectorXd>& sigma, int& vertex1_id, int& vertex2_id) {
    this->tempEdge = std::make_shared<edge>();
    tempEdge->setGlobalId(globalId);
    tempEdge->setObservations(observations);
    tempEdge->setSigma(sigma);
    tempEdge->setVertices(
        globalVerticesType1[vertex1_id],
        globalVerticesType2[vertex2_id]
    );

    lastEdgePair->second->insert(std::make_pair(vertex1_id, tempEdge));
    this->p->numEdges++;
    this->tempEdge.reset();
}

void OptimizerOrbSlam::addEdge(int globalId, int vertex1_id, int vertex2_id, std::shared_ptr<Eigen::VectorXd> observations, std::shared_ptr<Eigen::VectorXd> sigma) {
    if (p->numEdges == 0) {
        p->verticesSetted = true;
        this->p->setNumObservationVectors();
    }

    if (lastEdgePair->first != vertex2_id && edges.count(vertex2_id)) {
        lastEdgePair = new std::pair<int, std::shared_ptr<std::map<int, std::shared_ptr<edge>>>>(vertex2_id, edges.at(vertex2_id));
    }
    else if (lastEdgePair->first != vertex2_id) {
        lastEdgePair = new std::pair<int, std::shared_ptr<std::map<int, std::shared_ptr<edge>>>>(vertex2_id, std::make_shared<std::map<int, std::shared_ptr<edge>>>());
        edges.insert(*lastEdgePair);
    }

    if (lastEdgePair->second->count(vertex1_id)) {
        std::cerr << "Error: OptimizerOrbSlam::addEdge edge already exists" << std::endl;
    }
    else {
        makeEdge(globalId, observations, sigma, vertex1_id, vertex2_id);
        p->numOberservationsPerVertex1[vertex1_id]++;
        p->numOberservationsPerVertex2[vertex2_id - p->numActiveVertices1]++;
    }
}

void OptimizerOrbSlam::initialize() {
    int vertex1Location = 0;
    int vertex2Location = 0;
    int edgeId = 0;

    edge* edge = nullptr;

    if (this->timer != nullptr) {
        structure->setTimer(timer);
    }

    p->initialize();
    structure->setPropertise(p);
    structure->initialize();

    std::shared_ptr<JacobianCompressed> jacobian = structure->getJacobian();
    std::shared_ptr<ResidualsCompressed> residuals = structure->getResidual();
    std::shared_ptr<HessianOrb> Hessian = structure->getHessian();
    std::shared_ptr<bVectorOrb> bVector = structure->getbVector();
    std::shared_ptr<ParameterVectorBase> parameters = structure->getParameters();
    std::shared_ptr<SolverOrb> solver = structure->getSolver();
    this->optimizerAlgorithm = std::make_unique<OptimizationAlgorithmLevenberg>();

    for (auto pair : verticesType1) {
        pair.second->setParametersMap(std::move(parameters->getParameterVectorMap(true, pair.first)));
    }
    for (auto pair : verticesType2) {
        pair.second->setParametersMap(std::move(parameters->getParameterVectorMap(false, pair.first)));
    }

    std::shared_ptr<InformationMatrixOrb> informationMatrix = residuals->getInformationVector();
    std::shared_ptr<Eigen::VectorXd> informationVector = informationMatrix->getInfVecSqrt();

    std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets = std::make_shared<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>>(p->numEdges);

    jacobian->setParameterSets(parameterSets);
    residuals->setParameterSets(parameterSets);

    std::shared_ptr<Eigen::VectorXd> observationVector = std::make_shared<Eigen::VectorXd>(p->totalObservations);
    residuals->setObservationsVector(observationVector);

    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType1 = std::make_shared<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>(p->numActiveVertices1);
    for (int i = 0; i < p->numActiveVertices1; i++) {
        jacobiansToType1->at(i).reserve(p->numOberservationsPerVertex1[i]);
    }

    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType2 = std::make_shared<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>(p->numActiveVertices2);
    for (int i = 0; i < p->numActiveVertices2; i++) {
        jacobiansToType2->at(i).reserve(p->numOberservationsPerVertex2[i]);
    }

    std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToWBlocks = std::make_shared<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>(p->numEdges);

    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType1 = std::make_shared<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>>(p->numActiveVertices1);
    for (int i = 0; i < p->numActiveVertices1; i++) {
        residualsToType1->at(i).reserve(p->numOberservationsPerVertex1[i]);
    }

    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType2 = std::make_shared<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>>(p->numActiveVertices2);
    for (int i = 0; i < p->numActiveVertices2; i++) {
        residualsToType2->at(i).reserve(p->numOberservationsPerVertex2[i]);
    }

    Hessian->setjacobiansToABlocksObject(jacobiansToType1);
    Hessian->setjacobiansToBBlocksObject(jacobiansToType2);
    Hessian->setjacobiansToWBlocksObject(jacobiansToWBlocks);

    bVector->setResidualsToParameters1Object(residualsToType1);
    bVector->setResidualsToParameters2Object(residualsToType2);
    bVector->setJacobiansToType1Object(jacobiansToType1);
    bVector->setJacobiansToType2Object(jacobiansToType2);

    solver->setAHessianBlocksObject(Hessian->getABlocks());
    solver->setBHessianBlocksObject(Hessian->getBBlocks());
    solver->setBvector(bVector->getBVector());
    solver->setUpdateVector(parameters->getUpdateVector());

    for (auto& edgeSet : edges) {
        for (auto& edge : *edgeSet.second) {
            Vertex* v1 = edge.second->getVertex1().get();
            Vertex* v2 = edge.second->getVertex2().get();

            vertex1Location = v1->getId();
            vertex2Location = v2->getId();
            edge.second->setId(edgeId);

            parameterSets->at(edgeId).first = v1->getParametersMap();
            parameterSets->at(edgeId).second = v2->getParametersMap();

            observationVector->segment(static_cast<Eigen::Index>(edgeId) * p->edgeSize, p->edgeSize) = *edge.second->getObservations();
            informationVector->segment(static_cast<Eigen::Index>(edgeId) * p->edgeSize, p->edgeSize) = *edge.second->getinvSigma();

            Hessian->setJacobianVertex(vertex1Location, vertex2Location, edgeId, std::move(jacobian->getJacobianVertex(true, edgeId)), std::move(jacobian->getJacobianVertex(false, edgeId)));
            bVector->setResidualEdge(vertex1Location, vertex2Location, residuals->getResiduals(edgeId));
            solver->setEdge(vertex1Location, vertex2Location, edgeId, Hessian->getWblock(edgeId));
            edgeId++;
        }
    }

    optimizerAlgorithm->setPropertise(p);
    optimizerAlgorithm->setTimer(timer);
    optimizerAlgorithm->SetStructure(std::move(structure));
    optimizerAlgorithm->initialize();

    edges.clear();
    delete lastEdgePair;
    verticesType1.clear();
    verticesType2.clear();
    this->isInitialized = true;
}

void OptimizerOrbSlam::finalize() {
    if (this->isInitialized) {
        for (auto pair : globalVerticesType1) {
            pair.second->updarteParameters();
        }

        for (auto pair : globalVerticesType2) {
            pair.second->updarteParameters();
        }
        optimizerAlgorithm->finalize();
        this->isInitialized = false;
    }
    else {
        std::cerr << "Error: OptimizerOrbSlam::finalize :- not initialized" << std::endl;
    }
}

void OptimizerOrbSlam::Optimize(int n) {
    if (this->isInitialized) {
        optimizerAlgorithm->Optimize(n);
    }
    else {
        std::cerr << "Error: OptimizerOrbSlam::Optimize not initialized" << std::endl;
    }
}
