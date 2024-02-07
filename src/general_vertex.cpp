#include "../include/general_vertex.h"

void general_vertex::initialize(int id)
{
    if (this->id == std::numeric_limits<int>::min() && this->isInitialized == false) {
        this->id = id;
    }
    this->isInitialized = true;
}

general_vertex::general_vertex(int g_id, int id, int vertex_type) : id(id), global_id(g_id), vertex_type(vertex_type),
                isInitialized(false),isFixed(false),temp_id(-1){
 /*   this->isInitialized = false;
    this->isFixed = false;
    this->temp_id = -1;*/
}

general_vertex::general_vertex(int g_id) : global_id(g_id) {
    this->id = std::numeric_limits<int>::min();
    this->vertex_type = -20;
    this->isInitialized = false;
    this->isFixed = false;
    this->temp_id = -1;
}

// Type
void general_vertex::setType(int vertex_type)
{
    this->vertex_type = vertex_type;
}

int general_vertex::getType()
{
    return this->vertex_type;
}

// Id
int general_vertex::getId()
{
    return this->id;
}

int general_vertex::getGlobalId()
{
    // std::cout << "global id: " << this->global_id << std::endl;
    return this->global_id;
}

// Parameters
void general_vertex::setParameters(const Eigen::VectorXd& parameters)
{
    this->parameters = parameters;
}

void general_vertex::updateParameters(const Eigen::VectorXd& parametersUpdate)
{
    this->previous_parameters = this->parameters;
    this->parameters = this->parameters + parametersUpdate;
}

void general_vertex::revertParameters(const Eigen::VectorXd& parametersUpdate)
{
    this->parameters = this->previous_parameters;
}

Eigen::VectorXd general_vertex::getParameters()
{
    return this->parameters;
}

// Edges
void general_vertex::addEdge(general_edge* edge, general_vertex* general_vertex_ptr)
{
    if (!this->isInitialized) {
        this->attached_edges.insert(std::make_pair(edge->getId(), edge));
        this->attached_vertices.insert(std::make_pair(general_vertex_ptr->getId(), general_vertex_ptr));
        this->initialize(temp_id);
    }
}

const std::map<int, general_edge*> general_vertex::getAttachedEdges()
{
    return this->attached_edges;
}

const std::map<int, general_vertex*> general_vertex::getAttachedVertices()
{
    return this->attached_vertices;
}

// Fixedness
void general_vertex::setFixed(bool isFixed)
{
    this->isFixed = isFixed;
}

bool general_vertex::getFixed()
{
    return this->isFixed;
}

// Initialization
bool general_vertex::getInitialized()
{
    return this->isInitialized;
}

void general_vertex::setId(int id)
{
    this->temp_id = id;
}