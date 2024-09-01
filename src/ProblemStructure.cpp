// ProblemStructure.cpp
#include "ProblemStructure.h"

ProblemStructure::ProblemStructure() {
    // Constructor implementation
}

ProblemStructure::~ProblemStructure() {
    // Destructor implementation
}

void ProblemStructure::initialize() {
#ifndef NDEBUG
    assert(this->p != nullptr);
#endif // !NDEBUG

    // Set properties for all components
    parameters->setPropertise(this->p);
    jacobian->setPropertise(this->p);
    residual->setPropertise(this->p);
    Hessian->setPropertise(this->p);
    bVector->setPropertise(this->p);
    solver->setPropertise(this->p);

    // Set timers for all components if timer is available
    if (this->timer) {
        parameters->setTimer(this->timer);
        jacobian->setTimer(this->timer);
        residual->setTimer(this->timer);
        Hessian->setTimer(this->timer);
        bVector->setTimer(this->timer);
        solver->setTimer(this->timer);
    }

    // Initialize all components
    parameters->initialize();
    jacobian->initialize();
    residual->initialize();
    Hessian->initialize();
    bVector->initialize();
    solver->initialize();
}

void ProblemStructure::finalize() {
    if (this->isInitialized) {
        // Finalize all components
        solver->finalize();
        bVector->finalize();
        Hessian->finalize();
        residual->finalize();
        jacobian->finalize();
        parameters->finalize();

        this->isInitialized = false;
    }
    else {
        std::cerr << "ProblemStructure::finalize: Not initialized" << std::endl;
    }
}