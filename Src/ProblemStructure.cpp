#include "ProblemStructure.h"

ProblemStructure::ProblemStructure()
{
}

ProblemStructure::~ProblemStructure()
{
}

void ProblemStructure::initialize()
{
#ifndef NDEBUG
	assert(this->p != nullptr);
#endif // !NDEBUG

	parameters->setPropertise(this->p);
	jacobian->setPropertise(this->p);
	residual->setPropertise(this->p);
	Hessian->setPropertise(this->p);
	bVector->setPropertise(this->p);
	solver->setPropertise(this->p);

	if (this->timer) {
		parameters->setTimer(this->timer);
		jacobian->setTimer(this->timer);
		residual->setTimer(this->timer);
		Hessian->setTimer(this->timer);
		bVector->setTimer(this->timer);
		solver->setTimer(this->timer);
	}


	parameters->initialize();
	jacobian->initialize();
	residual->initialize();
	Hessian->initialize();
	bVector->initialize();
	solver->initialize();
}

void ProblemStructure::finalize()
{
	if (this->isInitialized)
	{	
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
