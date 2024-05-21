#include "DataStructureBases.h"

//InformationMatrixOrb
InformationMatrixOrb::InformationMatrixOrb() : InformationMatrixBase()
{
}
InformationMatrixOrb::~InformationMatrixOrb()
{
}


//InformationMatrixCompressed
InformationMatrixCompressed::InformationMatrixCompressed() : InformationMatrixOrb()
{
	this->infVec = nullptr;
	this->infVec_sqrt = nullptr;
}
InformationMatrixCompressed::~InformationMatrixCompressed()
{
}
void InformationMatrixCompressed::initialize()
{
	if (isInitialized)
		return;
#ifndef NDEBUG
	assert(p != nullptr);
#endif // !NDEBUG

	this->infVec = std::make_shared<Eigen::VectorXd>(p->totalObservations);
	this->infVec_sqrt = std::make_shared<Eigen::VectorXd>(p->totalObservations);
	this->isInitialized = true;
}
void InformationMatrixCompressed::finalize()
{
	if (isInitialized) {
		infVec.reset();
		infVec_sqrt.reset();
		this->isInitialized = false;
	}
}
void InformationMatrixCompressed::updateInfVec(int edgeLocation, const Eigen::VectorXd& vec)
{

#ifndef NDEBUG
	assert(vec.size() == p->edgeSize);
#endif // !NDEBUG
	for (int i = edgeLocation * p->edgeSize; i < (edgeLocation + 1) * p->edgeSize; i++)
	{
		double a = vec[i - edgeLocation * p->edgeSize];

#ifndef NDEBUG
		assert(a > 0);
#endif // !NDEBUG

		(*this->infVec_sqrt)[i] = 1 / a;
		(*this->infVec)[i] = (*this->infVec_sqrt)[i] * (*this->infVec_sqrt)[i];
	}
}
double InformationMatrixCompressed::getInfSqrt(int index)
{
#ifndef NDEBUG
	assert(index < infVec_sqrt->size());
#endif // !NDEBUG

	return (*this->infVec_sqrt)[index];
}
double InformationMatrixCompressed::getInf(int index)
{
#ifndef NDEBUG
	assert(index < infVec->size());
#endif // !NDEBUG
	return (*this->infVec)[index];
}

void InformationMatrixCompressed::printInformationMatrix()
{
	std::cout << "Information Matrix Sqrt: " << infVec_sqrt->transpose() << "\n\n\n" << std::endl;
}



//parameterVectorBase
parameterVectorBase::parameterVectorBase() : parameterUpdate(nullptr), BaseDataStructure()
{
	this->maxCoeffValue = 0.0;
	this->isBackedUp = false;
}
parameterVectorBase::~parameterVectorBase()
{
}

//parameterVectorMarginalized
parameterVectorMarginalized::parameterVectorMarginalized() : parameterVectorBase()
{
	this->activeParameterVector = std::make_shared<Eigen::VectorXd>();
	this->marginalizedParameterVector = std::make_shared<Eigen::VectorXd>();

	this->activeParameterVectorPrevious = std::make_shared<Eigen::VectorXd>();
	this->marginalizedParameterVectorPrevious = std::make_shared<Eigen::VectorXd>();

}
parameterVectorMarginalized::~parameterVectorMarginalized()
{
}
void parameterVectorMarginalized::initialize()
{
	if (isInitialized)
		return;

#ifndef NDEBUG
	assert(p != nullptr && parameterUpdate != nullptr);
#endif // !NDEBUG

	this->activeParameterVector->resize(p->totalType1Parameters);
	this->marginalizedParameterVector->resize(p->totalType2Parameters);

	this->activeParameterVectorPrevious->resizeLike(*activeParameterVector);
	this->marginalizedParameterVectorPrevious->resizeLike(*marginalizedParameterVector);

	this->updateVector = std::make_shared<Eigen::VectorXd>(p->totalWidth);
	this->activeUpdateParameterVectorMap = std::make_shared<Eigen::Map<Eigen::VectorXd>>(updateVector->data(), p->totalType1Parameters);
	this->marginalizedUpdateParameterVectorMap = std::make_shared<Eigen::Map<Eigen::VectorXd>>(updateVector->data() + p->totalType1Parameters, p->totalType2Parameters);

	if (p->DebugMode) {
		activeParameterVector->setOnes();
		marginalizedParameterVector->setOnes();
	}

	this->isInitialized = true;
}
void parameterVectorMarginalized::finalize()
{
	if (isInitialized) {

		activeUpdateParameterVectorMap.reset();
		marginalizedUpdateParameterVectorMap.reset();

		updateVector.reset();

		activeParameterVectorPrevious.reset();
		marginalizedParameterVectorPrevious.reset();


		activeParameterVector.reset();
		marginalizedParameterVector.reset();
		this->isInitialized = false;
	}
}
void parameterVectorMarginalized::setParameterVector(const Eigen::VectorXd& parameterVector)
{	
#ifndef NDEBUG
	assert(isInitialized);
	assert(parameterVector.size() == p->totalWidth);
#endif // !NDEBUG

	*activeParameterVector = parameterVector.segment(0, p->totalType1Parameters);
	*marginalizedParameterVector = parameterVector.segment(p->totalType1Parameters, p->totalType2Parameters);
}
void parameterVectorMarginalized::setActiveParameterVector(const Eigen::VectorXd& activeParameterVector)
{

#ifndef NDEBUG
	assert(isInitialized);
	assert(activeParameterVector.size() == p->totalType1Parameters);
#endif // !NDEBUG
	*this->activeParameterVector = activeParameterVector;
}
void parameterVectorMarginalized::setMarginalizedParameterVector(const Eigen::VectorXd& marginalizedParameterVector)
{
	
#ifndef NDEBUG
	assert(isInitialized);
	assert(marginalizedParameterVector.size() == p->totalType2Parameters);
#endif // !NDEBUG
	*this->marginalizedParameterVector = marginalizedParameterVector;
}
std::shared_ptr<Eigen::Map<Eigen::VectorXd>> parameterVectorMarginalized::getParameterVectorMap(bool isFirst, int location)
{
	if (isFirst)
		return std::make_shared<Eigen::Map<Eigen::VectorXd>>(activeParameterVector->data() + location * p->vertex1Size, p->vertex1Size);
	else
		return std::make_shared<Eigen::Map<Eigen::VectorXd>>(marginalizedParameterVector->data() + location * p->vertex2Size, p->vertex2Size);
}
std::shared_ptr<Eigen::VectorXd> parameterVectorMarginalized::getActiveParameterVector()
{
	
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	this->isBackedUp = false;
	return this->activeParameterVector;
}
std::shared_ptr<Eigen::VectorXd> parameterVectorMarginalized::getMarginalizedParameterVector()
{
	
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	this->isBackedUp = false;
	return this->marginalizedParameterVector;
}
void parameterVectorMarginalized::setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector)
{
	
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	
#ifndef NDEBUG
	assert(updateVector->size() == p->totalWidth);
#endif // !NDEBUG
	*this->updateVector = *updateVector;
}
void parameterVectorMarginalized::updateParameterVector()
{
	
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	backUp();
	parameterUpdate->update(
		*activeParameterVector,
		*activeUpdateParameterVectorMap
	);

	parameterUpdate->update(
		*marginalizedParameterVector,
		*marginalizedUpdateParameterVectorMap
	);

	this->maxCoeffValue = std::max(activeParameterVector->maxCoeff(), marginalizedParameterVector->maxCoeff());
}
void parameterVectorMarginalized::printParameterVector()
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	std::cout << "Active Parameters: " << activeParameterVector->transpose() <<"\n\n\n" << std::endl;
	std::cout << "Marginalized Parameters: " << marginalizedParameterVector->transpose() << "\n\n\n" << std::endl;
}
void parameterVectorMarginalized::backUp()
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	*activeParameterVectorPrevious = *activeParameterVector;
	*marginalizedParameterVectorPrevious = *marginalizedParameterVector;
	this->isBackedUp = true;
}
void parameterVectorMarginalized::restore()
{

	*activeParameterVector = *activeParameterVectorPrevious;
	*marginalizedParameterVector = *marginalizedParameterVectorPrevious;
}
double parameterVectorMarginalized::maxCoeff()
{
#ifndef NDEBUG
	assert(isInitialized && isBackedUp);
#endif // !NDEBUG
	return this->maxCoeffValue;
}



//JacobianBase
JacobianBase::JacobianBase() : kernel(nullptr), BaseDataStructure()
{
	this->isUpdated = false;
}
JacobianBase::~JacobianBase()
{
}
void JacobianCompressed::computeJacobianVertex(bool reverse, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param1, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param2, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian, double stepSize)
{
#define NumericDiff
	Eigen::Map<Eigen::VectorXd> &estimatedParameters1 = *param1;
	Eigen::Map<Eigen::VectorXd> &estimatedParameters2 = *param2;
#ifdef  NumericDiff
	Eigen::VectorXd y0(p->edgeSize); // y_est(estimate)
	Eigen::VectorXd y1(p->edgeSize);// y_est(estimate + h)
	double temp = 0;

	reprojection->reproject(estimatedParameters1, estimatedParameters2, y0);// calculate y_est(estimate)
	if (reverse) {
		//∂e/∂x = (de(estimate + step_size) - de(estimate)) / dstep_size => ( (y_est(estimate + h) - y_est(actual)) - (y_est(estimate) - y_est(actual)) )  ) /h => (y_est(estimate + h) - y_est(estimate)) / h
		for (int i = 0; i < p->vertex2Size; i++) {
			//update the parameter
			temp = estimatedParameters2[i];
			estimatedParameters2[i] += stepSize;
			// calculate y_est(estimate + h)
			reprojection->reproject(estimatedParameters1, estimatedParameters2, y1); // calculate y_est(estimate + h)
			//calculate the partial derivative
			for (int j = 0; j < p->edgeSize; j++) {
				(*jacobian)(j, i) = (y1[j] - y0[j]) / stepSize;
			}
			estimatedParameters2[i] = temp; //reset the parameter
}
	}
	else {
		for (int i = 0; i < p->vertex1Size; i++) {
			//update the parameter
			temp = estimatedParameters1[i];
			estimatedParameters1[i] += stepSize;
			// calculate y_est(estimate + h)
			reprojection->reproject(estimatedParameters1, estimatedParameters2, y1); // calculate y_est(estimate + h)
			//calculate the partial derivative
			for (int j = 0; j < p->edgeSize; j++) {
				(*jacobian)(j, i) = (y1[j] - y0[j]) / stepSize;
			}
			estimatedParameters1[i] = temp; //reset the parameter
		}
	}



#else

	std::cerr << "Analytical differentiation is not implemented yet" << std::endl;


#endif //  NumericDiff
}

// JacobianCompressed
JacobianCompressed::JacobianCompressed() : JacobianBase()
{
	this->jacobian1 = std::make_shared<Eigen::MatrixXd>();
	this->jacobian2 = std::make_shared<Eigen::MatrixXd>();
	this->jacobianSets = std::make_shared<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>();
}
JacobianCompressed::~JacobianCompressed()
{
}
void JacobianCompressed::initialize()
{
	if(isInitialized)
		return;

#ifndef NDEBUG
	assert(p != nullptr);
#endif // !NDEBUG

	if (informationMatrix->getInitialized() == false) {
		informationMatrix->setPropertise(p);
		if (this->timer)
			informationMatrix->setTimer(this->timer);
		informationMatrix->initialize();
	}

#ifndef NDEBUG
	assert((p->isRobust) == (kernel != nullptr));
#endif // !NDEBUG
	if(p->isRobust){
		if (kernel->getInitialized() == false) {
			kernel->setPropertise(p);
			if (this->timer)
				kernel->setTimer(this->timer);
			kernel->initialize();
		}


	}


	jacobian1->resize(p->edgeSize, p->vertex1Size * p->numEdges); // this is efficient becasue of the COL MAJOR storage of Eigen when taking blocks
	jacobian2->resize(p->edgeSize, p->vertex2Size * p->numEdges);

	jacobianSets->resize(p->numEdges);

	for (int edgeLocation = 0; edgeLocation < p->numEdges; edgeLocation++) {
		jacobianSets->at(edgeLocation).first = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(jacobian1->block( 0, edgeLocation * p->vertex1Size, p->edgeSize, p->vertex1Size));
		jacobianSets->at(edgeLocation).second = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(jacobian2->block(0, edgeLocation * p->vertex2Size, p->edgeSize, p->vertex2Size));
	}

	if (p->DebugMode) {
		jacobian1->setOnes();
		jacobian2->setOnes();
		
#ifndef NDEBUG
		assert((p->isRobust == (kernel != nullptr)) && informationMatrix != nullptr);
#endif // !NDEBUG
	}

	this->isInitialized = true;
}
void JacobianCompressed::finalize()
{
	if (isInitialized) {

		parameterSets->clear();
		parameterSets.reset();

		jacobianSets->clear();
		jacobianSets.reset();

		jacobian1.reset();
		jacobian2.reset();

		informationMatrix->finalize();
		informationMatrix.reset();
		if (true)
		{
			kernel->finalize();
		}
		kernel.reset();
		this->isInitialized = false;
	}

}
std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> JacobianCompressed::getJacobianVertex(bool isFirst, size_t edgeLocation)
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> j = isFirst ?
		jacobianSets->at(edgeLocation).first:
		jacobianSets->at(edgeLocation).second;
	return j;
}
void JacobianCompressed::updateJacobian()
{
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param1;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param2;

	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobianBlock1;
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobianBlock2;

	for(int edgeLocation = 0; edgeLocation < p->numEdges; edgeLocation++)
	{
		param1 = parameterSets->at(edgeLocation).first;
		param2 = parameterSets->at(edgeLocation).second;

		jacobianBlock1 = jacobianSets->at(edgeLocation).first;
		jacobianBlock2 = jacobianSets->at(edgeLocation).second;

		// Compute the Jacobian for the first vertex
		computeJacobianVertex(false, param1, param2, jacobianBlock1,1e-6);
		// Compute the Jacobian for the second vertex
		computeJacobianVertex(true, param1, param2, jacobianBlock2, 1e-6);
	}

	//apply Information matrix and robust kernel
	applyInformationSqrt();
	if (p->isRobust) {
		applyRobustKernel();
		kernel->setUpdated(false);
	}

	if (p->DebugMode && p->verbosityLevel > 3) {
		std::cout << "Jacobian1: " << "\n\n" << *jacobian1 << "\n\n\n" << std::endl;
		std::cout << "Jacobian2: " << "\n\n" << *jacobian2<< "\n\n\n" << std::endl;
	}
		

}
void JacobianCompressed::rowWiseMultiply(const Eigen::VectorXd& vec)
{
#ifndef NDEBUG
	assert(isInitialized);
	assert(vec.size() == p->totalObservations);
#endif // !NDEBUG
	
	for (int edgeLocation = 0; edgeLocation < p->numEdges; edgeLocation++)
	{
		// Access the blocks corresponding to the current edge
		auto& firstBlock = *(jacobianSets->at(edgeLocation).first);
		auto& secondBlock = *(jacobianSets->at(edgeLocation).second);
		// Assume each block has the same number of rows and the rows correspond to segments in the vector
		for (int row = 0; row < p->edgeSize; ++row) {
			// Multiply each element of the row by the corresponding vector element
			firstBlock.row(row) *= vec[edgeLocation * p->edgeSize + row];
			secondBlock.row(row) *= vec[edgeLocation * p->edgeSize + row];
		}
	}
}
void JacobianCompressed::applyRobustKernel()
{
#ifndef NDEBUG
	assert(isInitialized && kernel->isUpdated());
#endif // !NDEBUG
	rowWiseMultiply(*kernel->getWeightsVec());
}
void JacobianCompressed::applyInformationSqrt()
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	rowWiseMultiply(*informationMatrix->getInfVecSqrt());
}
void JacobianCompressed::insertParameterPair(int edgeLocation, std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>& paramSet)
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	//assert(paramSet.first.size() == p->vertex1Size && paramSet.second.size() == p->vertex2Size);
	parameterSets->at(edgeLocation) = paramSet;
}



//residualsBase
residualsBase::residualsBase() :kernel(nullptr),informationMatrix(nullptr), BaseDataStructure()
{
	this->isUpdated = false;
}
residualsBase::~residualsBase()
{
}

std::shared_ptr<InformationMatrixOrb> residualsBase::getInformationVector()
{
	return informationMatrix;
}

//residualsCompressed
residualsCompressed::residualsCompressed() : residualsBase()
{
	this->residuals = nullptr;
}
residualsCompressed::~residualsCompressed()
{
}
void residualsCompressed::initialize()
{
	
#ifndef NDEBUG
	assert(p != nullptr);
#endif // !NDEBUG
	if (informationMatrix->getInitialized() == false) {
		informationMatrix->setPropertise(p);
		//if (this->timer)
		//	informationMatrix->setTimer(this->timer);
		informationMatrix->initialize();
	}


	
#ifndef NDEBUG
	assert((p->isRobust) == (kernel != nullptr));
#endif // !NDEBUG
	if (p->isRobust) {
		if (kernel->getInitialized() == false) {
			kernel->setPropertise(p);
			//if (this->timer)
			//	kernel->setTimer(this->timer);
			kernel->initialize();
		}


	}


	residuals = std::make_shared<Eigen::VectorXd>(p->totalObservations);
	
	if(p->DebugMode){
		residuals->setOnes();
		
#ifndef NDEBUG
		assert((p->isRobust == (kernel != nullptr)) && informationMatrix != nullptr);
#endif // !NDEBUG
	}

	this->isInitialized = true;
}
void residualsCompressed::finalize()
{
	if (isInitialized) {
		residuals.reset();
		observationsVector.reset();
		parameterSets->clear();
		parameterSets.reset();

		informationMatrix->finalize();
		informationMatrix.reset();
		if (true)
		{
			kernel->finalize();
		}
		kernel.reset();
		this->isInitialized = false;
	}
}
void residualsCompressed::applyRobustKernel()
{
	
#ifndef NDEBUG
	assert(isInitialized && p->isRobust);
#endif // !NDEBUG
	kernel->robustifyResiduals(*residuals);
	kernel->setUpdated(true);
}
void residualsCompressed::applyInformationSqrt()
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	*this->residuals = (this->residuals->array()* this->informationMatrix->getInfVecSqrt()->array());
}
std::shared_ptr<Eigen::Map<Eigen::VectorXd>> residualsCompressed::getResiduals(int edgeLocation)
{
	return std::make_shared<Eigen::Map<Eigen::VectorXd>>(this->residuals->data() + edgeLocation * p->edgeSize, p->edgeSize);
}
void residualsCompressed::insertParameterPair(int edgeLocation, std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>& paramSet)
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG
	//assert(paramSet.first.size() == p->vertex1Size && paramSet.second.size() == p->vertex2Size);
	parameterSets->at(edgeLocation) = paramSet;
}
void residualsCompressed::updateResiduals()
{
#ifndef NDEBUG
	assert(isInitialized);
#endif // !NDEBUG

	if (p->DebugMode && p->verbosityLevel > 3)
	{
		std::cout << "Observations: " << observationsVector->transpose() << "\n\n\n" << std::endl;

	}

	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param1;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param2;
	Eigen::VectorXd y_est(p->edgeSize);
	for (int edgeLocation = 0; edgeLocation < p->numEdges; edgeLocation++)
	{
		param1 = parameterSets->at(edgeLocation).first;
		param2 = parameterSets->at(edgeLocation).second;
		reprojection->reproject(*param1, *param2, y_est);
		residuals->segment(edgeLocation * p->edgeSize, p->edgeSize) =  observationsVector->segment(edgeLocation * p->edgeSize, p->edgeSize) - y_est;
	}
	applyInformationSqrt();
	if (p->DebugMode && p->verbosityLevel > 3)
		informationMatrix->printInformationMatrix();
	if (p->isRobust) {
		applyRobustKernel();
			
	}

	if (p->DebugMode && p->verbosityLevel > 3)
		std::cout << "Residuals: " << residuals->transpose() << "\n\n\n" << std::endl;

	setIsUpdated(true);
}

double residualsCompressed::norm()
{
	return residuals->norm();
}

double residualsCompressed::getMaxCoeff()
{
	return residuals->cwiseAbs().maxCoeff();
}



//HessianBase
HessianBase::HessianBase() :  BaseDataStructure()
{
}
HessianBase::~HessianBase()
{
}

//HessianOrb
HessianOrb::HessianOrb() :HessianBase()
{
	this->A = std::make_shared<Eigen::MatrixXd>();
	this->B = std::make_shared<Eigen::MatrixXd>();
	this->W = std::make_shared<Eigen::MatrixXd>();
}
HessianOrb::~HessianOrb()
{

}
void HessianOrb::calculateInvB()
{
	//todo set a mechanism to donot update the invB if the B is not updated or if invB is already calculated
	for(int vertexLocation = 0; vertexLocation < p->numActiveVertices2; vertexLocation++)
	{
		*(*BBlocks)[vertexLocation] = (*BBlocks)[vertexLocation]->ldlt().solve(Eigen::MatrixXd::Identity(p->vertex2Size, p->vertex2Size));
	}
}
void HessianOrb::updateA()
{
	A->setZero();
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> ABlock = nullptr;
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobianBlock = nullptr;
	Eigen::MatrixXd tempBlock(p->vertex1Size, p->vertex1Size);
	for (int vertexLocation = 0; vertexLocation < p->numActiveVertices1; vertexLocation++)
	{
		getAblock(vertexLocation, ABlock);

		int numOfObservations = p->numOberservationsPerVertex1[vertexLocation];

		for (int edgeLocation = 0; edgeLocation < numOfObservations; edgeLocation++)
		{
			getAJacobianBlock(vertexLocation, edgeLocation, jacobianBlock);
			tempBlock.noalias() = jacobianBlock->transpose() * *jacobianBlock;
			*ABlock += tempBlock;
		}
	}
}
void HessianOrb::getAblock(int vertexLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& Ablock) {
	Ablock = (*Ablocks)[vertexLocation];
}
void HessianOrb::getWblock(int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& Wblock)
{
	Wblock = (*wBlocks)[edgeLocation];
}
std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> HessianOrb::getWblock(int edgeLocation)
{
	return (*wBlocks)[edgeLocation];
}
void HessianOrb::getAJacobianBlock(int vertexLocation, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& jacobianBlock) {
	jacobianBlock = (*jacobiansToABlocks)[vertexLocation][edgeLocation];
}
void HessianOrb::updateB()
{
	B->setZero();
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> BBlock = nullptr;
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobianBlock = nullptr;
	Eigen::MatrixXd tempBlock(p->vertex2Size, p->vertex2Size);
	for (int vertexLocation = 0; vertexLocation < p->numActiveVertices2; vertexLocation++)
	{
		getBBlock(vertexLocation, BBlock);

		int numOfObservations = p->numOberservationsPerVertex2[vertexLocation];

		for (int edgeLocation = 0; edgeLocation < numOfObservations; edgeLocation++)
		{
			getBJacobianBlock(vertexLocation, edgeLocation, jacobianBlock);
			tempBlock.noalias() = jacobianBlock->transpose() * *jacobianBlock;
			*BBlock += tempBlock;
		}
	}
}
void HessianOrb::getBBlock(int vertexLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& BBlock) {
	BBlock = BBlocks->at(vertexLocation);
}
void HessianOrb::getBJacobianBlock(int vertexLocation, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& jacobianBlock) {
	jacobianBlock = (*jacobiansToBBlocks)[vertexLocation][edgeLocation];
}
void HessianOrb::updateW()
{
	for (int edgeLocation = 0; edgeLocation < p->numEdges; edgeLocation++) {
		*(*wBlocks)[edgeLocation] = (*jacobiansToWBlocks)[edgeLocation].first->transpose() * *(*jacobiansToWBlocks)[edgeLocation].second;
	}
}
void HessianOrb::initialize()
{
	if (isInitialized)
		return;
	this->A->resize(p->vertex1Size, p->totalType1Parameters);
	this->B->resize(p->vertex2Size, p->totalType2Parameters);
	this->W->resize(p->vertex1Size, p->vertex2Size * p->numEdges);

	this->Ablocks = std::make_shared<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>();
	Ablocks->resize(p->numActiveVertices1);
	for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++) {
		(*Ablocks)[vertex1Location] = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(A->block(0, vertex1Location * p->vertex1Size, p->vertex1Size, p->vertex1Size));
	}

	this->BBlocks = std::make_shared<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>();
	BBlocks->resize(p->numActiveVertices2);
	for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++) {
		(*BBlocks)[vertex2Location] = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(B->block(0, vertex2Location * p->vertex2Size, p->vertex2Size, p->vertex2Size));
	}

	this->wBlocks = std::make_shared<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>();
	wBlocks->resize(p->numEdges);
	for (int edgeLocation = 0; edgeLocation < p->numEdges; edgeLocation++)
	{
		(*wBlocks)[edgeLocation] = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(W->block(0, static_cast<int>(edgeLocation) * p->vertex2Size, p->vertex1Size, p->vertex2Size));
	}

	this->isInitialized = true;

}
void HessianOrb::finalize()
{
	if (isInitialized) {
		Ablocks->clear();
		Ablocks.reset();
		BBlocks->clear();
		BBlocks.reset();
		wBlocks->clear();
		wBlocks.reset();

		jacobiansToABlocks->clear();
		jacobiansToABlocks.reset();
		jacobiansToBBlocks->clear();
		jacobiansToBBlocks.reset();
		jacobiansToWBlocks->clear();
		jacobiansToWBlocks.reset();

		A.reset();
		B.reset();
		W.reset();
		this->isInitialized = false;
	}
}
void HessianOrb::updateHessian()
{
	this->updateA();
	this->updateB();
	this->updateW();
	//if (p->DebugMode && p->verbosityLevel > 3)
	if (p->DebugMode && p->verbosityLevel > 3 )
	{
		std::cout << "A: " << "\n\n" << std::endl;
		for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++)
		{
			std::cout << "Vertex: "<<vertex1Location << "\n\n" << *(*Ablocks)[vertex1Location] << "\n\n" << std::endl;
		}

		std::cout << "B: " << "\n\n" << std::endl;
		for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
		{
			std::cout << "Vertex: " << vertex2Location << "\n\n" << *(*BBlocks)[vertex2Location] << "\n\n" << std::endl;
		}

		std::cout << "W: " << "\n\n" << std::endl;
		for (int edgeLocation = 0; edgeLocation < p->numEdges; edgeLocation++)
		{
			std::cout << "Edge: " << edgeLocation << "\n\n" << *(*wBlocks)[edgeLocation] << "\n\n" << std::endl;
		}
	}
}
double HessianOrb::getMaxDiagonalValue()
{
	double maxDiagonalValue = 0.0;
	for (int vertexLocation = 0; vertexLocation < p->numActiveVertices1; vertexLocation++)
	{
		std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> Ablock;
		getAblock(vertexLocation, Ablock);
		maxDiagonalValue = std::max(maxDiagonalValue, Ablock->diagonal().maxCoeff());
	}
	return maxDiagonalValue;
}
void HessianOrb::setJacobianVertex(int vertex1Location,int vertex2Location,int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian1Block, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian2Block)
{

	//std::cout << "jacobian1Blockrows: " << jacobian1Block->rows() << " jacobian1Blockcols: " << jacobian1Block->cols() << std::endl;
	//std::cout << "jacobian2Blockrows: " << jacobian2Block->rows() << " jacobian2Blockcols: " << jacobian2Block->cols() << std::endl;
#ifndef NDEBUG
	assert(jacobian1Block->rows() == p->edgeSize && jacobian1Block->cols() == p->vertex1Size);
	assert((*jacobiansToABlocks)[vertex1Location].size() < p->numOberservationsPerVertex1[vertex1Location]);
	
	assert(isInitialized);
	assert(jacobian2Block->rows() == p->edgeSize && jacobian2Block->cols() == p->vertex2Size);
	assert((*jacobiansToBBlocks)[vertex2Location].size() < p->numOberservationsPerVertex2[vertex2Location]);
	assert(edgeLocation < p->numEdges);
#endif // !NDEBUG
	
	(*jacobiansToABlocks)[vertex1Location].push_back(jacobian1Block);
	(*jacobiansToBBlocks)[vertex2Location].push_back(jacobian2Block);

	
	jacobiansToWBlocks->at(edgeLocation).first = jacobian1Block;
	jacobiansToWBlocks->at(edgeLocation).second = jacobian2Block;
}



//bVectorBase
bVectorBase::bVectorBase()
{
}
bVectorBase::~bVectorBase()
{
}

//bVectorOrb
bVectorOrb::bVectorOrb()
{
}
bVectorOrb::~bVectorOrb()
{
}
void bVectorOrb::initialize()
{
	if (isInitialized)
		return;
	
#ifndef NDEBUG
	assert(p != nullptr);
#endif // !NDEBUG
	this->b = std::make_shared<Eigen::VectorXd>(p->totalWidth);
	if (p->DebugMode) {
		b->setOnes();
	}

	type1BVec =std::make_shared<Eigen::Map<Eigen::VectorXd>>(b->data(), p->totalType1Parameters);
	type2BVec = std::make_shared<Eigen::Map<Eigen::VectorXd>>(b->data() + p->totalType1Parameters, p->totalType2Parameters);

	this->isInitialized = true;
}
void bVectorOrb::finalize()
{
	if (isInitialized) {
		residualsToType2->clear();
		residualsToType2.reset();
		residualsToType1->clear();
		residualsToType1.reset();
		jacobiansToType1->clear();
		jacobiansToType1.reset();
		jacobiansToType2->clear();
		jacobiansToType2.reset();


		type1BVec.reset();
		type2BVec.reset();
		b.reset();
		this->isInitialized = false;
	}
		
}
double bVectorOrb::getMaxCoeff()
{
	return b->cwiseAbs().maxCoeff();
}
std::shared_ptr<Eigen::Map<Eigen::VectorXd>> bVectorOrb::getBForParameter(bool isFirst, int vertexLocation)
{
	
#ifndef NDEBUG
	assert((isFirst && vertexLocation << p->numActiveVertices1) || (!isFirst && vertexLocation << p->numActiveVertices2));
#endif // !NDEBUG
	if(isFirst)
		return std::make_shared<Eigen::Map<Eigen::VectorXd>>(b->data() + vertexLocation * p->vertex1Size, p->vertex1Size);
	else
		return std::make_shared<Eigen::Map<Eigen::VectorXd>>(b->data() + p->totalType1Parameters + vertexLocation * p->vertex2Size, p->vertex2Size);
}
void bVectorOrb::setResidualEdge(int vertex1Location, int vertex2Location, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> residual)
{
	
#ifndef NDEBUG
	assert(vertex1Location < p->numActiveVertices1 && vertex2Location < p->numActiveVertices2);
#endif // !NDEBUG
	(*residualsToType1)[vertex1Location].push_back(residual);
	(*residualsToType2)[vertex2Location].push_back(residual);
}
void bVectorOrb::updateb()
{
	std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>> *jacobians = nullptr;
	std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>> *residuals = nullptr;
	this->b->setZero();

	for(int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++)
	{
		jacobians = &jacobiansToType1->at(vertex1Location);
		residuals = &residualsToType1->at(vertex1Location);
		for(int edgeLocation = 0; edgeLocation < p->numOberservationsPerVertex1[vertex1Location]; edgeLocation++)
		{
			b->segment(vertex1Location * p->vertex1Size, p->vertex1Size) += jacobians->at(edgeLocation)->transpose() * *residuals->at(edgeLocation);
		}
	}

	for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
	{
		jacobians = &jacobiansToType2->at(vertex2Location);
		residuals = &residualsToType2->at(vertex2Location);
		for (int edgeLocation = 0; edgeLocation < p->numOberservationsPerVertex2[vertex2Location]; edgeLocation++)
		{
			b->segment(p->totalType1Parameters + vertex2Location * p->vertex2Size, p->vertex2Size) += jacobians->at(edgeLocation)->transpose() * *residuals->at(edgeLocation);
		}
	}
	//if (p->DebugMode && p->verbosityLevel > 3)
	if (p->DebugMode && p->verbosityLevel > 3)
				std::cout << "b: \n\n" << b->transpose() << "\n\n\n" << std::endl;
}



//SolverBase
SolverBase::SolverBase()
{
}
SolverBase::~SolverBase()
{
}

//SolverOrb
SolverOrb::SolverOrb()
{
	this->Ablocks = nullptr;
	this->BBlocks = nullptr;
}
SolverOrb::~SolverOrb(){}

//SolverMarginalized
SolverMarginalized::SolverMarginalized()
{
}
SolverMarginalized::~SolverMarginalized()
{
}
void SolverMarginalized::initialize()
{
	if (isInitialized)
		return;
	
#ifndef NDEBUG
	assert(p != nullptr);
#endif // !NDEBUG
	this->BInv = std::make_shared<Eigen::MatrixXd>(p->vertex2Size, p->totalType2Parameters);
	this->ActiveHessian = std::make_shared<Eigen::MatrixXd>(p->totalType1Parameters,p->totalType1Parameters);
	this->YBlocksMatrix = std::make_shared<Eigen::MatrixXd>(p->vertex1Size, p->vertex2Size * p->numEdges);
	this->activeB = std::make_shared<Eigen::VectorXd>(p->totalType1Parameters);

	
	this->BInvBlocks = std::make_shared<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>();
	BInvBlocks->resize(p->numActiveVertices2);
	for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++) {
		(*BInvBlocks)[vertex2Location] = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(BInv->block(0, vertex2Location * p->vertex2Size, p->vertex2Size, p->vertex2Size));
	}
	
	this->Yblocks = std::make_shared<std::vector<std::map<int,std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>(p->numActiveVertices1);
	this->HessianBlocksToYBlocks = std::make_shared<std::vector<std::map<int,std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>>(p->numActiveVertices1);

	this->type2BVecstoActiveB = std::make_shared<std::vector<std::map<int, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>>(p->numActiveVertices1);
	
	this->w_delta_toMarginalizedB = std::make_shared<std::vector<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>>>(p->numActiveVertices2);
	for(int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
		w_delta_toMarginalizedB->at(vertex2Location).reserve(p->numOberservationsPerVertex2[vertex2Location]);

	this->ActiveHessianBlocks = std::make_shared< std::vector < std::vector< std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>(p->numActiveVertices1);
	for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++)
		ActiveHessianBlocks->at(vertex1Location).resize(p->numActiveVertices1);

	this->YWBlockstoHessianBlocks = std::make_shared<std::vector<std::vector< std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>>>(p->numActiveVertices1);
	for (int vertexLocation = 0; vertexLocation < p->numActiveVertices1; vertexLocation++)
	{
		YWBlockstoHessianBlocks->at(vertexLocation).resize(p->numActiveVertices1);
	}
	
	for(int rowLocation = 0; rowLocation < p->numActiveVertices1; rowLocation++)
	{
		for (int columnLocation = 0; columnLocation < p->numActiveVertices1; columnLocation++)
		{
			ActiveHessianBlocks->at(rowLocation)[columnLocation] = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(*ActiveHessian, rowLocation * p->vertex1Size, columnLocation * p->vertex1Size, p->vertex1Size, p->vertex1Size);
		}
	}
	this->isInitialized = true;
}
void SolverMarginalized::finalize()
{
	w_delta_toMarginalizedB->clear();
	w_delta_toMarginalizedB.reset();

	YWBlockstoHessianBlocks->clear();
	YWBlockstoHessianBlocks.reset();

	ActiveHessianBlocks->clear();
	ActiveHessianBlocks.reset();

	type2BVecstoActiveB->clear();
	type2BVecstoActiveB.reset();

	Yblocks->clear();
	Yblocks.reset();

	HessianBlocksToYBlocks->clear();
	HessianBlocksToYBlocks.reset();

	BInvBlocks->clear();
	BInvBlocks.reset();

	type2UpdateVec.reset();
	type1UpdateVec.reset();
	updateVector.reset();

	type2BVec.reset();
	type1BVec.reset();
	bVector.reset();

	activeB.reset();
	YBlocksMatrix.reset();
	ActiveHessian.reset();
	BInv.reset();

	this->isInitialized = false;
}
void SolverMarginalized::setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector)
{
	this->updateVector = updateVector;
	this->type1UpdateVec = std::make_shared<Eigen::Map<Eigen::VectorXd>>(updateVector->data(), p->totalType1Parameters);
	this->type2UpdateVec = std::make_shared<Eigen::Map<Eigen::VectorXd>>(updateVector->data() + p->totalType1Parameters, p->totalType2Parameters);
}
void SolverMarginalized::setBvector(std::shared_ptr<Eigen::VectorXd> bVector)
{
	this->bVector = bVector;
	this->type1BVec = std::make_shared<Eigen::Map<Eigen::VectorXd>>(bVector->data(), p->totalType1Parameters);
	this->type2BVec = std::make_shared<Eigen::Map<Eigen::VectorXd>>(bVector->data() + p->totalType1Parameters, p->totalType2Parameters);
}
double SolverMarginalized::getUpdateNorm()
{
	return this->updateVector->norm();
}
double SolverMarginalized::dotProduct(std::string firstVec, std::string secondVec)
{
	if (firstVec == "bVec" && secondVec == "updateVec") {
		return bVector->dot(*updateVector);
	}
	else {
		std::cerr << "Invalid input for dot product" << std::endl;
		return 0;
	}
}
void SolverMarginalized::setEdge(int vertex1Location, int vertex2Location, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock)
{
#ifndef NDEBUG
	assert(vertex1Location < p->numActiveVertices1 && vertex2Location < p->numActiveVertices2);
#endif // !NDEBUG
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> YBlock = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(YBlocksMatrix->block(0, edgeLocation * p->vertex2Size, p->vertex1Size, p->vertex2Size));
	
	(*Yblocks)[vertex1Location].insert(std::make_pair(vertex2Location,YBlock));
	(*HessianBlocksToYBlocks)[vertex1Location].insert(std::make_pair(vertex2Location, std::make_pair(WBlock, BInvBlocks->at(vertex2Location))));

	(*type2BVecstoActiveB)[vertex1Location].insert(std::make_pair(vertex2Location, std::make_shared<Eigen::Map<Eigen::VectorXd>>(type2BVec->data() + vertex2Location * p->vertex2Size, p->vertex2Size)));

	(*w_delta_toMarginalizedB)[vertex2Location].push_back(std::make_pair(WBlock, std::make_shared<Eigen::Map<Eigen::VectorXd>>(type1UpdateVec->data() + vertex1Location * p->vertex1Size, p->vertex1Size)));

}
bool SolverMarginalized::solve()
{
	applyLamda();
	if(p->DebugMode && p->verbosityLevel > 3){
		//print A blocks and B blocks
		std::cout << "Ablocks: " << "\n\n";
		for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++)
		{
			std::cout << "Vertex:\n\n" << vertex1Location <<"\n\n" << *Ablocks->at(vertex1Location) << "\n\n";
		}

		std::cout << "\n\n\nBblocks: " << "\n\n";
		for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
		{
			std::cout << "Vertex:" << vertex2Location << "\n\n" << *BBlocks->at(vertex2Location) << "\n\n";
		}
		std::cout << "\n\n\n";
	}
	if(!updateInvB()) return false;
	if(p->DebugMode && p->verbosityLevel > 3){
		//print BInv blocks
		std::cout << "BInvBlocks: " << "\n\n";
		for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
		{
			std::cout << "Vertex:" << vertex2Location << "\n\n" << *BInvBlocks->at(vertex2Location) << "\n\n";
		}
	}

	updateYBlocks();
	updateActiveHessianV2();
	updateActiveB();
	if(!solveForType1()) return false;
	solveForType2();

	 return true;
}

void SolverMarginalized::applyLamda()
{
	double lamda = p->Lamda;
	if(p->isRepeatAttempt)
		lamda -= previousLamda;

	previousLamda = lamda;

	for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++)
	{
		Ablocks->at(vertex1Location)->diagonal().array() += lamda;
	}

	for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
	{
		BBlocks->at(vertex2Location)->diagonal().array() += lamda;
	}
}
bool SolverMarginalized::updateInvB()
{
	for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
	{
		Eigen::LDLT<Eigen::MatrixXd> ldlt = BBlocks->at(vertex2Location)->ldlt();

		if (ldlt.info() != Eigen::Success) {
			std::cerr << "LDLT decomposition failed at vertex location " << vertex2Location << std::endl;
			return false; 
		}

		*BInvBlocks->at(vertex2Location) = ldlt.solve(Eigen::MatrixXd::Identity(p->vertex2Size, p->vertex2Size));

		if (ldlt.info() != Eigen::Success) {
			std::cerr << "Solving for inverse failed at vertex location " << vertex2Location << std::endl;
			return false; 
		}
	}
	return true;
}
void SolverMarginalized::updateYBlocks()
{
	//YBlocksMatrix->setZero();
	for (int vertex1Location = 0; vertex1Location < this->p->numActiveVertices1; vertex1Location++)
	{
		
		std::map<int, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>& YBlocksForVertex = (*this->Yblocks)[vertex1Location];
		std::map<int, std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>& HessianBlocksToYBlocksForVertex =
			(*this->HessianBlocksToYBlocks)[vertex1Location];

		for (auto pair : YBlocksForVertex) {
#ifndef NDEBUG
			assert(HessianBlocksToYBlocksForVertex.count(pair.first));
#endif // !NDEBUG
			std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& YBlock = pair.second;
			YBlock->noalias() = *HessianBlocksToYBlocksForVertex[pair.first].first * *HessianBlocksToYBlocksForVertex[pair.first].second;
		
			if (p->DebugMode && p-> verbosityLevel > 3) {
				std::cout << "YBlock: V1: " << vertex1Location << " V2: " << pair.first << "\n\n";
				
				std::cout << "Wblock:\n\n" << *HessianBlocksToYBlocksForVertex[pair.first].first << "\n\n";
				
				std::cout << "BInvblock:\n\n" << *HessianBlocksToYBlocksForVertex[pair.first].second << "\n\n";
				
				std::cout<<"Final Y Block:\n\n" << *YBlock << "\n\n";
		}

		
		}
	}


}
void SolverMarginalized::updateActiveHessian()
{
	ActiveHessian->setZero();
	//adding information came from marginalization to the final hessian : this can be more optimized if we store the data on a map
	for (int rowLocation = 0; rowLocation < p->numActiveVertices1; rowLocation++)
	{

		for (int columnLocation = 0; columnLocation < p->numActiveVertices1; columnLocation++)
		{
			for (auto& pair : Yblocks->at(rowLocation))
			{
				if (HessianBlocksToYBlocks->at(columnLocation).count(pair.first)) {
					*ActiveHessianBlocks->at(rowLocation)[columnLocation] -= *pair.second * HessianBlocksToYBlocks->at(columnLocation)[pair.first].first->transpose();
				}
				else {
					continue;
				}
			}


		}
	}
	//print active hessian before adding Ablocks
	if(p->DebugMode && p->verbosityLevel > 3)
		std::cout << "ActiveHessian before adding Ablocks: -(temp2)" << "\n\n" << *ActiveHessian << "\n\n\n" << std::endl;

	//adding Ablocks to the final hessian's diagonals
	for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++)
	{
		*ActiveHessianBlocks->at(vertex1Location)[vertex1Location] += *Ablocks->at(vertex1Location);
	}

	//print active hessian after adding Ablocks
	if (p->DebugMode && p->verbosityLevel > 3)
		std::cout << "ActiveHessian after adding Ablocks: -(temp2)" << "\n\n" << *ActiveHessian << "\n\n\n" << std::endl;


}
void SolverMarginalized::updateActiveHessianV2()
{
	ActiveHessian->setZero();
	//adding information came from marginalization to the final hessian : this can be more optimized if we store the data on a map
	if(!this->activeHessianSetted)
		setActiveStructure();
	else
		updateActiveStructure();
	//adding Ablocks to the final hessian's diagonals
	for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++)
	{
		*ActiveHessianBlocks->at(vertex1Location)[vertex1Location] += *Ablocks->at(vertex1Location);
	}
}
void SolverMarginalized::updateActiveStructure()
{
	for (int rowLocation = 0; rowLocation < p->numActiveVertices1; rowLocation++)
	{

		for (int columnLocation = 0; columnLocation < p->numActiveVertices1; columnLocation++)
		{
			for (auto& pair : YWBlockstoHessianBlocks->at(rowLocation)[columnLocation])
			{
				*ActiveHessianBlocks->at(rowLocation)[columnLocation] -= *pair.first * pair.second->transpose();
			}
		}
	}
	
}
void SolverMarginalized::setActiveStructure()
{
	for (int rowLocation = 0; rowLocation < p->numActiveVertices1; rowLocation++)
	{

		for (int columnLocation = 0; columnLocation < p->numActiveVertices1; columnLocation++)
		{
			for (auto& pair : Yblocks->at(rowLocation))
			{	
				//YWBlockstoHessianBlocks->at(rowLocation)[columnLocation].clear();
				if (HessianBlocksToYBlocks->at(columnLocation).count(pair.first)) {
					*ActiveHessianBlocks->at(rowLocation)[columnLocation] -= *pair.second * HessianBlocksToYBlocks->at(columnLocation)[pair.first].first->transpose();
					YWBlockstoHessianBlocks->at(rowLocation)[columnLocation].push_back(std::make_pair(pair.second, HessianBlocksToYBlocks->at(columnLocation)[pair.first].first));
				}
				else {
					continue;
				}
			}
		}
	}
	this->activeHessianSetted = true;
}
void SolverMarginalized::updateActiveB()
{
	*activeB = *type1BVec;
	for (int vertex1Location = 0; vertex1Location < p->numActiveVertices1; vertex1Location++) {
		for (auto& YBlock : Yblocks->at(vertex1Location)) {
			activeB->segment(vertex1Location * p->vertex1Size, p->vertex1Size) -= *YBlock.second * *type2BVecstoActiveB->at(vertex1Location)[YBlock.first];
		}
	}

	if (p->DebugMode && p->verbosityLevel > 3)
		std::cout << "ActiveB: " << "\n\n" << activeB->transpose() << "\n\n\n" << std::endl;
}
bool SolverMarginalized::solveForType1() {

	Eigen::LDLT<Eigen::MatrixXd> ldlt = ActiveHessian->ldlt();

	if (ldlt.info() != Eigen::Success) {
		std::cout << "LDLT decomposition failed" << std::endl;
		return false;
	}

	*type1UpdateVec = ldlt.solve(*activeB);

	if(p->DebugMode  && p->verbosityLevel > 3)
		std::cout << "type1UpdateVec: " << "\n\n" << type1UpdateVec->transpose() << "\n\n\n" << std::endl;

	if (ldlt.info() != Eigen::Success) {
		std::cout << "Solving for type1UpdateVec failed" << std::endl;
		return false;
	}

	return true;
}
void SolverMarginalized::solveForType2()
{
	*type2UpdateVec = *type2BVec;

	for (int vertex2Location = 0; vertex2Location < p->numActiveVertices2; vertex2Location++)
	{
		for (auto& pair : w_delta_toMarginalizedB->at(vertex2Location))
		{
			type2UpdateVec->segment(vertex2Location * p->vertex2Size, p->vertex2Size) -= pair.first->transpose() * *pair.second;
		}

		type2UpdateVec->segment(static_cast<int>(vertex2Location) * p->vertex2Size, p->vertex2Size) = *BInvBlocks->at(vertex2Location) * type2UpdateVec->segment(static_cast<int>(vertex2Location) * p->vertex2Size, p->vertex2Size);
	}

	if(p->DebugMode && p->verbosityLevel > 3)
		std::cout << "type2UpdateVec: " << "\n\n" << type2UpdateVec->transpose() << "\n\n\n" << std::endl;
}
 

