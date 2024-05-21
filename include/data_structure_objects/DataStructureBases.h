// InformationMatrixBase.h
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "RobustKernels.h"
#include "ParameterUpdateBase.h"
#include "ReprojectionBase.h"
#include "Misc.h"
#include <iostream>
#include "Timer.h"

	/**
	 * @class InformationMatrixBase
	 * @brief Base class for handling information matrices in SLAM.
	 */
class InformationMatrixBase : public BaseDataStructure {
protected:

public:
	InformationMatrixBase() : BaseDataStructure() {}
	~InformationMatrixBase() {}

	/**
	 * @brief Retrieves the square root of the information value at a given index.
	 * @param index Index of the information value.
	 * @return Square root of the information value.
	 */
	virtual double getInfSqrt(int index) = 0;

	/**
	 * @brief Retrieves the information value at a given index.
	 * @param index Index of the information value.
	 * @return Information value.
	 */
	virtual double getInf(int index) = 0;
};

// InformationMatrixOrb.h
class InformationMatrixOrb : public InformationMatrixBase {
protected:

public:
	InformationMatrixOrb();
	~InformationMatrixOrb();

	/**
	 * @brief Updates the information vector based on the provided vector at a specific edge location.
	 * @param edgeLocation Location of the edge.
	 * @param vec Vector containing new information values.
	 */
	virtual void updateInfVec(int edgeLocation, const Eigen::VectorXd& vec) = 0;

	/**
	 * @brief Retrieves the information vector.
	 * @return Shared pointer to the information vector.
	 */
	virtual std::shared_ptr<Eigen::VectorXd> getInfVec() = 0;

	/**
	 * @brief Retrieves the square root of the information vector.
	 * @return Shared pointer to the square root of the information vector.
	 */
	virtual std::shared_ptr<Eigen::VectorXd> getInfVecSqrt() = 0;

	/**
	 * @brief Sets the information vector.
	 * @param infVec Vector containing information values.
	 */
	virtual void setInformationVector(const Eigen::VectorXd& infVec) = 0;

	/**
	 * @brief Sets the square root of the information vector.
	 * @param infVec Vector containing square roots of information values.
	 */
	virtual void setInformationVectorSqrt(const Eigen::VectorXd& infVec) = 0;

	/**
	 * @brief Prints the information matrix.
	 */
	virtual void printInformationMatrix() = 0;
};

// InformationMatrixCompressed.h
class InformationMatrixCompressed : public InformationMatrixOrb
{
protected:
	std::shared_ptr<Eigen::VectorXd> infVec;
	std::shared_ptr<Eigen::VectorXd> infVec_sqrt;

public:
	InformationMatrixCompressed();
	~InformationMatrixCompressed();

	void initialize() override;
	void finalize() override;
	void updateInfVec(int edgeLocation, const Eigen::VectorXd& vec) override;
	std::shared_ptr<Eigen::VectorXd> getInfVec() override { return infVec; }
	std::shared_ptr<Eigen::VectorXd> getInfVecSqrt() override { return infVec_sqrt; }
	double getInfSqrt(int index) override;
	double getInf(int index) override;
	void setInformationVector(const Eigen::VectorXd& infVec) override { this->infVec = std::make_shared<Eigen::VectorXd>(infVec); }
	void setInformationVectorSqrt(const Eigen::VectorXd& infVec) override { this->infVec = std::make_shared<Eigen::VectorXd>(infVec); }
	void printInformationMatrix() override;

};

	/**
	 * @class ParameterVectorBase
	 * @brief Base class for handling Parameters.
	 */
	// ParameterVectorBase.h
class ParameterVectorBase : public BaseDataStructure {
protected:
	bool isBackedUp;
	double maxCoeffValue;

	std::unique_ptr<ParameterUpdateBase> parameterUpdate;

public:
	ParameterVectorBase();
	~ParameterVectorBase();

	/**
	 * @brief Sets the parameter update strategy.
	 * @param parameterUpdate Unique pointer to the parameter update base object.
	 */
	virtual void setParameterUpdate(std::unique_ptr<ParameterUpdateBase> parameterUpdate) { this->parameterUpdate = std::move(parameterUpdate); }
	
	/**
	 * @brief Sets the update vector.
	 * @param updateVector Shared pointer to the update vector.
	 */
	virtual void setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector) = 0;

	/**
	 * @brief Sets the parameter vector.
	 * @param parameterVector Parameter vector to set.
	 */
	virtual void setParameterVector(const Eigen::VectorXd& parameterVector) = 0;

	/**
	 * @brief Retrieves the parameter vector map.
	 * @param isFirst True if it is the first parameter set, false otherwise.
	 * @param location Location index in the parameter vector.
	 * @return Shared pointer to the parameter vector map.
	 */
	virtual std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getParameterVectorMap(bool isFirst, int location) = 0;

	/**
	 * @brief Retrieves the update vector.
	 * @return Shared pointer to the update vector.
	 */
	virtual std::shared_ptr<Eigen::VectorXd> getUpdateVector() = 0;

	/**
	 * @brief Updates the parameter vector with the current update vector.
	 */
	virtual void updateParameterVector() = 0;

	/**
	 * @brief Prints the parameter vector.
	 */
	virtual void printParameterVector() = 0;

	/**
	 * @brief Backs up the current parameter vector.
	 */
	virtual void backUp() = 0;

	/**
	 * @brief Restores the parameter vector from the backup.
	 */
	virtual void restore() = 0;

	/**
	 * @brief Retrieves the maximum coefficient value from the parameter vector.
	 * @return Maximum coefficient value.
	 */
	virtual double maxCoeff() = 0;
};

class parameterVectorMarginalized : public ParameterVectorBase
{
protected:
	std::shared_ptr<Eigen::VectorXd> activeParameterVector;
	std::shared_ptr<Eigen::VectorXd> marginalizedParameterVector;

	std::shared_ptr<Eigen::VectorXd> activeParameterVectorPrevious;
	std::shared_ptr<Eigen::VectorXd> marginalizedParameterVectorPrevious;

	std::shared_ptr<Eigen::VectorXd> updateVector;

	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> activeUpdateParameterVectorMap;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> marginalizedUpdateParameterVectorMap;

public:


	parameterVectorMarginalized();
	~parameterVectorMarginalized();

	void initialize() override;
	void finalize() override;

	void setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector) override;
	void setParameterVector(const Eigen::VectorXd& parameterVector) override; //not implemented
	void setActiveParameterVector(const Eigen::VectorXd& activeParameterVector); //not implemented
	void setMarginalizedParameterVector(const Eigen::VectorXd& marginalizedParameterVector); //not implemented

	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getParameterVectorMap(bool isFirst, int location) override;

	std::shared_ptr<Eigen::VectorXd> getActiveParameterVector();
	std::shared_ptr<Eigen::VectorXd> getMarginalizedParameterVector();
	
	std::shared_ptr<Eigen::VectorXd> getUpdateVector() { return updateVector; }

	void updateParameterVector() override;

	void printParameterVector() override;

	void backUp() override;
	void restore() override;
	double maxCoeff() override;
};



// JacobianBase.h
class JacobianBase : public BaseDataStructure {
protected:
	bool isUpdated;

	std::shared_ptr<RobustKernelBase> kernel;
	std::shared_ptr<InformationMatrixOrb> informationMatrix;
	std::shared_ptr<ReprojectionBase> reprojection;

	/**
	 * @brief Computes the Jacobian for a vertex.
	 * @param reverse If true, computes in reverse order.
	 * @param param1 First set of parameters.
	 * @param param2 Second set of parameters.
	 * @param jacobian Reference to the Jacobian block to update.
	 * @param stepSize Step size used for numerical differentiation.
	 */
	virtual void computeJacobianVertex(bool reverse, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param1, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param2, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian, double stepSize) = 0;

public:
	JacobianBase();
	~JacobianBase();
	
	/**
	 * @brief Sets the robust kernel for Jacobian computation.
	 * @param kernel Shared pointer to the robust kernel.
	 */
	virtual void setRobustKernel(std::shared_ptr<RobustKernelBase> kernel) { this->kernel = kernel; }
	
	/**
	 * @brief Sets the information vector for Jacobian computation.
	 * @param informationMatrix Shared pointer to the information matrix.
	 */
	virtual void setInformationVector(std::shared_ptr<InformationMatrixOrb> informationMatrix) { this->informationMatrix = informationMatrix; }


	/**
	 * @brief Gets the information vector used in Jacobian computation.
	 * @return Shared pointer to the information matrix.
	 */
	virtual std::shared_ptr<InformationMatrixOrb> getInformationVector() { return informationMatrix; }

	/**
	 * @brief Sets the reprojection strategy for Jacobian computation.
	 * @param reprojection Shared pointer to the reprojection base object.
	 */
	virtual void setReprojection(std::shared_ptr<ReprojectionBase> reprojection) { this->reprojection = reprojection; }


	/**
		 * @brief Applies the robust kernel to the Jacobian.
		 */
	virtual void applyRobustKernel() = 0;

	/**
	 * @brief Updates the Jacobian based on the current parameters.
	 */
	virtual void updateJacobian() = 0;

	/**
	 * @brief Gets the updated status of the Jacobian.
	 * @return True if updated, false otherwise.
	 */
	bool getIsUpdated() { return isUpdated; }

	/**
	 * @brief Sets the updated status of the Jacobian.
	 * @param isUpdated Updated status to set.
	 */
	void setIsUpdated(bool isUpdated) { this->isUpdated = isUpdated; }

};


// JacobianCompressed.h
class JacobianCompressed : public JacobianBase {
protected:
	// Memory locations of the parameters to calculate the Jacobian
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets; // [0:Vertex1, 1:Vertex2]

	// Memory locations of the Jacobians
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobianSets; // [0:Vertex1, 1:Vertex2]

	std::shared_ptr<Eigen::MatrixXd> jacobian1;
	std::shared_ptr<Eigen::MatrixXd> jacobian2;

	/**
	 * @brief Computes the Jacobian for a vertex.
	 * @param reverse If true, computes in reverse order.
	 * @param param1 First set of parameters.
	 * @param param2 Second set of parameters.
	 * @param jacobian Reference to the Jacobian block to update.
	 * @param stepSize Step size used for numerical differentiation.
	 */
	virtual void computeJacobianVertex(bool reverse, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param1, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param2, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian, double stepSize) override;

public:
	JacobianCompressed();
	~JacobianCompressed();

	void initialize() override;
	void finalize() override;

	/**
	 * @brief Retrieves the Jacobian block for a specified vertex.
	 * @param isFirst If true, retrieves the first vertex's Jacobian; otherwise, the second.
	 * @param edgeLocation Edge location index.
	 * @return Shared pointer to the Jacobian block.
	 */
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> getJacobianVertex(bool isFirst, size_t edgeLocation);

	void updateJacobian() override;

	/**
	 * @brief Multiplies each row of the Jacobian by a corresponding vector element.
	 * @param vec Vector to multiply.
	 */
	void rowWiseMultiply(const Eigen::VectorXd& vec);

	/**
	 * @brief Applies the robust kernel to the Jacobian.
	 */
	void applyRobustKernel() override;

	/**
	 * @brief Applies the square root of the information matrix to the Jacobian.
	 */
	virtual void applyInformationSqrt();

	/**
	 * @brief Sets the parameter sets used in Jacobian computation.
	 * @param parameterSets Shared pointer to the vector of parameter sets.
	 */
	void setParameterSets(std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets) { this->parameterSets = parameterSets; }

	/**
	 * @brief Gets the parameter sets used in Jacobian computation.
	 * @return Shared pointer to the vector of parameter sets.
	 */
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> getParameterSets() { return parameterSets; }

	/**
	 * @brief Inserts a parameter pair for a specific edge.
	 * @param edgeLocation Location of the edge.
	 * @param paramSet Pair of parameter maps for the edge.
	 */
	void insertParameterPair( int edgeLocation , std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>& paramSet);
};



// ResidualsBase.h
class ResidualsBase : public BaseDataStructure {
protected:
	bool isUpdated;

	std::shared_ptr<RobustKernelBase> kernel;
	std::shared_ptr<InformationMatrixOrb> informationMatrix;
	std::shared_ptr<ReprojectionBase> reprojection;

public:
	ResidualsBase();
	~ResidualsBase();

	/**
	 * @brief Sets the robust kernel for residuals computation.
	 * @param kernel Shared pointer to the robust kernel.
	 */
	virtual void setRobustKernel(std::shared_ptr<RobustKernelBase> kernel) { this->kernel = kernel; }



	/**
	 * @brief Sets the information vector for residuals computation.
	 * @param informationMatrix Shared pointer to the information matrix.
	 */
	virtual void setInformationVector(std::shared_ptr<InformationMatrixOrb> informationMatrix) { this->informationMatrix = informationMatrix; }

	/**
	 * @brief Gets the information vector used in residuals computation.
	 * @return Shared pointer to the information matrix.
	 */
	virtual std::shared_ptr<InformationMatrixOrb> getInformationVector();

	/**
	 * @brief Sets the reprojection strategy for residuals computation.
	 * @param reprojection Shared pointer to the reprojection base object.
	 */
	virtual void setReprojection(std::shared_ptr<ReprojectionBase> reprojection) { this->reprojection = reprojection; }

	/**
	 * @brief Applies the robust kernel to the residuals.
	 */
	virtual void applyRobustKernel() = 0;

	/**
	 * @brief Gets the updated status of the residuals.
	 * @return True if updated, false otherwise.
	 */
	bool getIsUpdated() { return isUpdated; }


	/**
	 * @brief Sets the updated status of the residuals.
	 * @param isUpdated Updated status to set.
	 */
	void setIsUpdated(bool isUpdated) { this->isUpdated = isUpdated; }

	/**
	 * @brief Updates the residuals based on the current parameters and observations.
	 */
	virtual void updateResiduals() = 0;

	/**
	 * @brief Calculates the norm of the residuals.
	 * @return Norm of the residuals.
	 */
	virtual double norm() = 0;

	/**
	 * @brief Retrieves the maximum absolute coefficient from the residuals.
	 * @return Maximum absolute coefficient.
	 */
	virtual double getMaxCoeff() = 0;
};

// ResidualsCompressed.h
class ResidualsCompressed : public ResidualsBase {
protected:
	std::shared_ptr<Eigen::VectorXd> residuals;
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets; // [0:Vertex1, 1:Vertex2]
	std::shared_ptr<const Eigen::VectorXd> observationsVector;

public:
	ResidualsCompressed();
	~ResidualsCompressed();

	void initialize() override;
	void finalize() override;

	/**
	 * @brief Applies the robust kernel to the residuals.
	 */
	void applyRobustKernel() override;

	/**
	 * @brief Applies the square root of the information matrix to the residuals.
	 */
	virtual void applyInformationSqrt();

	/**
	 * @brief Retrieves the residuals for a specific edge.
	 * @param edgeLocation Edge location index.
	 * @return Shared pointer to the map of residuals for the edge.
	 */
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getResiduals(int edgeLocation);

	/**
	 * @brief Sets the observations vector used in residuals computation.
	 * @param observations Shared pointer to the constant vector of observations.
	 */
	void setObservationsVector(std::shared_ptr<const Eigen::VectorXd> observations) { this->observationsVector = observations; }

	/**
	 * @brief Sets the parameter sets used in residuals computation.
	 * @param parameterSets Shared pointer to the vector of parameter sets.
	 */
	void setParameterSets(std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets) { this->parameterSets = parameterSets; }

	/**
	 * @brief Gets the parameter sets used in residuals computation.
	 * @return Shared pointer to the vector of parameter sets.
	 */
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> getParameterSets() { return parameterSets; }

	/**
	 * @brief Inserts a parameter pair for a specific edge.
	 * @param edgeLocation Location of the edge.
	 * @param paramSet Pair of parameter maps for the edge.
	 */
	void insertParameterPair(int edgeLocation, std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>& paramSet);

	void updateResiduals() override;
	double norm() override;
	double getMaxCoeff() override;
};




// HessianBase.h
class HessianBase : public BaseDataStructure {
protected:

public:
    HessianBase();
    ~HessianBase();

    /**
     * @brief Updates the Hessian matrix based on the current Jacobians and residuals.
     */
    virtual void updateHessian() = 0;

    /**
     * @brief Retrieves the maximum value from the diagonal of the Hessian matrix.
     * @return Maximum diagonal value.
     */
    virtual double getMaxDiagonalValue() = 0;
};


/**
 * @class HessianOrb
 * @brief Handles the construction and management of the Hessian matrix for ORB-SLAM.
 */
class HessianOrb : public HessianBase
{
protected:
	// TODO: Add updated flags for A, B, W and check them before proceeding to the next stage.

	/// Shared pointers to the matrices storing the values for the Hessian.
	std::shared_ptr<Eigen::MatrixXd> A;
	std::shared_ptr<Eigen::MatrixXd> B;
	std::shared_ptr<Eigen::MatrixXd> W;

	/// Memory locations of the Jacobians used to calculate different blocks of the Hessian.
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToABlocks;  ///< [Vertex][Edge]
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToBBlocks;  ///< [Vertex][Edge]
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToWBlocks;  ///< [Edge][0:Vertex1,1:Vertex2]

	/// Memory locations of different types of Hessian blocks.
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> Ablocks;
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BBlocks;
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> wBlocks;  ///< [EdgeId]

	/**
	 * @brief Updates the A matrix of the Hessian.
	 */
	virtual void updateA();

	/**
	 * @brief Updates the B matrix of the Hessian.
	 */
	virtual void updateB();

	/**
	 * @brief Updates the W matrix of the Hessian.
	 */
	virtual void updateW();

	/**
	 * @brief Retrieves the Jacobian block for matrix A.
	 * @param vertexLocation Location of the vertex.
	 * @param edgeLocation Location of the edge.
	 * @param jacobianBlock Shared pointer to the Jacobian block.
	 */
	void getAJacobianBlock(int vertexLocation, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& jacobianBlock);

	/**
	 * @brief Retrieves the Jacobian block for matrix B.
	 * @param vertexLocation Location of the vertex.
	 * @param edgeLocation Location of the edge.
	 * @param jacobianBlock Shared pointer to the Jacobian block.
	 */
	void getBJacobianBlock(int vertexLocation, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& jacobianBlock);

public:
	HessianOrb();
	~HessianOrb();

	/**
	 * @brief Sets the object containing Jacobian blocks for matrix A.
	 * @param jacobiansToABlocks Shared pointer to the vector of Jacobian blocks.
	 */
	virtual void setjacobiansToABlocksObject(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToABlocks) { this->jacobiansToABlocks = jacobiansToABlocks; }

	/**
	 * @brief Sets the object containing Jacobian blocks for matrix B.
	 * @param jacobiansToBBlocks Shared pointer to the vector of Jacobian blocks.
	 */
	virtual void setjacobiansToBBlocksObject(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToBBlocks) { this->jacobiansToBBlocks = jacobiansToBBlocks; }

	/**
	 * @brief Sets the object containing Jacobian blocks for matrix W.
	 * @param jacobiansToWBlocks Shared pointer to the vector of Jacobian blocks.
	 */
	virtual void setjacobiansToWBlocksObject(std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToWBlocks) { this->jacobiansToWBlocks = jacobiansToWBlocks; }

	/**
	 * @brief Retrieves the blocks for matrix A.
	 * @return Shared pointer to the vector of blocks.
	 */
	virtual std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> getABlocks() { return Ablocks; }

	/**
	 * @brief Retrieves the blocks for matrix B.
	 * @return Shared pointer to the vector of blocks.
	 */
	virtual std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> getBBlocks() { return BBlocks; }

	/**
	 * @brief Calculates the inverse of matrix B.
	 */
	void calculateInvB();

	void initialize() override;
	void finalize() override;
	void updateHessian() override;

	/**
	 * @brief Retrieves the maximum value of the diagonal of the Hessian matrix.
	 * @return Maximum diagonal value.
	 */
	double getMaxDiagonalValue() override;

	/**
	 * @brief Sets the Jacobian blocks for two vertices and an edge.
	 * @param vertex1Location Location of the first vertex.
	 * @param vertex2Location Location of the second vertex.
	 * @param edgeLocation Location of the edge.
	 * @param jacobian1Block Shared pointer to the first Jacobian block.
	 * @param jacobian2Block Shared pointer to the second Jacobian block.
	 */
	void setJacobianVertex(int vertex1Location, int vertex2Location, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian1Block, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian2Block);

	/**
	 * @brief Retrieves the block for matrix B at a given vertex location.
	 * @param vertexLocation Location of the vertex.
	 * @param BBlock Shared pointer to the block.
	 */
	void getBBlock(int vertexLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& BBlock);

	/**
	 * @brief Retrieves the block for matrix A at a given vertex location.
	 * @param vertexLocation Location of the vertex.
	 * @param Ablock Shared pointer to the block.
	 */
	void getAblock(int vertexLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& Ablock);

	/**
	 * @brief Retrieves the block for matrix W at a given edge location.
	 * @param edgeLocation Location of the edge.
	 * @param Wblock Shared pointer to the block.
	 */
	void getWblock(int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& Wblock);

	/**
	 * @brief Retrieves the block for matrix W at a given edge location.
	 * @param edgeLocation Location of the edge.
	 * @return Shared pointer to the block.
	 */
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> getWblock(int edgeLocation);
};



// bVectorBase.h
class bVectorBase : public BaseDataStructure {
protected:

public:
    bVectorBase();
    ~bVectorBase();

};

// bVectorOrb.h
class bVectorOrb : public BaseDataStructure {
protected:
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type1BVec;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type2BVec;

	std::shared_ptr<Eigen::VectorXd> b;

	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType1; //[Vertex][Edge]
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType2; //[Vertex][Edge]
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType1; //[Vertex][Edge]
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType2; //[Vertex][Edge]
public:
	bVectorOrb();
	~bVectorOrb();

	/**
	 * @brief Retrieves the 'b' vector for a specific parameter set.
	 * @param isFirst If true, retrieves the first type 'b' vector; otherwise, the second.
	 * @param vertexLocation Location of the vertex.
	 * @return Shared pointer to the 'b' vector map.
	 */
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getBForParameter(bool isFirst, int vertexLocation);

	/**
	 * @brief Sets the Jacobians for specific parameter sets.
	 * @param jacobiansToType1 Shared pointer to the vector of Jacobians for type 1.
	 * @param jacobiansToType2 Shared pointer to the vector of Jacobians for type 2.
	 */
	virtual void setJacobiansToType1Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType1) { this->jacobiansToType1 = jacobiansToType1; }
	virtual void setJacobiansToType2Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType2) { this->jacobiansToType2 = jacobiansToType2; }

	/**
	 * @brief Sets the residuals for specific parameter sets.
	 * @param residualsToType1 Shared pointer to the vector of residuals for type 1.
	 * @param residualsToType2 Shared pointer to the vector of residuals for type 2.
	 */
	virtual void setResidualsToParameters1Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType1) { this->residualsToType1 = residualsToType1; }
	virtual void setResidualsToParameters2Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType2) { this->residualsToType2 = residualsToType2; }

	/**
	 * @brief Sets a residual for a specific edge and parameter set.
	 * @param vertex1Location Location of the first vertex.
	 * @param vertex2Location Location of the second vertex.
	 * @param residual Shared pointer to the map of the residual.
	 */
	virtual void setResidualEdge(int vertex1Location, int vertex2Location, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> residual);

	/**
	 * @brief Retrieves the 'b' vector.
	 * @return Shared pointer to the 'b' vector.
	 */
	virtual std::shared_ptr<Eigen::VectorXd>  getBVector() { return b; }

	/**
	* @brief Updates the 'b' vector based on the current Jacobians and residuals.
	*/	
	virtual void updateb();

	/**
	* @brief Initializes the 'b' vector.
	*/
	virtual void initialize() override;

	/**
	* @brief Finalizes the 'b' vector.
	*/
	void finalize() override;


	/**
	 * @brief Retrieves the maximum coefficient from the 'b' vector.
	 * @return Maximum coefficient.
	 */
	virtual double getMaxCoeff();
};



// SolverBase.h
class SolverBase : public BaseDataStructure {
protected:

public:
	SolverBase();
	~SolverBase();

	/**
	 * @brief Solves the optimization problem.
	 * @return True if solved successfully, false otherwise.
	 */
	virtual bool solve() = 0;

};


// SolverOrb.h
class SolverOrb : public SolverBase {
protected:
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> Ablocks;
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BBlocks;

public:
	SolverOrb();
	~SolverOrb();

	/**
	 * @brief Sets the 'A' Hessian blocks for the solver.
	 * @param Ablocks Shared pointer to the vector of 'A' Hessian blocks.
	 */

	virtual void setAHessianBlocksObject(std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> Ablocks) { this->Ablocks = Ablocks; }

	/**
	 * @brief Sets the 'B' Hessian blocks for the solver.
	 * @param BBlocks Shared pointer to the vector of 'B' Hessian blocks.
	 */
	virtual void setBHessianBlocksObject(std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BBlocks){ this->BBlocks = BBlocks; }
	
	/**
	  * @brief Sets the 'b' vector for the solver.
	  * @param type1BVec Shared pointer to the 'b' vector.
	  */
	virtual void setBvector(std::shared_ptr<Eigen::VectorXd> type1BVec) = 0;

	/**
	 * @brief Sets the update vector for the solver.
	 * @param updateVector Shared pointer to the update vector.
	 */
	virtual void setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector) = 0;

	/**
	 * @brief Retrieves the norm of the update vector.
	 * @return Norm of the update vector.
	 */
	virtual double getUpdateNorm() = 0;

	/**
	 * @brief Calculates the dot product between two specified vectors.
	 * @param firstVec Name of the first vector ("bVec" or "updateVec").
	 * @param secondVec Name of the second vector ("bVec" or "updateVec").
	 * @return Result of the dot product.
	 */
	virtual double dotProduct(std::string firstVec, std::string secondvec) = 0;

	/**
	 * @brief Sets an edge for the solver.
	 * @param vertex1Location Location of the first vertex.
	 * @param vertex2Location Location of the second vertex.
	 * @param edgeLocation Location of the edge.
	 * @param WBlock Shared pointer to the 'W' block of the Hessian.
	 */
	virtual void setEdge(int vertex1Location, int vertex2Location, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock) = 0;
};


/**
 * @class SolverMarginalized
 * @brief Solver class for handling marginalized optimization in ORB-SLAM.
 */
class SolverMarginalized : public SolverOrb
{
protected:
	/// Stored in the solver.
	std::shared_ptr<Eigen::MatrixXd> BInv;
	std::shared_ptr<Eigen::MatrixXd> ActiveHessian;
	std::shared_ptr<Eigen::MatrixXd> YBlocksMatrix; ///< W*Binv | vertex1 x vertex2*numedges
	std::shared_ptr<Eigen::VectorXd> activeB;

	/// These come from bVector.
	std::shared_ptr<Eigen::VectorXd> bVector;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type1BVec;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type2BVec;

	/// These come from the parameterVector.
	std::shared_ptr<Eigen::VectorXd> updateVector;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type1UpdateVec;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type2UpdateVec;

	double previousLamda;
	bool activeHessianSetted = false;

	/// To make the intermediary Hessian BInv blocks.
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BInvBlocks;

	/// To make the intermediary Y blocks.
	std::shared_ptr<std::vector<std::map<int, std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>> HessianBlocksToYBlocks; ///< [Vertex1][Vertex2][0:Wblock,1:BinvBlock]

	/// To make the final Hessian.
	std::shared_ptr<std::vector<std::map<int, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> Yblocks; ///< [Vertex1][Vertex2]
	std::shared_ptr<std::vector<std::map<int, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> type2BVecstoActiveB; ///< [Vertex1][Vertex2]
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> ActiveHessianBlocks; ///< [Row][Column]

	std::shared_ptr<std::vector<std::vector<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>>> YWBlockstoHessianBlocks; ///< [Vertex1][Vertex2][edgeLocation][0:Yblock,1:Wblock]

	/// To get the marginalized parameters.
	std::shared_ptr<std::vector<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>>> w_delta_toMarginalizedB; ///< [Vertex2][edgelocation][0:Wblock,1:deltaType1Vec]

	/**
	 * @brief Applies the lambda parameter for optimization.
	 */
	void applyLamda();

	/**
	 * @brief Updates the inverse of matrix B.
	 * @return True if successful, otherwise false.
	 */
	bool updateInvB();

	/**
	 * @brief Updates the Y blocks matrix.
	 */
	void updateYBlocks();

	/**
	 * @brief Updates the active Hessian matrix.
	 */
	void updateActiveHessian();

	/**
	 * @brief Alternative method to update the active Hessian matrix.
	 */
	void updateActiveHessianV2();

	/**
	 * @brief Updates the active structure.
	 */
	void updateActiveStructure();

	/**
	 * @brief Sets the active structure.
	 */
	void setActiveStructure();

	/**
	 * @brief Updates the active B vector.
	 */
	void updateActiveB();

	/**
	 * @brief Solves for type 1 vectors.
	 * @return True if successful, otherwise false.
	 */
	bool solveForType1();

	/**
	 * @brief Solves for type 2 vectors.
	 */
	void solveForType2();

public:
	SolverMarginalized();
	~SolverMarginalized();

	void initialize() override;
	void finalize() override;

	/**
	 * @brief Sets the update vector.
	 * @param updateVector Shared pointer to the update vector.
	 */
	virtual void setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector) override;

	/**
	 * @brief Sets the B vector.
	 * @param type1BVec Shared pointer to the B vector.
	 */
	virtual void setBvector(std::shared_ptr<Eigen::VectorXd> type1BVec) override;

	/**
	 * @brief Retrieves the norm of the update vector.
	 * @return Norm of the update vector.
	 */
	virtual double getUpdateNorm() override;

	/**
	 * @brief Computes the dot product of two vectors.
	 * @param firstVec Name of the first vector.
	 * @param secondvec Name of the second vector.
	 * @return Dot product of the two vectors.
	 */
	virtual double dotProduct(std::string firstVec, std::string secondvec) override;

	/**
	 * @brief Sets the type 1 B vector object.
	 * @param type1BVec Shared pointer to the type 1 B vector.
	 */
	virtual void setType1BVectorObject(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type1BVec) { this->type1BVec = type1BVec; }

	/**
	 * @brief Sets the type 2 B vector object.
	 * @param type2BVec Shared pointer to the type 2 B vector.
	 */
	virtual void setType2BVectorObject(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type2BVec) { this->type2BVec = type2BVec; }

	/**
	 * @brief Sets the edge with the given locations and W block.
	 * @param vertex1Location Location of the first vertex.
	 * @param vertex2Location Location of the second vertex.
	 * @param edgeLocation Location of the edge.
	 * @param WBlock Shared pointer to the W block.
	 */
	virtual void setEdge(int vertex1Location, int vertex2Location, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock) override;

	/**
	 * @brief Solves the system.
	 * @return True if successful, otherwise false.
	 */
	virtual bool solve() override;
};

