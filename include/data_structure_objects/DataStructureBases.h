#pragma once
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "RobustKernels.h"
#include "ParameterUpdateBase.h"
#include "ReprojectionBase.h"
#include "Misc.h"
#include <iostream>
#include "Timer.h"
#include "BaseDataStructure.h"



class InformationMatrixBase : public BaseDataStructure
{
protected:


public:
	InformationMatrixBase() : BaseDataStructure() {}
	~InformationMatrixBase(){}

	virtual double getInfSqrt(int index) = 0;
	virtual double getInf(int index) = 0;
};

// in OrbSlam they do not store off diagonal values of the information matrix
class InformationMatrixOrb : public InformationMatrixBase
{
protected:

public:
	InformationMatrixOrb();
	~InformationMatrixOrb();

	virtual void updateInfVec(int edgeLocation, const Eigen::VectorXd& vec) = 0;
	virtual std::shared_ptr<Eigen::VectorXd> getInfVec() = 0 ;
	virtual std::shared_ptr<Eigen::VectorXd> getInfVecSqrt() = 0;
	virtual void setInformationVector(const Eigen::VectorXd& infVec) = 0;
	virtual void setInformationVectorSqrt(const Eigen::VectorXd& infVec) = 0;
	virtual void printInformationMatrix() = 0;
};

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


class parameterVectorBase : public BaseDataStructure
{
protected:
	bool isBackedUp;
	double maxCoeffValue;

	std::unique_ptr<ParameterUpdateBase> parameterUpdate;
public:
	parameterVectorBase();
	~parameterVectorBase();

	virtual void setParameterUpdate(std::unique_ptr<ParameterUpdateBase> parameterUpdate) { this->parameterUpdate = std::move(parameterUpdate); }
	virtual void setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector) = 0; 
	virtual void setParameterVector(const Eigen::VectorXd& parameterVector) = 0;
	virtual std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getParameterVectorMap(bool isFirst, int location) = 0;

	virtual std::shared_ptr<Eigen::VectorXd> getUpdateVector() = 0;
	virtual void updateParameterVector() = 0;

	virtual void printParameterVector() = 0;

	virtual void backUp() = 0;
	virtual void restore() = 0;
	virtual double maxCoeff() = 0;
};

class parameterVectorMarginalized : public parameterVectorBase
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



class JacobianBase : public BaseDataStructure
{
protected:

	bool isUpdated;

	std::shared_ptr<RobustKernelBase> kernel;
	std::shared_ptr<InformationMatrixOrb> informationMatrix;
	std::shared_ptr<ReprojectionBase> reprojection;

	virtual void computeJacobianVertex(bool reverse, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param1, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param2, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian, double stepSize) = 0;

public:
	JacobianBase();
	~JacobianBase();
	virtual void setRobustKernel(std::shared_ptr<RobustKernelBase> kernel) { this->kernel = kernel; }
	virtual void setInformationVector(std::shared_ptr<InformationMatrixOrb> informationMatrix) { this->informationMatrix = informationMatrix; }
	virtual std::shared_ptr<InformationMatrixOrb> getInformationVector() { return informationMatrix; }
	virtual void setReprojection(std::shared_ptr<ReprojectionBase> reprojection) { this->reprojection = reprojection; }
	virtual void applyRobustKernel() = 0;
	virtual void updateJacobian() = 0;

	bool getIsUpdated() { return isUpdated; }
	void setIsUpdated(bool isUpdated) { this->isUpdated = isUpdated; }

};

class JacobianCompressed : public JacobianBase
{
	protected:
		// memory locations of the parameters to calculate the jacobian
		std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets; // [0:Vertex1, 1:Vertex2]

		// memory locations of the jacobians
		std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobianSets; // [0:Vertex1, 1:Vertex2]


		std::shared_ptr<Eigen::MatrixXd> jacobian1;
		std::shared_ptr<Eigen::MatrixXd> jacobian2;

		//update jacobian vertex
		virtual void computeJacobianVertex(bool reverse, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param1, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> param2, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian, double stepSize) override;

	public:

		JacobianCompressed();
		~JacobianCompressed();

		void initialize() override;
		void finalize() override;
		std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> getJacobianVertex(bool isFirst, size_t edgeLocation );

		void updateJacobian() override;

		void rowWiseMultiply(const Eigen::VectorXd& vec);
		void applyRobustKernel() override;
		virtual void applyInformationSqrt();

		void setParameterSets(std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets) { this->parameterSets = parameterSets; }
		std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> getParameterSets() { return parameterSets; }
		void insertParameterPair( int edgeLocation , std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>& paramSet);
};



class residualsBase : public BaseDataStructure
{
protected:
	bool isUpdated;

	std::shared_ptr<RobustKernelBase> kernel;
	std::shared_ptr<InformationMatrixOrb> informationMatrix;
	std::shared_ptr<ReprojectionBase> reprojection;
public:
	residualsBase();
	~residualsBase();

	virtual void setRobustKernel(std::shared_ptr<RobustKernelBase> kernel) { this->kernel = kernel; }
	virtual void setInformationVector(std::shared_ptr<InformationMatrixOrb> informationMatrix) { this->informationMatrix = informationMatrix; }
	virtual std::shared_ptr<InformationMatrixOrb> getInformationVector();
	virtual void setReprojection(std::shared_ptr<ReprojectionBase> reprojection) { this->reprojection = reprojection; }
	virtual void applyRobustKernel() = 0;

	bool getIsUpdated() { return isUpdated; }
	void setIsUpdated(bool isUpdated) { this->isUpdated = isUpdated; }

	virtual void updateResiduals() = 0;

	virtual double norm() = 0;
	virtual double getMaxCoeff() = 0;
};

class residualsCompressed : public residualsBase
{
protected:
	std::shared_ptr<Eigen::VectorXd> residuals;
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets; // [0:Vertex1, 1:Vertex2]
	std::shared_ptr<const Eigen::VectorXd>  observationsVector;

public:
	residualsCompressed();
	~residualsCompressed();

	void initialize() override;
	void finalize() override;
	void applyRobustKernel() override;
	virtual void applyInformationSqrt();

	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getResiduals(int edgeLocation);

	void setObservationsVector(std::shared_ptr<const Eigen::VectorXd> observations) { this->observationsVector = observations;}

	void setParameterSets(std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> parameterSets) { this->parameterSets = parameterSets; }
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> getParameterSets() { return parameterSets; }
	void insertParameterPair(int edgeLocation, std::pair<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>& paramSet);

	void updateResiduals() override;
	double norm() override;
	double getMaxCoeff() override;
};



class HessianBase : public BaseDataStructure
{
	protected:

	public:
		HessianBase();
		~HessianBase();

		virtual void updateHessian() = 0;
		virtual double getMaxDiagonalValue() = 0;
		
};

class HessianOrb : public HessianBase
{
protected:
	//todo add updated flags for A,B,W ad check them before getting to next stage
	
	//this is where the values for the hessian are stored
	std::shared_ptr<Eigen::MatrixXd> A;
	std::shared_ptr<Eigen::MatrixXd> B;
	std::shared_ptr<Eigen::MatrixXd> W;

	//memory locations of the jacobians to calculate different blocks of the hessian
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToABlocks; //[Vertex][Edge]
	std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToBBlocks; //[Vertex][Edge]
	std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToWBlocks; //[edge][0:Vertex1,1:Vertex2]

	//memory locations of the differet types blocks of hessian
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> Ablocks;
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BBlocks;
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> wBlocks; // [edgeId]

	virtual void updateA();
	virtual void updateB();
	virtual void updateW();

	void getAJacobianBlock(int vertexLocation, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& jacobianBlock);
	void getBJacobianBlock(int vertexLocation, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& jacobianBlock);

public:
	HessianOrb();
	~HessianOrb();

	virtual void setjacobiansToABlocksObject(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToABlocks) { this->jacobiansToABlocks = jacobiansToABlocks; }
	virtual void setjacobiansToBBlocksObject(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToBBlocks) { this->jacobiansToBBlocks = jacobiansToBBlocks; }
	virtual void setjacobiansToWBlocksObject(std::shared_ptr<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToWBlocks) { this->jacobiansToWBlocks = jacobiansToWBlocks; }

	virtual std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> getABlocks() { return Ablocks; }
	virtual std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> getBBlocks() { return BBlocks; }

	
	void calculateInvB();

	void initialize() override;
	void finalize() override;
	void updateHessian() override;
	double getMaxDiagonalValue() override;

	void setJacobianVertex(int vertex1Location, int vertex2Location, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian1Block, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> jacobian2Block);

	void getBBlock(int vertexLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& BBlock);
	void getAblock(int vertexLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& Ablock);
	void getWblock(int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>& Wblock);
	std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> getWblock(int edgeLocation);
	
	
};


class bVectorBase : public BaseDataStructure
{
	protected:

	public:
		bVectorBase();
		~bVectorBase();

};

class bVectorOrb : public BaseDataStructure
{
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

	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> getBForParameter(bool isFirst, int vertexLocation);

	virtual void setJacobiansToType1Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType1) { this->jacobiansToType1 = jacobiansToType1; }
	virtual void setJacobiansToType2Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> jacobiansToType2) { this->jacobiansToType2 = jacobiansToType2; }
	virtual void setResidualsToParameters1Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType1) { this->residualsToType1 = residualsToType1; }
	virtual void setResidualsToParameters2Object(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> residualsToType2) { this->residualsToType2 = residualsToType2; }

	virtual void setResidualEdge(int vertex1Location, int vertex2Location, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> residual);
	virtual std::shared_ptr<Eigen::VectorXd>  getBVector() { return b; }
	virtual void updateb();
	virtual void initialize() override;
	void finalize() override;

	virtual double getMaxCoeff();
};



class SolverBase : public BaseDataStructure
{
protected:	


public:
	SolverBase();
	~SolverBase();

	virtual bool solve() = 0;

};

class SolverOrb : public SolverBase
{
protected:
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> Ablocks;
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BBlocks;
	
public:
	SolverOrb();
	~SolverOrb();

	virtual void setAHessianBlocksObject(std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> Ablocks){ this->Ablocks = Ablocks; }
	virtual void setBHessianBlocksObject(std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BBlocks){ this->BBlocks = BBlocks; }
	
	virtual void setBvector(std::shared_ptr<Eigen::VectorXd> type1BVec) = 0;
	virtual void setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector) = 0;

	virtual double getUpdateNorm() = 0;
	virtual double dotProduct(std::string firstVec, std::string secondvec) = 0;

	virtual void setEdge(int vertex1Location, int vertex2Location, int edgeLocation,std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock) = 0;
};


class SolverMarginalized : public SolverOrb
{
protected:
	//these are stored in the solver
	std::shared_ptr<Eigen::MatrixXd> BInv;
	std::shared_ptr<Eigen::MatrixXd> ActiveHessian;
	std::shared_ptr<Eigen::MatrixXd> YBlocksMatrix; // W*Binv | vertex1 x vertex2*numedges
	std::shared_ptr<Eigen::VectorXd> activeB;

	//these come from bVector
	std::shared_ptr<Eigen::VectorXd> bVector;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type1BVec;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type2BVec;

	//these come from the parameterVector
	std::shared_ptr<Eigen::VectorXd> updateVector;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type1UpdateVec;
	std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type2UpdateVec;

	double previousLamda;
	bool  activeHessianSetted = false;

	//to make the intermediary Hessian BInv blocks
	std::shared_ptr<std::vector<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>> BInvBlocks;

	//to make the Intermediary Y blocks
	std::shared_ptr<std::vector<std::map<int, std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>> HessianBlocksToYBlocks; //[Vertex1][Vertex2][0:Wblock,1:BinvBlock]

	//to make the final Hessian
	std::shared_ptr<std::vector<std::map<int, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> Yblocks; //[Vertex1][Vertex2]
	std::shared_ptr<std::vector<std::map<int, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>> type2BVecstoActiveB; //[Vertex1][Vertex2]
	std::shared_ptr < std::vector < std::vector< std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>> ActiveHessianBlocks; //[Row][Column]

	std::shared_ptr<std::vector<std::vector<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>>>>>> YWBlockstoHessianBlocks; //[Vertex1][Vertex2][edgeLocation][0:Yblock,1:Wblock]



	//to get the Margialized Parameters
	std::shared_ptr<std::vector<std::vector<std::pair<std::shared_ptr<Eigen::Block<Eigen::MatrixXd>>, std::shared_ptr<Eigen::Map<Eigen::VectorXd>>>>>> w_delta_toMarginalizedB; //[Vertex2][edgelocation][0:Wblock,1:deltaType1Vec]


	void applyLamda();
	bool updateInvB();
	void updateYBlocks();
	void updateActiveHessian();
	void updateActiveHessianV2();

	void updateActiveStructure();

	void setActiveStructure();

	void updateActiveB();
	bool solveForType1();
	void solveForType2();

public:
	SolverMarginalized();
	~SolverMarginalized();

	void initialize() override;
	void finalize() override;
	virtual void setUpdateVector(std::shared_ptr<Eigen::VectorXd> updateVector) override;

	virtual void setBvector(std::shared_ptr<Eigen::VectorXd> type1BVec) override;
	virtual double getUpdateNorm() override;
	virtual double dotProduct(std::string firstVec, std::string secondvec) override;

	virtual void setType1BVectorObject(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type1BVec) { this->type1BVec = type1BVec; }
	virtual void setType2BVectorObject(std::shared_ptr<Eigen::Map<Eigen::VectorXd>> type2BVec) { this->type2BVec = type2BVec; }

	virtual void setEdge(int vertex1Location, int vertex2Location, int edgeLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock) override;
	virtual bool solve() override;

}; 

