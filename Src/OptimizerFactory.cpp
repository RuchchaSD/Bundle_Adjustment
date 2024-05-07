#include "OptimizerFactory.h"
#include "Misc.h"
#include "RobustKernels.h"
#include "ReprojectionBase.h"
#include "ParameterUpdateBase.h"
#include "ProblemStructure.h"
#include "OptimizationAlgorithmBase.h"
#include "OptimizationAlgorithmLevenberg.h"

std::unique_ptr<OptimizerBase> OptimizerFactoryOrbSlamNew::getOptimizer(std::unique_ptr<OptimizerSettings> settings)
{
    if (!settings->getIsValid()) {
        if (!settings->validate())
            std::cerr << "Optimizer is not validated" << std::endl;
    }
    
    std::unique_ptr<OptimizerBase> optimizerAlgorithm = std::make_unique<OptimizerOrbSlam>();
    std::unique_ptr<ProblemStructure> structure = std::make_unique<ProblemStructure>();
    std::shared_ptr<Propertise> p = std::make_shared<Propertise>();
    

    p->DebugMode = settings->DebugMode;
    p->verbosityLevel = settings->VerbosityLvl;
    p->vertex1Size = settings->vertexType1Size;
    p->vertex2Size = settings->vertexType2Size;
    p->edgeSize = settings->edgeSize;
    //algorithm
    std::unique_ptr<OptimizationAlgorithmBase> algorithm = nullptr;
    if (settings->Algorithm == "LM") {
        p->isLM = true;
        algorithm = std::make_unique<OptimizationAlgorithmLevenberg>();
    }
    else if (settings->Algorithm == "GN")
    {
        p->isLM = false;
        std::cerr << "Gauss Newton is not implemented yet" << std::endl;
    }
    else{
		std::cerr << "Optimizer type is not recognized" << std::endl;
	}

    p->maxIterations = settings->MaxIterations;
    p->maxRepeats = settings->maxRepeats;

    


    std::shared_ptr<parameterVectorBase> ParametersData = std::make_shared<parameterVectorMarginalized>();
    ParametersData->setParameterUpdate(std::move(settings->getParameterUpdate()));

    structure->setParameters(ParametersData);



    std::shared_ptr<RobustKernelBase> robustKernel = nullptr;
    if (settings->Robust) {
        p->isRobust = true;
        if (settings->RobustType == "Huber") {
			robustKernel = std::make_shared<HuberKernel>();
		}
        else if (settings->RobustType == "Cauchy") {
			//robustKernel = std::make_shared<CauchyKernel>(settings->RobustParameter);
            std::cerr << "Cauchy kernel is not implemented yet" << std::endl;
		}
        else if (settings->RobustType == "PseudoHuber") {
			//robustKernel = std::make_shared<PseudoHuberKernel>(settings->RobustParameter);
			std::cerr << "PseudoHuber kernel is not implemented yet" << std::endl;
		}
        else if (settings->RobustType == "custom") {
			robustKernel = settings->getRobustKernel();
		}
        else {
			std::cerr << "Robust type is not recognized" << std::endl;
		}
	}
    else {
		p->isRobust = false;
	}

    p->delta = settings->RobustParameter;
    
    p->isSparse = settings->isSparse; // no dense implementation yet
    std::shared_ptr<JacobianCompressed> jacobian =nullptr;
    std::shared_ptr<residualsCompressed> residuals = nullptr;
    if (p->isSparse) {
        jacobian = std::make_shared<JacobianCompressed>();
        jacobian->setReprojection(settings->getReprojection());
        jacobian->setRobustKernel(robustKernel);

        residuals = std::make_shared<residualsCompressed>();
        residuals->setReprojection(settings->getReprojection());
        residuals->setRobustKernel(robustKernel);
    }
    else {
		std::cerr << "Dense implementation is not available yet" << std::endl;
	}

    std::shared_ptr<InformationMatrixOrb> informationMatrix = std::make_shared<InformationMatrixCompressed>();
    jacobian->setInformationVector(informationMatrix);
    residuals->setInformationVector(informationMatrix);

    structure->setJacobian(jacobian);
    structure->setResidual(residuals);


    //todo : extend hessian to have LM methods| class HessianLm  : public HessianOrb
    std::shared_ptr<HessianOrb> Hessian = std::make_shared<HessianOrb>();
    structure->setHessian(Hessian);

    std::shared_ptr<bVectorOrb> bVector = std::make_shared<bVectorOrb>();
    structure->setbVector(bVector);

    p->isMarginalized = settings->isMalginalized;
    
    std::shared_ptr<SolverOrb> solver = nullptr;
    if (p->isMarginalized) {
        solver = std::make_shared<SolverMarginalized>();
    }
    else {
        std::cerr << "Non marginalized Solver is not implemented yet" << std::endl;
    }

    structure->setSolver(solver);

    p->hasFixedVertices = settings->isFixedAvailable;
    if (p->hasFixedVertices) {
		std::cerr << "a method to fix vertices is not implemented yet" << std::endl;
	}

    if (settings->needRemove) {
        std::cerr << "a method to remove vertices is not implemented yet" << std::endl;
    }

    optimizerAlgorithm->SetStructure(std::move(structure));
    optimizerAlgorithm->SetOptimizer(std::move(algorithm));
    optimizerAlgorithm->setPropertise(p);





    return std::move(optimizerAlgorithm);
}
