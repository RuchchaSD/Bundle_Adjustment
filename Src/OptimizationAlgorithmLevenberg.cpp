#include "OptimizationAlgorithmLevenberg.h"

OptimizationAlgorithmLevenberg::OptimizationAlgorithmLevenberg()
{
}

OptimizationAlgorithmLevenberg::~OptimizationAlgorithmLevenberg()
{
}

void OptimizationAlgorithmLevenberg::Optimize(int maxIterations)
{
	if (!this->isInitialized) {
		std::cerr << "OptimizationAlgorithmLevenberg::Optimize: Not initialized" << std::endl;
		return;
	}

    if (p->verbosityLevel > 0)
        std::cout << "Optimization started! \n" << std::endl;

    bool stop = false;


    double b_max = 0;
    double b_max_thresh = 1e-6; //these values should be changed with the parameters norm

    double update_norm = 0;
    double update_thresh = 1e-6; //these values should be changed with the parameters norm

    double lamda = 0;
    p->Lamda = lamda; //lamda is stored in properties and used in the solver
    int scale_factor_repeat_lamda = 2;
    double scale_factor_initial_lamda = 1e-6;
    double reduction_factor_success_lamda = 1;//this is calculated at each iteration

    double rho = 0;
    double prev_residuals_norm = 0, current_residuals_norm = 0;

    int current_iteration = 0;

    int max_repeats = p->maxRepeats;
    int repeat_attempt = 0;


    if(p->DebugMode && p->verbosityLevel > 3)
        parameters->printParameterVector(); ///debug

    residuals->updateResiduals();
    current_residuals_norm = residuals->norm();
    jacobian->updateJacobian();
    

    
    Hessian->updateHessian();
    lamda = scale_factor_initial_lamda * Hessian->getMaxDiagonalValue();
    p->Lamda = lamda;


    bVector->updateb();
    b_max = bVector->getMaxCoeff();

    if (p->verbosityLevel > 1)
        std::cout << "initial mu: " << lamda << " | Initial max error: " << residuals->getMaxCoeff() << " | Initial Norm Error: " << current_residuals_norm << " | Initial max b: " << b_max << "\n";

    stop = b_max < b_max_thresh;
    while (!stop && current_iteration < maxIterations) {
        current_iteration++;
        if (p->verbosityLevel > 1)
            std::cout << "                 Iteration: " << current_iteration << "\n";

        repeat_attempt = 0;

        while (repeat_attempt < max_repeats) {

            stop = !solver->solve();

            update_norm = solver->getUpdateNorm();

            parameters->updateParameterVector();


            //calclate rho
            prev_residuals_norm = current_residuals_norm;
            residuals->updateResiduals();
            current_residuals_norm = residuals->norm();

            double numerator = 0, denominator = 0;
            numerator = (std::pow(prev_residuals_norm,2) - std::pow(current_residuals_norm,2)); // square these terms if there are errors
            //denominator = (solver->getUpdateVector()->transpose() * (lamda * solver->getUpdateVector() + *bVector->getBVector()))[0];
            denominator = lamda * std::pow(solver->getUpdateNorm(), 2) + solver->dotProduct("bVec", "updateVec");
            rho = numerator / denominator;


            if (p->verbosityLevel > 1)
                std::cout << "rho: " << rho << " | update_norm: " << update_norm;

            if (rho >= 0) {
                if (p->verbosityLevel > 1)
                    std::cout << " | Success";

                //print some info
                if (update_norm < update_thresh) {//update_thresh should be multiplied with the norm of the parameters
                    if (p->verbosityLevel > 1)
                        std::cout << " | Update norm is less than threshold: " << update_norm << " < " << update_thresh << std::endl;
                    stop = true;
                    break;
                }

                //residuals already updated
                jacobian->updateJacobian();
                Hessian->updateHessian();
                bVector->updateb();

                b_max = bVector->getMaxCoeff();

                if (p->verbosityLevel > 0)
                    std::cout << " | b_max: " << b_max << " | mu: " << lamda << " | error norm: " << residuals->norm();

                reduction_factor_success_lamda = 1 - std::pow(2 * rho - 1, 3);
                reduction_factor_success_lamda = std::min(2.0 / 3, reduction_factor_success_lamda);
                reduction_factor_success_lamda = std::max(1.0 / 3, reduction_factor_success_lamda);
                lamda *= reduction_factor_success_lamda;
                p->Lamda = lamda;
                scale_factor_repeat_lamda = 2;

                if (p->verbosityLevel > 0) {
                    std::cout << " | new mu: " << lamda;


                    std::cout << "\n";
                }

                if (b_max < b_max_thresh) {
                    if (p->verbosityLevel > 0)
                        std::cout << "b_max is less than threshold: " << b_max << " < " << b_max_thresh << std::endl;
                    stop = true;
                    break;
                }
            }
            else {
                p->isRepeatAttempt = true;
                

                if (p->verbosityLevel > 1)
                    std::cout << " | Repeat | numerator: " << numerator << " | Denominator: " << denominator << " mu: " << lamda << std::endl;

                parameters->restore();
                
                current_residuals_norm = prev_residuals_norm; //revert the residuals

                lamda = lamda * scale_factor_repeat_lamda;
                p->Lamda = lamda;
                scale_factor_repeat_lamda = 2 * scale_factor_repeat_lamda;
            }

            if (rho >= 0 || stop) break;
        }
        repeat_attempt++;
        if (repeat_attempt == max_repeats) {
            stop = true;
            if (p->verbosityLevel > 1)
                std::cout << "Max repeats reached | Max repeats: " << max_repeats << std::endl;
        }

    }
    if (p->verbosityLevel > 0)
        std::cout << "\nOptimization finished\n" << "b max :" << b_max << " | update_norm: " << update_norm << " | Iterations: " << current_iteration << "\n";

}

void OptimizationAlgorithmLevenberg::initialize()
{
    if(p == nullptr)
		std::cerr << "OptimizationAlgorithmLevenberg::initialize: propertise not set" << std::endl;
    //do other checks

    jacobian = structure->getJacobian();
    residuals = structure->getResidual();
    Hessian = structure->getHessian();
    bVector = structure->getbVector();
    parameters = structure->getParameters();
    solver = structure->getSolver();
    isInitialized = true;
}

void OptimizationAlgorithmLevenberg::finalize()
{
    if (!isInitialized) {
		std::cerr << "OptimizationAlgorithmLevenberg::finalize: Not initialized" << std::endl;
		return;
	}
	//do other checks
    solver.reset();
    parameters.reset();
    bVector.reset();
    Hessian.reset();
    residuals.reset();
    jacobian.reset();
    structure->finalize();
    isInitialized = false;
}
 