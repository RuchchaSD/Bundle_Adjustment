// ParameterUpdateBase.h
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * @class ParameterUpdateBase
 * @brief Base class for parameter update strategies.
 */
class ParameterUpdateBase {
protected:

public:
    /**
     * @brief Default constructor.
     */
    ParameterUpdateBase();

    /**
     * @brief Destructor.
     */
    ~ParameterUpdateBase();

    /**
     * @brief Virtual function to update parameters.
     * @param parameters Parameters to update.
     * @param update Update values to apply.
     */
    virtual void update(Eigen::VectorXd& parameters, const Eigen::VectorXd& update);
};
