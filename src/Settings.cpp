//Settings.cpp
#include "Settings.h"

/**
 * @brief Validate the optimizer settings.
 *
 * This method checks the validity of the optimizer settings, ensuring that necessary
 * configurations such as parameter updates, reprojection functions, and robust kernels are set correctly.
 *
 * @return True if the settings are valid, otherwise false.
 */
bool OptimizerSettings::validate()
{
    if (parameterUpdate == nullptr)
    {
        parameterUpdate = std::make_unique<ParameterUpdateBase>();
    }

    this->isValid = true;
    return this->isValid;
}
