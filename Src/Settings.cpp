#include "Settings.h"

bool OptimizerSettings::validate()
{
    // Check for the validity of the settings here
    //add parameter update normal one if not set
    // user can set robustkernel only if Robust type is "custom" and robust is true
    //user have to set the reprojection function otherwise the settings are invalid

    if (parameterUpdate == nullptr)
    {
	    parameterUpdate = std::make_unique<ParameterUpdateBase>();
	}

    this->isValid = true;
    return this->isValid;
} 
