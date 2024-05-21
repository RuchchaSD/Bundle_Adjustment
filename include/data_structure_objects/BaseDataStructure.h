#pragma once
#include <iostream>
#include "Misc.h" 
#include "Timer.h"


class BaseDataStructure
{
protected:
	bool isInitialized;
	std::shared_ptr<Propertise> p;
	std::shared_ptr<Timer> timer;
public:
	BaseDataStructure() {
		this->isInitialized = false;
		this->p = nullptr;
		this->timer = nullptr;
	}
	~BaseDataStructure() {}

	virtual void initialize() = 0;
	virtual void finalize() = 0;
	virtual void setPropertise(std::shared_ptr<Propertise> p) { this->p = p; }
	virtual void setTimer(std::shared_ptr<Timer> timer) {

		if (!isInitialized)
			this->timer = timer;
		else
			std::cout << "Timer can only be set before initialization" << std::endl;
	}
	virtual std::shared_ptr<Propertise> getPropertise() { return p; }
	virtual std::shared_ptr<Timer> getTimer() { return timer; }
	virtual bool getInitialized() { return isInitialized; }
};