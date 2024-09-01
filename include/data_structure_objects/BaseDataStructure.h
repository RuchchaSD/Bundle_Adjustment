// BaseDataStructure.h
#pragma once
#include <iostream>
#include "Misc.h"
#include "Timer.h"
#include <memory>

/**
 * @class BaseDataStructure
 * @brief Base class for data structures used in SLAM optimization.
 */
class BaseDataStructure {
protected:
    bool isInitialized;
    std::shared_ptr<Propertise> p;
    std::shared_ptr<Timer> timer;

public:
    /**
     * @brief Default constructor.
     */
    BaseDataStructure() : isInitialized(false), p(nullptr), timer(nullptr) {}

    /**
     * @brief Destructor.
     */
    ~BaseDataStructure() {}

    /**
     * @brief Pure virtual function to initialize the data structure.
     */
    virtual void initialize() = 0;

    /**
     * @brief Pure virtual function to finalize the data structure.
     */
    virtual void finalize() = 0;

    /**
     * @brief Sets the properties of the data structure.
     * @param p Shared pointer to the properties.
     */
    virtual void setPropertise(std::shared_ptr<Propertise> p) { this->p = p; }

    /**
     * @brief Sets the timer for the data structure.
     * @param timer Shared pointer to the timer.
     */
    virtual void setTimer(std::shared_ptr<Timer> timer) {
        if (!isInitialized)
            this->timer = timer;
        else
            std::cout << "Timer can only be set before initialization" << std::endl;
    }

    /**
     * @brief Gets the properties of the data structure.
     * @return Shared pointer to the properties.
     */
    virtual std::shared_ptr<Propertise> getPropertise() { return p; }

    /**
     * @brief Gets the timer of the data structure.
     * @return Shared pointer to the timer.
     */
    virtual std::shared_ptr<Timer> getTimer() { return timer; }

    /**
     * @brief Checks if the data structure is initialized.
     * @return True if initialized, false otherwise.
     */
    virtual bool getInitialized() { return isInitialized; }
};
