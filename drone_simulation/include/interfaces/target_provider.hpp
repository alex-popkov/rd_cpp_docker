#pragma once
#include "../simulation.hpp"

class ITargetProvider {
public:
    virtual int getTargetCount() = 0;
    virtual int getTimeSteps() = 0;
    virtual Coord* getTarget(int index) = 0;
    virtual ~ITargetProvider() {}

};