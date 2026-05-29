#pragma once
#include "../simulation.hpp"

class ITargetProvider {
public:
    virtual int getTargetCount() = 0;
    virtual std::vector<Coord>& getTarget(int index) = 0;
    virtual ~ITargetProvider() {}
};