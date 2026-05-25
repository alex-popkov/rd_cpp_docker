#pragma once
#include "../simulation.hpp"

class IConfigLoader {
public:
    virtual void load() = 0;
    virtual void getConfig() = 0;
    virtual AmmoParams getAmmoParams() = 0;
    virtual ~IConfigLoader() {}
};