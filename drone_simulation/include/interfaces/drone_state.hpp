#pragma once
#include <memory>
#include <cstring>
#include "simulation.hpp"

class IDroneState {
public:
    virtual ~IDroneState() = default;
 
    // Виконати логіку стану, повернути наступний стан.
    // Якщо стан не змінився — повернути nullptr
    // (головний цикл залишить поточний).
    virtual std::unique_ptr<IDroneState> execute(DroneContext& context) = 0;
 
    virtual const std::string name() const = 0;
};

