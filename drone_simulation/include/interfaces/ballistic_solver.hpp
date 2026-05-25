#pragma once

class IBallisticSolver {
public:
    virtual int solve() = 0;
    virtual ~IBallisticSolver() {}
};