#pragma once

#include "interfaces/target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
    Coord** targets;
    int count;
    int timeSteps;

    public:
        JsonTargetProvider(const char* path);
        ~JsonTargetProvider() override;
        
        auto getTargetCount() -> int override;
        auto getTarget(int i) -> Coord* override;
        auto getTimeSteps() -> int override;
};