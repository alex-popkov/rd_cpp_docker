#pragma once

#include "interfaces/target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
    Coord** targets;
    int count;
    int timeSteps;

    public:

        JsonTargetProvider(const char* path) {
            json jsonFile = parseJSONfile(path);
            count = jsonFile.size();
            targets = fillTargets(jsonFile);
            timeSteps = jsonFile["timeSteps"];
        }
        
        auto getTargetCount() -> int override {
            return count;
        }

        auto getTarget(int i) -> Coord* override {
            return targets[i];
        }
        
        auto getTimeSteps() -> int override {
            return timeSteps;
        }
        
};