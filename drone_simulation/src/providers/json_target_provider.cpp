#include "providers/json_target_provider.hpp"

JsonTargetProvider::JsonTargetProvider(const char* path) {
    json jsonFile = parseJSONfile(path);
    count = jsonFile.size();//targetCount
    targets = fillTargets(jsonFile);
    timeSteps = jsonFile["timeSteps"];
}

JsonTargetProvider::~JsonTargetProvider() {
    if (!targets) {
        return;
    }

    for (int i = 0; i < count; i++) {
        delete[] targets[i];   
        targets[i] = nullptr;  
    }  
    delete[] targets;
    targets = nullptr;
}

auto JsonTargetProvider::getTargetCount() -> int {
    return count;
}

auto JsonTargetProvider::getTarget(int i) -> Coord* {
    return targets[i];
}

auto JsonTargetProvider::getTimeSteps() -> int {
    return timeSteps;
}
