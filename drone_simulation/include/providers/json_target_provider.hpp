#pragma once
#include <cstring>
#include <vector>
#include "interfaces/target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
    std::vector<std::vector<Coord>> targets;
    int count;

    public:
        JsonTargetProvider(const std::string path);
        ~JsonTargetProvider() override;
        
        auto getTargetCount() -> int override;
        auto getTarget(int i) -> std::vector<Coord>& override;
};