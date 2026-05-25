#include "interfaces/target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
    Coord** targets;
    int count;
    
    JsonTargetProvider(const char* path) {
         
    }

    public:
        auto getTargetCount() -> int override {
            return count;
        }

        auto getTarget(int i) -> Coord* override {
            return targets[i];
        }
         
};