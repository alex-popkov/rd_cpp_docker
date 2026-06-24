#pragma once
#include <cstring>
#include "interfaces/config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
    AmmoParams ammoParams;
    DroneConfig droneConfig;
    const std::string droneConfigPath;
    const std::string ammoConfigPath;
    bool loaded = false; 

    public:
        FileConfigLoader(const std::string droneConfigPath, const std::string ammoConfigPath);
        
        auto load() -> void override;
        auto getConfig() -> DroneConfig override;
        auto getAmmoParams() -> AmmoParams override;
};