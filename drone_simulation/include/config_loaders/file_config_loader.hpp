#pragma once
#include "interfaces/config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
    AmmoParams ammoParams;
    DroneConfig droneConfig;
    const char* droneConfigPath;
    const char* ammoConfigPath;
    bool loaded = false; 

    public:
        FileConfigLoader(const char* droneConfigPath, const char* ammoConfigPath);
        
        auto load() -> void override;
        auto getConfig() -> DroneConfig override;
        auto getAmmoParams() -> AmmoParams override;
};