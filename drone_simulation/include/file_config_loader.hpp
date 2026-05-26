#pragma once

#include "interfaces/config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
    AmmoParams ammoParams;
    DroneConfig droneConfig;
    const char* droneConfigPath;
    const char* ammoConfigPath;
    bool loaded = false;  

    public:

        FileConfigLoader(const char* droneConfigPath, const char* ammoConfigPath): droneConfigPath(droneConfigPath), ammoConfigPath(ammoConfigPath) {
        }
        
        auto load() -> void override {
            if (loaded) {
                return;
            }
            droneConfig = readDroneConfig(droneConfigPath);
            json ammoJSON = parseJSONfile(ammoConfigPath);
            AmmoParams* ammoArr = readAmmo(ammoJSON);
            ammoParams = findAmmo(ammoArr, droneConfig.ammoName, ammoJSON.size());
            delete[] ammoArr;
            loaded = true;
        };

        auto getConfig() -> DroneConfig override {
            return droneConfig;
        };

        auto getAmmoParams() -> AmmoParams override {
            return ammoParams;
        };
};