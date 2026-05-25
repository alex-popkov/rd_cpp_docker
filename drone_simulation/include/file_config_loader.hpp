#pragma once

#include "interfaces/config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
    AmmoParams ammoParams;
    DroneConfig droneConfig;
    const char* droneConfigPath;
    const char* ammoConfigPath;

    public:

        FileConfigLoader(const char* droneConfigPath, const char* ammoConfigPath) {
            this->ammoConfigPath = ammoConfigPath;
            this->droneConfigPath = droneConfigPath;
        }
        
        auto load() -> void override {
            droneConfig = readDroneConfig(droneConfigPath);
            json ammoJSON = parseJSONfile(ammoConfigPath);
            AmmoParams* ammoArr = readAmmo(ammoJSON);
            ammoParams = findAmmo(ammoArr, droneConfig.ammoName, ammoJSON.size());
            delete[] ammoArr;
        };

        auto getConfig() -> DroneConfig override {
            return droneConfig;
        };

        auto getAmmoParams() -> AmmoParams override {
            return ammoParams;
        };
};