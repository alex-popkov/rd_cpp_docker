 #include "config_loaders/file_config_loader.hpp"


FileConfigLoader::FileConfigLoader(
    const char* droneConfigPath,
    const char* ammoConfigPath
): droneConfigPath(droneConfigPath), ammoConfigPath(ammoConfigPath) {
}

auto FileConfigLoader::load() -> void {
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

auto FileConfigLoader::getConfig() -> DroneConfig {
    return droneConfig;
};

auto FileConfigLoader::getAmmoParams() -> AmmoParams {
    return ammoParams;
};