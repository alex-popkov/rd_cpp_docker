 #include <vector>
 #include "config_loaders/file_config_loader.hpp"


FileConfigLoader::FileConfigLoader(
    const std::string droneConfigPath,
    const std::string ammoConfigPath
): droneConfigPath(droneConfigPath), ammoConfigPath(ammoConfigPath) {
}

auto FileConfigLoader::load() -> void {
    if (loaded) {
        return;
    }
    droneConfig = readDroneConfig(droneConfigPath);
    json ammoJSON = parseJSONfile(ammoConfigPath);
    std::vector<AmmoParams> ammoArr = readAmmo(ammoJSON);
    ammoParams = findAmmo(ammoArr, droneConfig.ammoName);
    loaded = true;
};

auto FileConfigLoader::getConfig() -> DroneConfig {
    return droneConfig;
};

auto FileConfigLoader::getAmmoParams() -> AmmoParams {
    return ammoParams;
};