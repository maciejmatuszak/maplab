#ifndef VI_MAP_DATA_IMPORT_EXPORT_EXPORT_SENSOR_H_
#define VI_MAP_DATA_IMPORT_EXPORT_EXPORT_SENSOR_H_

#include <string>

#include <vi-map/vi-map.h>

namespace data_import_export {

int exportSensor(
    const vi_map::VIMap& map, const std::string& sensor_file, const std::string& sensor_id_str);
int importSensor(
    vi_map::VIMap& map, const std::string& sensor_file, const std::string& mission_id_str);

}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_EXPORT_SENSOR_H_
