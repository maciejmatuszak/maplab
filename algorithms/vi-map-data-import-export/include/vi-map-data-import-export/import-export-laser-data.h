#ifndef VI_MAP_DATA_IMPORT_EXPORT_IMPORT_EXPORT_LASER_DATA_H_
#define VI_MAP_DATA_IMPORT_EXPORT_IMPORT_EXPORT_LASER_DATA_H_

#include <string>
#include <vector>

#include <vi-map/vi-map.h>

namespace data_import_export {

void importLaserDataFromRosbag(
        const std::string& bag_filename, const std::string& laser_topic_s,
        const std::string& sensor_id_s, const std::string& mission_id_s,
        vi_map::VIMap* map);

}  // namespace data_import_export


#endif  // VI_MAP_DATA_IMPORT_EXPORT_IMPORT_EXPORT_LASER_DATA_H_
