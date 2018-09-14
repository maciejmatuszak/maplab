#ifndef VI_MAP_DATA_IMPORT_EXPORT_IMPORT_EXPORT_LASER_DATA_H_
#define VI_MAP_DATA_IMPORT_EXPORT_IMPORT_EXPORT_LASER_DATA_H_

#include <string>
#include <vector>

#include <vi-map/vi-map.h>

namespace data_import_export {
using namespace vi_map;

void importLaserDataFromRosbag(
        const std::string& bag_filename, const std::string& laser_topic_s,
        const std::string& sensor_id_s, VIMission &mission,
        VIMap* map);

}  // namespace data_import_export


#endif  // VI_MAP_DATA_IMPORT_EXPORT_IMPORT_EXPORT_LASER_DATA_H_
