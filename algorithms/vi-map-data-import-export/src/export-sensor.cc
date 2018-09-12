#include "vi-map-data-import-export/export-sensor.h"


#include <aslam/common/unique-id.h>
#include <console-common/command-registerer.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>

namespace data_import_export {
using namespace vi_map;


int exportSensor(const vi_map::VIMap &map, const std::string &sensor_file, const std::string &sensor_id_str)
{
    const vi_map::SensorManager& sensor_manager = map.getSensorManager();
    SensorId sensor_id;
    sensor_id.fromHexString(sensor_id_str);
    if(!sensor_manager.hasSensor(sensor_id))
    {
        LOG(ERROR) << "There is no sensor with Id:"
                   << sensor_id;
        return common::kStupidUserError;
    }

    YAML::Node yaml_node;
    YAML::Node yaml_node_sensor;
    YAML::Node yaml_node_extr;
    yaml_node["sensor"] = yaml_node_sensor;
    yaml_node["extrinsics"] = yaml_node_extr;


    const Sensor &sensor = sensor_manager.getSensor(sensor_id);
    sensor.serialize(&yaml_node_sensor);
    LOG(INFO) << "Sensor info exported successfully; Sensor ID: " << sensor_id.hexString();

    aslam::Transformation T_R_S;
    if(sensor_manager.getSensor_T_R_S(sensor_id, &T_R_S))
    {
        Extrinsics ex(T_R_S);
        ex.serialize(&yaml_node_extr);
        LOG(INFO) << "Sensor extrinsics exported successfully; Sensor ID: " << sensor_id.hexString();
    }

    CHECK(yaml_node.IsDefined());

    std::ofstream output_file_stream(sensor_file);
    CHECK(output_file_stream.is_open()) << "Failed to open file " << sensor_file
                                        << " for writing.";
    output_file_stream << yaml_node;
    output_file_stream.close();
    LOG(INFO) << "Sensor export successfyll; File:" << sensor_file;
    return common::kSuccess;
}

int importSensor(const VIMap &map, const std::string &sensor_file, const std::string &mission_id_str)
{
    const vi_map::SensorManager& sensor_manager = map.getSensorManager();

    MissionIdSet missionIdsToLink;
    if(mission_id_str.empty())
    {
        LOG(INFO) << "Imported sensor will be linked to all missions";
        map.getAllMissionIds(&missionIdsToLink);
    }
    else
    {
        MissionId selected_mission_id;
        CHECK(selected_mission_id.fromHexString(mission_id_str))
            << "Invalid mission id: " << mission_id_str;
        CHECK(map.hasMission(selected_mission_id))
            << "This map does not contain a mission with id: " << selected_mission_id.hexString();
        missionIdsToLink.emplace(selected_mission_id);
        LOG(INFO) << "Imported sensor will be linked to mission: " << selected_mission_id.hexString();
    }

    YAML::Node yaml_node = YAML::LoadFile(sensor_file.c_str());
    return common::kSuccess;
}

}  // namespace data_import_export
