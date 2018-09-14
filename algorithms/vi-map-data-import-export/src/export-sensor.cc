#include "vi-map-data-import-export/export-sensor.h"

#include <aslam/common/unique-id.h>
#include <console-common/command-registerer.h>
#include <maplab-common/file-system-tools.h>
#include <sensors/sensor-extrinsics.h>
#include <sensors/sensor-factory.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>

namespace data_import_export {
constexpr char kYamlFieldNameSensor[] = "sensor";
constexpr char kYamlFieldNameExtrinsics[] = "extrinsics";
using namespace vi_map;

int exportSensor(
    const vi_map::VIMap& map, const std::string& sensor_file,
    const std::string& sensor_id_str) {
  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  SensorId sensor_id;
  sensor_id.fromHexString(sensor_id_str);
  CHECK(sensor_id.isValid());
  if (!sensor_manager.hasSensor(sensor_id)) {
    LOG(ERROR) << "There is no sensor with Id:" << sensor_id.hexString()
               << " in map: " << map.getMapFolder();
    return common::kStupidUserError;
  }

  // create nodes for sensor and extrinsics
  YAML::Node yaml_node;
  YAML::Node yaml_node_sensor;
  YAML::Node yaml_node_extr;
  yaml_node[static_cast<std::string>(kYamlFieldNameSensor)] = yaml_node_sensor;
  yaml_node[static_cast<std::string>(kYamlFieldNameExtrinsics)] =
      yaml_node_extr;

  // export sensor
  const Sensor& sensor = sensor_manager.getSensor(sensor_id);
  CHECK(sensor.isValid());
  sensor.serialize(&yaml_node_sensor);
  LOG(INFO) << "Sensor info exported successfully; Sensor ID: "
            << sensor_id.hexString();

  // export extrinsics
  aslam::Transformation T_R_S;
  if (sensor_manager.getSensor_T_R_S(sensor_id, &T_R_S)) {
    Extrinsics ex(T_R_S);
    ex.serialize(&yaml_node_extr);
    LOG(INFO) << "Sensor extrinsics exported successfully; Sensor ID: "
              << sensor_id.hexString();
  }

  // save to file
  std::ofstream output_file_stream(sensor_file);
  CHECK(output_file_stream.is_open()) << "Failed to open file " << sensor_file
                                      << " for writing.";
  output_file_stream << yaml_node;
  output_file_stream.close();
  LOG(INFO) << "Sensor export successfyll; File:" << sensor_file;
  return common::kSuccess;
}

int importSensor(
    VIMap& map, const std::string& sensor_file,
    const std::string& mission_id_str) {
  MissionIdList missionIdsToLink;
  if (mission_id_str.empty()) {
    LOG(INFO) << "Imported sensor will be linked to all missions";
    map.getAllMissionIds(&missionIdsToLink);
  } else {
    MissionId selected_mission_id;
    CHECK(selected_mission_id.fromHexString(mission_id_str))
        << "Invalid mission id: " << mission_id_str;
    CHECK(map.hasMission(selected_mission_id))
        << "This map does not contain a mission with id: "
        << selected_mission_id.hexString();
    missionIdsToLink.push_back(selected_mission_id);
    LOG(INFO) << "Imported sensor will be linked to mission: "
              << selected_mission_id.hexString();
  }

  // load node from file
  YAML::Node yaml_node = YAML::LoadFile(sensor_file.c_str());
  CHECK(yaml_node.IsMap()) << "Invalid structure of " << sensor_file;

  // get sensor object from yaml
  const YAML::Node& yaml_node_sensor =
      yaml_node[static_cast<std::string>(kYamlFieldNameSensor)];
  CHECK(yaml_node_sensor) << "missing sensor node in file: " << sensor_file;
  vi_map::Sensor::UniquePtr sensorPtr =
      vi_map::createSensorFromYaml(yaml_node_sensor);
  CHECK(sensorPtr) << " Unable to create sensor from yaml";
  SensorId sensor_id = sensorPtr->getId();
  CHECK(sensorPtr->isValid()) << "invalid sensor node in file: " << sensor_file;

  SensorManager& sm = map.getSensorManager();

  // check if the sensor exists or create it
  if (sm.hasSensor(sensorPtr->getId())) {
    LOG(INFO) << "sensor id: " << sensorPtr->getId().hexString()
              << " already exists ";
    const Sensor& existing_sensor = sm.getSensor(sensorPtr->getId());
    CHECK(existing_sensor == *sensorPtr)
        << "Imported sensor and existing sensors are different !";
  } else {
    LOG(INFO) << "adding sensor id: " << sensorPtr->getId().hexString();
    sm.addSensor(std::move(sensorPtr), missionIdsToLink[0u]);
  }
  // link the sensor to all selected missions
  for (size_t idx = 0u; idx < missionIdsToLink.size(); ++idx) {
    const MissionId& mission_id = missionIdsToLink[idx];
    if (!sm.hasSensorMissionAssociation(sensor_id, mission_id)) {
      sm.associateExistingSensorWithMission(sensor_id, mission_id);
    }
  }

  // now for the extrinsics
  const YAML::Node& yaml_node_extr =
      yaml_node[static_cast<std::string>(kYamlFieldNameExtrinsics)];
  Extrinsics::UniquePtr exPtr = Extrinsics::createFromYaml(yaml_node_extr);

  if (exPtr) {
    if (!sm.hasSensorSystem()) {
      // we do not have sensor system
      // we need reference sensor - lets pick first imu
      SensorIdSet imu_sensors_ids;
      sm.getAllSensorIdsOfTypeAssociatedWithMission(
          SensorType::kImu, missionIdsToLink[0], &imu_sensors_ids);
      // only supporting missions with on imu, a flag can ba added to select
      // reference sensor if required
      CHECK_EQ(imu_sensors_ids.size(), 1u);
      SensorId refImuId = *(imu_sensors_ids.begin());
      SensorSystem::UniquePtr ss(new SensorSystem(refImuId));
      sm.addSensorSystem(std::move(ss));
    }
    // now we have sensor system we can set extrinsics
    sm.setSensor_T_R_S(sensor_id, exPtr->get_T_R_S());
  }

  return common::kSuccess;
}

}  // namespace data_import_export
