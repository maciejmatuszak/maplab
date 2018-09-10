#include <string>
#include <vector>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensors/lidar.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map-serialization.h>

#include "resource-importer/message-conversion.h"
#include "resource-importer/simple-rosbag-reader.h"

using namespace vi_map;
DEFINE_string(map_path, "", "Map input path.");
DEFINE_string(
    mission_id, "",
    "Id of mission to attach resources to. This flag is optional if map only "
    "contains one mission.");

DEFINE_string(map_output_path, "", "Map output path.");

DEFINE_string(rosbag_path, "", "Rosbag path.");

DEFINE_string(resource_topic, "", "ROS topic of resources to import.");

DEFINE_string(
    lidar_config_file, "",
    "Path to Lidar YAML file to load sensor info with extrinsics.");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Check flags.
  CHECK(!FLAGS_map_path.empty()) << "You have to provide a maplab map to "
                                 << "attach the resources to! Flag: --map_path";
  if (FLAGS_map_output_path.empty()) {
    LOG(WARNING) << "You have not provided an output map, the resources will "
                 << "be attached to the input map, otherwise abort and use "
                 << "this flag: --map_output_path";
    FLAGS_map_output_path = FLAGS_map_path;
  }

  CHECK(!FLAGS_resource_topic.empty())
      << "You have to provide a rostopic for the camera resource!";

  // Get VIMap.
  vi_map::VIMap map;
  const bool result =
      vi_map::serialization::loadMapFromFolder(FLAGS_map_path, &map);
  CHECK(result) << "Map loading failed.";

  // Set the resource folder to an external folder, such that adding resources
  // to this map does not pollute the map folder.
  const std::string kTemporaryResourceFolder = "/tmp/vi_map_resources";
  // clean the folder
  common::removePath(kTemporaryResourceFolder);
  map.useExternalResourceFolder(kTemporaryResourceFolder);

  // Get the mission that we want to add the resource to.
  CHECK_GT(map.numMissions(), 0u);
  vi_map::MissionIdList mission_ids;
  map.getAllMissionIds(&mission_ids);
  vi_map::MissionId selected_mission_id;
  if (mission_ids.size() > 1u) {
    CHECK(!FLAGS_mission_id.empty())
        << "If the map contains more than one mission, you need to provide the "
        << "mission id of the mission you want to add the resources to! Flag: "
        << "--mission_id";
    CHECK(selected_mission_id.fromHexString(FLAGS_mission_id))
        << "Invalid mission id: " << FLAGS_mission_id;
    CHECK(map.hasMission(selected_mission_id))
        << "This map does not contain a mission with id: " << FLAGS_mission_id;
  } else {
    CHECK_EQ(mission_ids.size(), 1u);
    selected_mission_id = mission_ids[0];
  }
  vi_map::VIMission& selected_mission = map.getMission(selected_mission_id);

  // add lidar sensor
  vi_map::Lidar::UniquePtr lidar;
  vi_map::SensorId sensor_id;
  common::generateId(&sensor_id);
  lidar.reset(new vi_map::Lidar(sensor_id, "/Hokuyo"));
  CHECK(lidar->getId().isValid());

  // add the lidar sensor
  map.getSensorManager().addSensor(std::move(lidar), selected_mission_id);
  aslam::Transformation tr;
  if (!map.getSensorManager().hasSensorSystem()) {
    SensorIdSet imu_sensors_ids;
    map.getSensorManager().getAllSensorIdsOfTypeAssociatedWithMission(
        SensorType::kImu, selected_mission_id, &imu_sensors_ids);
    // only supporting missions with on imu, a flag can ba added to select
    // reference sensor if required
    CHECK_EQ(imu_sensors_ids.size(), 1u);
    SensorId refImuId = *(imu_sensors_ids.begin());
    SensorSystem::UniquePtr ss(new SensorSystem(refImuId));
    map.getSensorManager().addSensorSystem(std::move(ss));
  }
  map.getSensorManager().setSensor_T_R_S(sensor_id, tr);

  // setup rosbag source

  SimpleRosbagSource rosbag_source(
      FLAGS_rosbag_path, FLAGS_resource_topic, "",
      "", "");

  std::function<void(sensor_msgs::PointCloud2ConstPtr)> pointcloud_callback =
      [&](sensor_msgs::PointCloud2ConstPtr point_cloud_msg) {
        CHECK(point_cloud_msg);
        const int64_t timestamp_ns = point_cloud_msg->header.stamp.toNSec();
        CHECK_GE(timestamp_ns, 0);

        LOG(INFO) << "Found Pointcloud at " << timestamp_ns << "ns";

        resources::PointCloud maplab_pointcloud;
        convertPointCloudMessage(point_cloud_msg, &maplab_pointcloud);

        map.storeOptionalLidarPointCloudXYZI();
            sensor_id, timestamp_ns, maplab_pointcloud, &selected_mission);
      };
  rosbag_source.setPointcloudCallback(pointcloud_callback);

  rosbag_source.readRosbag();

  // we done - save the bag
  backend::SaveConfig save_config;
  save_config.overwrite_existing_files = true;
  save_config.move_resources_when_migrating = false;
  save_config.migrate_resources_settings = backend::SaveConfig::
      MigrateResourcesSettings::kMigrateResourcesToMapFolder;
  const bool write_map_result = vi_map::serialization::saveMapToFolder(
      FLAGS_map_output_path, save_config, &map);

  CHECK(write_map_result) << "Saving the output map to the file system failed.";

  CHECK(vi_map::checkMapConsistency(map));

  return 0;
}
