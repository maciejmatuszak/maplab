#include "vi-map-data-import-export/import-export-gps-data.h"

#include <aslam/common/time.h>
#include <console-common/command-registerer.h>
#include <glog/logging.h>
#include <laser_geometry/laser_geometry.h>
#include <maplab-common/progress-bar.h>
#include <resource-importer/message-conversion.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>

#include <vi-map/vi-map.h>

namespace data_import_export {
using namespace vi_map;

void importLaserDataFromRosbag(
    const std::string& bag_filename, const std::string& laser_topic_s,
    const std::string& sensor_id_s, VIMission& mission, VIMap* map) {
  CHECK_NOTNULL(map);

  SensorId sensor_id;

  CHECK(sensor_id.fromHexString(sensor_id_s)) << " Invalid sensor id string: "
                                              << sensor_id_s;
  CHECK(map->getSensorManager().hasSensor(sensor_id))
      << " The sensor id:" << sensor_id_s << " does not exists.";

  Sensor& sensor = map->getSensorManager().getSensor(sensor_id);
  CHECK(sensor.getSensorType() == SensorType::kLidar)
      << " The sensor id:" << sensor_id_s << " is not of type lidar.";

  CHECK(
      map->getSensorManager().hasSensorMissionAssociation(
          sensor_id, mission.id()))
      << "The sensor: " << sensor_id_s
      << " is not associated with the mission: " << mission.id();

  CHECK(common::fileExists(bag_filename))
      << "Bag file does not exists or is not accessible: " << bag_filename;

  rosbag::Bag bag;

  try {
    bag.open(bag_filename, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& bag_exception) {  // NOLINT
    LOG(FATAL) << "Could not open the rosbag " << bag_filename << ": "
               << bag_exception.what();
  }

  std::vector<std::string> topics;
  topics.emplace_back(laser_topic_s);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  const size_t num_messages = view.size();
  if (num_messages == 0u) {
    LOG(ERROR) << "The bag view contains zero messages. "
               << "Selected GPS topic: " << laser_topic_s;
    return;
  }

  pose_graph::VertexIdList vList;
  map->getAllVertexIdsInMissionAlongGraph(mission.id(), &vList);
  int64_t missionStartTs =
      map->getVertex(vList.front()).getMinTimestampNanoseconds();
  int64_t missionEndTs =
      map->getVertex(vList.back()).getMinTimestampNanoseconds();

  common::ProgressBar progress_bar(num_messages);
  // we will need projector to convert laser scanns to point cloud 2
  // TODO: ass new resources to store LaserScan it can be converted to
  // point cloud better with correct trajectories i.e. after map optimisation
  laser_geometry::LaserProjection laser_projector;
  size_t num_added = 0u;
  size_t num_ignored = 0u;
  for (rosbag::MessageInstance& message : view) {
    CHECK_EQ(message.getTopic(), laser_topic_s);

    sensor_msgs::LaserScanConstPtr laser_scann_message;
    laser_scann_message = message.instantiate<sensor_msgs::LaserScan>();

    // message should be instance of laser scann
    CHECK_NOTNULL(laser_scann_message);

    const int64_t timestamp_ns = laser_scann_message->header.stamp.toNSec();
    CHECK_GE(timestamp_ns, 0);
    if (timestamp_ns < missionStartTs || timestamp_ns > missionEndTs) {
      LOG(WARNING) << "\nLidar scan outside mission timing ignore; timestamp :"
                   << timestamp_ns;
      progress_bar.increment();
      ++num_ignored;
      continue;
    }

    if (mission.hasOptionalSensorResourceId(
            backend::ResourceType::kPointCloudXYZI, sensor_id, timestamp_ns)) {
      LOG(WARNING) << "\nLidar resource already exists @ time:" << timestamp_ns;
      progress_bar.increment();
      ++num_ignored;
      continue;
    }

    // convert to point cloud2
    sensor_msgs::PointCloud2Ptr pc2Ptr =
        boost::make_shared<sensor_msgs::PointCloud2>();
    laser_projector.projectLaser(*laser_scann_message, *pc2Ptr);

    resources::PointCloud maplab_pointcloud;
    convertPointCloudMessageIntensity(pc2Ptr, &maplab_pointcloud);

    map->storeOptionalLidarPointCloudXYZI(
        sensor_id, timestamp_ns, maplab_pointcloud, &mission);
    ++num_added;
    progress_bar.increment();
  }
  LOG(INFO) << "\nImported " << num_added << " point cloud resources\n"
            << "Ignored " << num_ignored << " point cloud resources\n";
  if (num_added > 0) {
    LOG(INFO)
        << "Map metadata is not saved but resources are imported - please save";
  }
}

}  // namespace data_import_export
