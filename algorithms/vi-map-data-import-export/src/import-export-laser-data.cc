#include "vi-map-data-import-export/import-export-gps-data.h"

#include <aslam/common/time.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensors/sensor-factory.h>
#include <vi-map/vi-map.h>

namespace data_import_export {
using namespace vi_map;

void importLaserDataFromRosbag(
    const std::string& bag_filename, const std::string& laser_topic_s,
    const std::string& sensor_id_s, const std::string& mission_id_s,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  SensorId sensor_id;
  MissionId mission_id;

  CHECK(sensor_id.fromHexString(sensor_id_s)) << " Invalid sensor id string: "
                                              << sensor_id_s;
  CHECK(map->getSensorManager().hasSensor(sensor_id))
      << " The sensor id:" << sensor_id_s << " does not exists.";

  CHECK(mission_id.fromHexString(mission_id_s))
      << " Invalid mission id string: " << mission_id_s;
  CHECK(map->hasMission(mission_id)) << " The mission id:" << mission_id_s
                                     << " does not exists.";

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

  common::ProgressBar progress_bar(num_messages);
  size_t num_added = 0u;
  for (rosbag::MessageInstance& message : view) {
    CHECK_EQ(message.getTopic(), laser_topic_s);
    if (!process_gps_measurement(
            message, gps_sensor_id, &optional_sensor_data)) {
      LOG(ERROR) << "Unable to process message. Aborting. Added " << num_added
                 << " GPS measurements.";
      break;
    }
    ++num_added;
    progress_bar.increment();
  }
}

}  // namespace data_import_export
