#include "dense-reconstruction/dense-reconstruction-plugin.h"

#include <cstring>
#include <string>
#include <vector>
#include <unordered_map>

#include <console-common/console.h>
#include <dense-reconstruction/conversion-tools.h>
#include <dense-reconstruction/pmvs-file-utils.h>
#include <dense-reconstruction/pmvs-interface.h>
#include <dense-reconstruction/stereo-dense-reconstruction.h>
#include <gflags/gflags.h>
#include <map-manager/map-manager.h>
#include <maplab-common/file-system-tools.h>
#include <map-resources/resource-common.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>
#include <maplab-common/sigint-breaker.h>
#include <visualization/viwls-graph-plotter.h>
#include <voxblox-interface/integration.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <visualization/rviz-visualization-sink.h>


DECLARE_string(map_mission_list);
DECLARE_bool(overwrite);

DEFINE_bool(
    dense_use_distorted_camera, false,
    "If enabled, the depth map reprojection assumes that the map has "
    "been created using the distorted image. Therefore, the distorted "
    "camera model is used for reprojection.");

DEFINE_string(
    dense_result_mesh_output_file, "",
    "Path to the PLY mesh file that is generated from the "
    "reconstruction command.");

DEFINE_double(
    dense_tsdf_voxel_size_m, 0.02, "Voxel size of the TSDF grid [m].");

DEFINE_uint64(
    dense_tsdf_voxels_per_side, 16u,
    "Voxels per side of a Block of the TSDF grid.");

DEFINE_bool(
    dense_tsdf_voxel_carving_enabled, true,
    "If true, the entire length of a ray is integrated, if false only the region inside the trunaction distance is used.");

DEFINE_bool(
    dense_tsdf_voxel_use_clearing_rays, true,
    "If enabled it will ray-trace points that are beyond the maxium ray length "
    "defined by --dense_tsdf_max_ray_length_m up to the threshold to clear "
    "free space. This option is intended to create maps for path planning.");

DEFINE_bool(
    dense_tsdf_voxel_use_const_weight, false,
    "If enabled the point cloud integration will not use weighting based on distance, "
    "this is usefull especially for laser point cloud integration.");

DEFINE_double(
    dense_tsdf_truncation_distance_m, 0.1,
    "Truncation distance of the TSDF grid [m].");

DEFINE_double(
    dense_tsdf_min_ray_length_m, 0.05,
    "Minimum ray length integrated into the TSDF grid.");

DEFINE_double(
    dense_tsdf_max_ray_length_m, 20.,
    "Maximum ray length integrated into the TSDF grid.");

DEFINE_string(
    dense_image_export_path, "",
    "Export folder for image export function. console command: "
    "export_timestamped_images");

DEFINE_int32(
    dense_depth_resource_output_type, 17,
    "Output resource type of the dense reconstruction algorithms."
    "Supported commands: "
    "stereo_dense_reconstruction "
    "Supported types: "
    "PointCloudXYZRGBN = 17, RawDepthMap = 8");

DEFINE_int32(
    dense_depth_resource_input_type, 17,
    "Input resource type of the dense reconstruction algorithms."
    "Supported commands: "
    "create_tsdf_from_depth_resource "
    "Supported types: "
    "RawDepthMap = 8, OptimizedDepthMap = 9, PointCloudXYZ = 16, "
    "PointCloudXYZRGBN = 17, kPointCloudXYZI = 21");

namespace dense_reconstruction {

bool parseMultipleMissionIds(
    const vi_map::VIMap& vi_map, vi_map::MissionIdList* mission_ids) {
  CHECK_NOTNULL(mission_ids);
  if (!vi_map::csvIdStringToIdList(FLAGS_map_mission_list, mission_ids)) {
    LOG(ERROR) << "The provided CSV mission id list is not valid!";
    return false;
  }

  if (mission_ids->empty()) {
    LOG(INFO) << "No mission id list was provided, operating on all missions!";
    return true;
  }

  LOG(INFO) << "Compute depth maps from multi view stereo for the "
               "following missions:";
  bool success = true;
  for (const vi_map::MissionId& mission_id : *mission_ids) {
    if (mission_id.isValid() && vi_map.hasMission(mission_id)) {
      LOG(INFO) << "-> " << mission_id;
    } else {
      LOG(ERROR) << "-> " << mission_id
                 << " does not exist in the selected map!";
      success = false;
    }
  }
  return success;
}

common::CommandStatus exportTsdfMeshToFile(
    const std::string& mesh_file_path, voxblox::TsdfMap* tsdf_map) {
  CHECK_NOTNULL(tsdf_map);

  if (mesh_file_path.empty()) {
    LOG(ERROR) << "No mesh output path specified, please set "
                  "--dense_result_mesh_output_file .";
    return common::kStupidUserError;
  }

  if (!common::createPathToFile(mesh_file_path)) {
    LOG(ERROR) << "Unable to create a path for the mesh output file: "
               << mesh_file_path;
    return common::kStupidUserError;
  }

  if (common::fileExists(mesh_file_path)) {
    LOG(ERROR) << "Output mesh file already exists: " << mesh_file_path;
    return common::kStupidUserError;
  }

  voxblox::MeshLayer mesh_layer(tsdf_map->block_size());

  voxblox::MeshIntegratorConfig mesh_config;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_config, tsdf_map->getTsdfLayerPtr(), &mesh_layer);
  // We mesh the whole grid at once anyways, so all of them should be
  // updated.
  constexpr bool kMeshOnlyUpdatedBlocks = false;
  // No need to reset, we are not gonna mesh again.
  constexpr bool kResetUpdatedFlag = false;
  mesh_integrator.generateMesh(kMeshOnlyUpdatedBlocks, kResetUpdatedFlag);

  voxblox::outputMeshLayerAsPly(mesh_file_path, mesh_layer);
  return common::kSuccess;
}

DenseReconstructionPlugin::DenseReconstructionPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"export_timestamped_images"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (FLAGS_dense_image_export_path.empty()) {
          LOG(ERROR) << "Please define the export path with "
                        "--dense_image_export_path!";
          return common::kStupidUserError;
        }

        if (!dense_reconstruction::exportAllImagesForCalibration(
                FLAGS_dense_image_export_path, map.get())) {
          return common::kUnknownError;
        }
        return common::kSuccess;
      },
      "Export timestamped image resources, such that they can be used for "
      "calibration. Use --dense_image_export_path to set the export path.",
      common::Processing::Sync);

  addCommand(
      {"export_for_pmvs_reconstruction", "export_for_pmvs"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapReadAccess map =
            map_manager.getMapReadAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        const dense_reconstruction::PmvsConfig config =
            dense_reconstruction::PmvsConfig::getFromGflags();
        if (mission_ids.empty()) {
          if (!dense_reconstruction::exportVIMapToPmvsSfmInputData(
                  config, *map)) {
            return common::kUnknownError;
          }
        } else {
          if (!dense_reconstruction::exportVIMapToPmvsSfmInputData(
                  config, mission_ids, *map)) {
            return common::kUnknownError;
          }
        }
        return common::kSuccess;
      },
      "Export the map and the associated image resources to the PMVS/CMVS "
      "input format, such that we can reconstruct the whole map.",
      common::Processing::Sync);

  addCommand(
      {"stereo_dense_reconstruction", "stereo_dense", "sdr"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        const backend::ResourceType output_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_output_type);

        if (mission_ids.empty()) {
          dense_reconstruction::computeDepthForAllStereoCameras(
              output_resource_type, map.get());
        } else {
          dense_reconstruction::computeDepthForAllStereoCameras(
              output_resource_type, mission_ids, map.get());
        }
        return common::kSuccess;
      },
      "Uses OpenCvs stereo matcher to compute depth resources for all stereo "
      "cameras in the map (or all selected missions). Use the "
      "--dense_stereo_* "
      "flags for configuration of the stereo matcher. Currently only the "
      "pinhole camera model is supported. The depth output type can be set "
      "using --dense_depth_resource_output_type",
      common::Processing::Sync);

  addCommand(
      {"convert_all_depth_maps_to_point_clouds"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (!dense_reconstruction::convertAllDepthMapsToPointClouds(
                map.get())) {
          return common::kUnknownError;
        }
        return common::kSuccess;
      },
      "Convert all depth maps into point clouds (which are stored as PLYs).",
      common::Processing::Sync);

  addCommand(
      {"create_tsdf_from_depth_resource", "tsdf", "depth_fusion"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        voxblox::TsdfIntegratorBase::Config tsdf_integrator_config;

        tsdf_integrator_config.voxel_carving_enabled =
            FLAGS_dense_tsdf_voxel_carving_enabled;
        tsdf_integrator_config.allow_clear =
            FLAGS_dense_tsdf_voxel_use_clearing_rays;
        tsdf_integrator_config.default_truncation_distance =
            static_cast<float>(FLAGS_dense_tsdf_truncation_distance_m);
        tsdf_integrator_config.min_ray_length_m =
            static_cast<voxblox::FloatingPoint>(
                FLAGS_dense_tsdf_min_ray_length_m);
        tsdf_integrator_config.max_ray_length_m =
            static_cast<voxblox::FloatingPoint>(
                FLAGS_dense_tsdf_max_ray_length_m);
        tsdf_integrator_config.use_const_weight =
                FLAGS_dense_tsdf_voxel_use_const_weight;

        voxblox::TsdfMap::Config tsdf_map_config;
        tsdf_map_config.tsdf_voxel_size =
            static_cast<voxblox::FloatingPoint>(FLAGS_dense_tsdf_voxel_size_m);
        tsdf_map_config.tsdf_voxels_per_side = FLAGS_dense_tsdf_voxels_per_side;

            voxblox::TsdfMap tsdf_map(tsdf_map_config);

        const backend::ResourceType input_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_input_type);

        if (!voxblox_interface::integrateAllDepthResourcesOfType(
                mission_ids, input_resource_type,
                !FLAGS_dense_use_distorted_camera, tsdf_integrator_config, *map,
                &tsdf_map)) {
          LOG(ERROR) << "Unable to compute Voxblox TSDF grid.";
          return common::kStupidUserError;
        }

        const bool has_resource = map->hasVoxbloxTsdfMap(mission_ids);
        if (has_resource && FLAGS_overwrite) {
          map->replaceVoxbloxTsdfMap(mission_ids, tsdf_map);
        } else if (has_resource && !FLAGS_overwrite) {
          LOG(ERROR)
              << "Could not store the Voxblox TSDF map, because there is "
              << "already a map stored. Use --overwrite!";
          return common::kStupidUserError;
        } else {
          map->storeVoxbloxTsdfMap(tsdf_map, mission_ids);
        }

        constexpr double kBytesToMegaBytes = 1e-6;
        LOG(INFO) << "TSDF map:";
        LOG(INFO) << "  allocated blocks: "
                  << tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks();
        LOG(INFO) << "  size: "
                  << tsdf_map.getTsdfLayer().getMemorySize() * kBytesToMegaBytes
                  << "MB";

        if (!FLAGS_dense_result_mesh_output_file.empty()) {
                      LOG(INFO) << "Creating mesh file: " << FLAGS_dense_result_mesh_output_file;
          return exportTsdfMeshToFile(
              FLAGS_dense_result_mesh_output_file, &tsdf_map);
        }
        else
        {
          LOG(INFO) << "Mesh file will not be created: ";
        }
        return common::kSuccess;
      },
      "Use all depth resources the selected missions "
      "and integrate them into a Voxblox TSDF map. The map is then stored as "
      "resource associated with the selected set of missions. This command "
      "will use the resource type specified by "
      "--dense_depth_resource_input_type if available.",
      common::Processing::Sync);

  addCommand(
      {"publish_point_clouds", "ppcs"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        const vi_map::VIMapManager::MapReadAccess map =
            map_manager.getMapReadAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }
        const backend::ResourceType input_resource_type =
          static_cast<backend::ResourceType>(
              FLAGS_dense_depth_resource_input_type);

        // If no mission were selected, use all missions.
        if (mission_ids.empty())
        {
            map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }
        for (const vi_map::MissionId& mission_id : mission_ids)
        {
            const aslam::Transformation& T_G_M =
            map.get()->getMissionBaseFrameForMission(mission_id).get_T_G_M();
            const landmark_triangulation::PoseInterpolator pose_interpolator;
            landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
            int64_t min_timestamp_ns;
            int64_t max_timestamp_ns;
            pose_interpolator.getVertexToTimeStampMap(
                *map, mission_id, &vertex_to_time_map, 
                &min_timestamp_ns,
                &max_timestamp_ns);
            const vi_map::VIMission &mission = map.get()->getMission(mission_id);
            typedef std::unordered_map<vi_map::SensorId,backend::OptionalSensorResources> SensorsToResourceMap;
            const SensorsToResourceMap* sensor_id_to_res_id_map;

            LOG(INFO) << "Getting all resources for mission: " << mission_id.shortHex()
                    << "; of type:" << static_cast<int>(input_resource_type) << ";";
            sensor_id_to_res_id_map = mission.getAllOptionalSensorResourceIdsOfType<vi_map::SensorId>(
                              input_resource_type);

            if(sensor_id_to_res_id_map == NULL || sensor_id_to_res_id_map->empty())
            {
                LOG(INFO) << "In mission mission: " << mission_id.shortHex()
                                  << " there is no resources of type:" << static_cast<int>(input_resource_type) << ";";
                continue;
            }
            LOG(INFO) << "Got : " << sensor_id_to_res_id_map->size() << "; resources;";
            common::SigintBreaker sigint_breaker;
            for (const typename SensorsToResourceMap::value_type& sensor_to_res_ids :
                           *sensor_id_to_res_id_map)
            {
                const vi_map::SensorId& sensor_id = sensor_to_res_ids.first;
                const backend::OptionalSensorResources& resource_buffer =
                              sensor_to_res_ids.second;

                // Get transformation between reference (e.g. IMU) and sensor.
                aslam::Transformation T_I_S;
                map.get()->getSensorManager().getSensorOrCamera_T_R_S(
                              sensor_id, &T_I_S);
                const vi_map::Sensor& sensor = map.get()->getSensorManager().getSensor(sensor_id);
                const size_t num_resources = resource_buffer.size();
                LOG(INFO) << "Sensor " << sensor_id.shortHex() << " has "
                                  << num_resources << " such resources.";
                // Collect all timestamps that need to be interpolated.
                Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(num_resources);
                size_t idx = 0u;
                for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id : resource_buffer) {
                    // If the resource timestamp does not lie within the min and max
                    // timestamp of the vertices, we cannot interpolate the position. To
                    // keep this efficient, we simply replace timestamps outside the range
                    // with the min or max. Since their transformation will not be used
                    // later, that's fine.
                    resource_timestamps[idx] = std::max(
                        min_timestamp_ns,
                        std::min(
                            max_timestamp_ns,
                            stamped_resource_id.first));
                    ++idx;

                }
                // Interpolate poses at resource timestamp.
                aslam::TransformationVector poses_M_I;
                pose_interpolator.getPosesAtTime(
                *map, mission_id, resource_timestamps, &poses_M_I);

                CHECK_EQ(static_cast<int>(poses_M_I.size()), resource_timestamps.size());
                CHECK_EQ(poses_M_I.size(), resource_buffer.size());
                // Retrieve and integrate all resources.
                idx = 0u;
                common::ProgressBar tsdf_progress_bar(resource_buffer.size());
                for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
                   resource_buffer) {
                    tsdf_progress_bar.increment();

                    // We assume the frame of reference for the sensor system is the IMU
                    // frame.
                    const aslam::Transformation& T_M_I = poses_M_I[idx];
                    const aslam::Transformation T_G_S = T_G_M * T_M_I * T_I_S;
                    ++idx;

                    // we pretend it is happening now
                    ros::Time time = ros::Time::now();

                    visualization::publishTF(
                        T_G_M,
                        visualization::kDefaultMapFrame,
                        visualization::kDefaultMissionFrame,
                        time);
                    visualization::publishTF(
                        T_M_I,
                        visualization::kDefaultMissionFrame,
                        visualization::kDefaultImuFrame,
                        time);
                    visualization::publishTF(
                        T_I_S,
                        visualization::kDefaultImuFrame,
                        sensor.getHardwareId(),
                        time);
                    const int64_t timestamp_ns = stamped_resource_id.first;

                    // If the resource timestamp does not lie within the min and max
                    // timestamp of the vertices, we cannot interpolate the position.
                    if (timestamp_ns < min_timestamp_ns ||
                        timestamp_ns > max_timestamp_ns) {
                      LOG(WARNING) << "The optional depth resource at " << timestamp_ns
                                   << " is outside of the time range of the pose graph, "
                                   << "skipping.";
                      continue;
                    }
                    resources::PointCloud point_cloud;
                    if (!map.get()->getOptionalSensorResource(
                          mission, input_resource_type, sensor_id,
                          timestamp_ns, &point_cloud)) {
                          LOG(FATAL) << "Cannot retrieve optional point cloud resources at "
                               << "timestamp " << timestamp_ns << "!";
                    }
                    sensor_msgs::PointCloud2 pc2;
                    pc2.header.stamp = time;
                    pc2.header.frame_id = sensor.getHardwareId();
                    CHECK(backend::convertPointCloudType(point_cloud, &pc2));
                    visualization::RVizVisualizationSink::publish<sensor_msgs::PointCloud2>(visualization::ViwlsGraphRvizPlotter::kResourcePcTopic, pc2);
                    if(sigint_breaker.isBreakRequested())
                    {
                        LOG(INFO) << "Uaser requested break (CTRL^C)";
                        break;
                    }
                    usleep(10000);


                }
            }

        }



        return common::kSuccess;
      },
      "Use all optional resources point clouds for the selected missions "
      "and publish them.",
      common::Processing::Sync);
  addCommand(
      {"create_mesh_from_tsdf_grid", "export_tsdf"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        voxblox::TsdfMap::Config tsdf_map_config;
        voxblox::TsdfMap tsdf_map(tsdf_map_config);
        if (!map.get()->getVoxbloxTsdfMap(mission_ids, &tsdf_map)) {
          LOG(ERROR)
              << "No Voxblox TSDF grid stored for the selected missions!";
          return common::kStupidUserError;
        }

        return exportTsdfMeshToFile(
            FLAGS_dense_result_mesh_output_file, &tsdf_map);
      },
      "Compute mesh of the Voxblox TSDF grid resource associated with "
      "the selected missions.",
      common::Processing::Sync);
}

}  // namespace dense_reconstruction

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    dense_reconstruction::DenseReconstructionPlugin);
