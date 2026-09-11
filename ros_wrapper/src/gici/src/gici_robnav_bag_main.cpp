/**
* @Function: Direct rosbag runner for the RobNav RTK/IMU/LiDAR RRR profile
*
* Copyright (C) 2026
**/
#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "gici/fusion/multisensor_estimating.h"
#include "gici/ros_interface/ros_node_handle.h"
#include "gici/utility/node_option_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"

namespace {

struct Arguments {
  std::string config;
  std::string sensor_bag;
  std::string rover_bag;
  std::string reference_bag;
  std::string ephemeris_bag;
  std::string imu_topic;
  std::string lidar_topic;
  std::string rover_observations_topic;
  std::string reference_observations_topic;
  std::string ephemerides_topic;
  double base_ecef_x = 0.0;
  double base_ecef_y = 0.0;
  double base_ecef_z = 0.0;
  double start_time_s = 0.0;
  double duration_s = 0.0;
};

void printUsage(const char* executable)
{
  std::cerr << "Usage: " << executable << " <config.yaml> <sensor.bag> <rover.bag> "
            << "<reference.bag> <ephemeris.bag> <imu-topic> <lidar-topic> "
            << "<rover-observations-topic> <reference-observations-topic> "
            << "<ephemerides-topic> <base-ecef-x> <base-ecef-y> <base-ecef-z> "
            << "<start-time-s> <duration-s>" << std::endl;
}

bool parseArguments(int argc, char** argv, Arguments* arguments)
{
  if (argc != 16) return false;
  try {
    arguments->config = argv[1];
    arguments->sensor_bag = argv[2];
    arguments->rover_bag = argv[3];
    arguments->reference_bag = argv[4];
    arguments->ephemeris_bag = argv[5];
    arguments->imu_topic = argv[6];
    arguments->lidar_topic = argv[7];
    arguments->rover_observations_topic = argv[8];
    arguments->reference_observations_topic = argv[9];
    arguments->ephemerides_topic = argv[10];
    arguments->base_ecef_x = std::stod(argv[11]);
    arguments->base_ecef_y = std::stod(argv[12]);
    arguments->base_ecef_z = std::stod(argv[13]);
    arguments->start_time_s = std::stod(argv[14]);
    arguments->duration_s = std::stod(argv[15]);
  }
  catch (const std::exception&) {
    return false;
  }
  return arguments->duration_s > 0.0;
}

ros::Time timeFromSec(double seconds)
{
  ros::Time timestamp;
  timestamp.fromSec(seconds);
  return timestamp;
}

}  // namespace

int main(int argc, char** argv)
{
  Arguments arguments;
  if (!parseArguments(argc, argv, &arguments)) {
    printUsage(argv[0]);
    return 2;
  }

  ros::init(argc, argv, "gici_robnav_bag");
  ros::NodeHandle nh("~");
  gici::initializeSignalHandles();

  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(arguments.config);
  }
  catch (const YAML::BadFile&) {
    std::cerr << "Unable to load config file: " << arguments.config << std::endl;
    return 2;
  }

  auto node_options = std::make_shared<gici::NodeOptionHandle>(yaml_node);
  if (!node_options->valid) {
    std::cerr << "Invalid configuration: " << arguments.config << std::endl;
    return 2;
  }

  std::unique_ptr<gici::RosNodeHandle> node_handle(
      new gici::RosNodeHandle(nh, node_options));
  // Start existing streamer and estimator workers; this runner never calls ros::spin().
  gici::SpinControl::run();
  auto imu_stream = node_handle->getRosStream("str_ros_imu");
  auto lidar_stream = node_handle->getRosStream("str_ros_lidar");
  auto rover_stream = node_handle->getRosStream("str_ros_gnss_rov");
  auto reference_stream = node_handle->getRosStream("str_ros_gnss_ref");
  if (!imu_stream || !lidar_stream || !rover_stream || !reference_stream) {
    std::cerr << "RobNav input streams are missing; expected tags str_ros_imu, "
              << "str_ros_lidar, str_ros_gnss_rov, str_ros_gnss_ref." << std::endl;
    gici::SpinControl::kill();
    return 2;
  }

  std::shared_ptr<gici::MultiSensorEstimating> estimator;
  for (const auto& estimating : node_handle->getEstimatings()) {
    estimator = std::dynamic_pointer_cast<gici::MultiSensorEstimating>(estimating);
    if (estimator) break;
  }
  if (!estimator) {
    std::cerr << "No multi-sensor estimator was initialized." << std::endl;
    gici::SpinControl::kill();
    return 2;
  }

  // Allow existing static file streamers, including DCB, to complete initialization.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  try {
    rosbag::Bag sensor_bag(arguments.sensor_bag, rosbag::bagmode::Read);
    rosbag::Bag rover_bag(arguments.rover_bag, rosbag::bagmode::Read);
    rosbag::Bag reference_bag(arguments.reference_bag, rosbag::bagmode::Read);
    rosbag::Bag ephemeris_bag(arguments.ephemeris_bag, rosbag::bagmode::Read);

    size_t ephemeris_count = 0;
    rosbag::View ephemeris_view(ephemeris_bag,
                                rosbag::TopicQuery({arguments.ephemerides_topic}));
    for (const auto& instance : ephemeris_view) {
      auto message = instance.instantiate<gici_ros::GnssEphemerides>();
      if (!message) {
        throw std::runtime_error("Invalid ephemerides message in rosbag.");
      }
      reference_stream->feedGnssEphemerides(message);
      ++ephemeris_count;
    }

    gici_ros::GnssAntennaPositionPtr antenna_message(
        new gici_ros::GnssAntennaPosition());
    antenna_message->pos = {arguments.base_ecef_x, arguments.base_ecef_y,
                            arguments.base_ecef_z};
    reference_stream->feedGnssAntennaPosition(antenna_message);

    const ros::Time start_time = timeFromSec(arguments.start_time_s);
    const ros::Time end_time = timeFromSec(arguments.start_time_s + arguments.duration_s);
    rosbag::View formal_view;
    formal_view.addQuery(sensor_bag,
                         rosbag::TopicQuery({arguments.imu_topic, arguments.lidar_topic}),
                         start_time, end_time);
    formal_view.addQuery(rover_bag, rosbag::TopicQuery({arguments.rover_observations_topic}),
                         start_time, end_time);
    formal_view.addQuery(reference_bag,
                         rosbag::TopicQuery({arguments.reference_observations_topic}),
                         start_time, end_time);

    size_t imu_count = 0;
    size_t lidar_count = 0;
    size_t rover_count = 0;
    size_t reference_count = 0;
    ros::Time last_record_time;
    bool has_record_time = false;
    const auto wall_start = std::chrono::steady_clock::now();
    for (const auto& instance : formal_view) {
      if (has_record_time && instance.getTime() < last_record_time) {
        throw std::runtime_error("Merged rosbag view has descending record time.");
      }
      last_record_time = instance.getTime();
      has_record_time = true;

      if (instance.getTopic() == arguments.imu_topic) {
        auto message = instance.instantiate<sensor_msgs::Imu>();
        if (!message) throw std::runtime_error("Invalid IMU message in rosbag.");
        imu_stream->feedImu(message);
        ++imu_count;
      }
      else if (instance.getTopic() == arguments.lidar_topic) {
        auto message = instance.instantiate<sensor_msgs::PointCloud2>();
        if (!message) throw std::runtime_error("Invalid LiDAR message in rosbag.");
        lidar_stream->feedHesaiAt128(message);
        ++lidar_count;
      }
      else if (instance.getTopic() == arguments.rover_observations_topic) {
        auto message = instance.instantiate<gici_ros::GnssObservations>();
        if (!message) throw std::runtime_error("Invalid rover GNSS message in rosbag.");
        rover_stream->feedGnssObservations(message);
        ++rover_count;
      }
      else if (instance.getTopic() == arguments.reference_observations_topic) {
        auto message = instance.instantiate<gici_ros::GnssObservations>();
        if (!message) throw std::runtime_error("Invalid reference GNSS message in rosbag.");
        reference_stream->feedGnssObservations(message);
        ++reference_count;
      }
    }

    estimator->notifyInputFinished();
    const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    std::chrono::steady_clock::time_point idle_candidate;
    bool has_idle_candidate = false;
    bool drained = false;
    std::chrono::steady_clock::time_point wall_end;
    while (std::chrono::steady_clock::now() < drain_deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (estimator->pipelineIdle()) {
        if (!has_idle_candidate) {
          idle_candidate = now;
          has_idle_candidate = true;
        }
        else if (now - idle_candidate >= std::chrono::milliseconds(100)) {
          wall_end = idle_candidate;
          drained = true;
          break;
        }
      }
      else {
        has_idle_candidate = false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!drained) {
      std::cerr << "Timed out while draining the estimator pipeline." << std::endl;
      gici::SpinControl::kill();
      ros::shutdown();
      return 3;
    }

    const auto peaks = estimator->pipelineQueuePeaks();
    const double wall_processing_time_s =
        std::chrono::duration<double>(wall_end - wall_start).count();
    std::cout << "direct_bag_wall_processing_time_s=" << wall_processing_time_s << std::endl
              << "ephemerides=" << ephemeris_count << ", imu=" << imu_count
              << ", lidar=" << lidar_count << ", rover_gnss=" << rover_count
              << ", reference_gnss=" << reference_count << std::endl
              << "queue_peak_addin=" << peaks.addin
              << ", queue_peak_lidar_frontend=" << peaks.lidar_frontend
              << ", queue_peak_backend=" << peaks.backend << std::endl;

    sensor_bag.close();
    rover_bag.close();
    reference_bag.close();
    ephemeris_bag.close();
  }
  catch (const std::exception& exception) {
    std::cerr << "Direct rosbag execution failed: " << exception.what() << std::endl;
    gici::SpinControl::kill();
    ros::shutdown();
    return 3;
  }

  gici::SpinControl::kill();
  ros::shutdown();
  return 0;
}
