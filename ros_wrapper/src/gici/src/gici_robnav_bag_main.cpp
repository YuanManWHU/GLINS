/**
* @Function: Direct rosbag runner for the RobNav RTK/IMU/LiDAR RRR profile
*
* Copyright (C) 2026
**/
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "gici/fusion/multisensor_estimating.h"
#include "gici/ros_interface/ros_node_handle.h"
#include "gici/utility/experiment_recorder.h"
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

constexpr size_t kAddinHighWatermark = 100;
constexpr size_t kAddinLowWatermark = 40;
constexpr auto kBackpressureSleep = std::chrono::milliseconds(1);
constexpr double kBackendLagHighWatermarkS = 1.5;
constexpr double kBackendLagLowWatermarkS = 0.8;

struct BackpressureStats {
  size_t enter_count = 0;
  double wait_time_s = 0.0;
  size_t max_addin = 0;
  size_t backend_enter_count = 0;
  double backend_wait_time_s = 0.0;
  double backend_lag_max_s = 0.0;
};

void applyDirectBackpressure(
    const std::shared_ptr<gici::MultiSensorEstimating>& estimator,
    BackpressureStats* stats)
{
  auto state = estimator->directInputFlowState();
  stats->max_addin = std::max(stats->max_addin, state.addin_size);
  stats->backend_lag_max_s = std::max(stats->backend_lag_max_s, state.backend_lag);

  const bool addin_overloaded = state.addin_size >= kAddinHighWatermark;
  const bool backend_overloaded = state.backend_lag >= kBackendLagHighWatermarkS;
  if (!addin_overloaded && !backend_overloaded) {
    return;
  }

  // Only addin-triggered pauses need future IMU coverage for the newest raw scan.
  if (addin_overloaded && !backend_overloaded &&
      state.latest_input_lidar_timefinal > 0.0 &&
      state.latest_input_imu_timestamp < state.latest_input_lidar_timefinal) {
    return;
  }

  if (addin_overloaded) ++stats->enter_count;
  if (backend_overloaded) ++stats->backend_enter_count;
  const auto wait_start = std::chrono::steady_clock::now();
  while (gici::SpinControl::ok()) {
    state = estimator->directInputFlowState();
    stats->max_addin = std::max(stats->max_addin, state.addin_size);
    stats->backend_lag_max_s = std::max(stats->backend_lag_max_s, state.backend_lag);
    if (state.addin_size <= kAddinLowWatermark &&
        state.backend_lag <= kBackendLagLowWatermarkS) {
      break;
    }
    std::this_thread::sleep_for(kBackpressureSleep);
  }

  const double wait_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count();
  if (addin_overloaded) stats->wait_time_s += wait_time_s;
  if (backend_overloaded) stats->backend_wait_time_s += wait_time_s;
}

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

  // GRLINS opens bags and creates views before MSCKF construction starts its timer.
  const ros::Time start_time = timeFromSec(arguments.start_time_s);
  const ros::Time end_time = timeFromSec(arguments.start_time_s + arguments.duration_s);
  const char* result_dir = std::getenv("GLINS_RESULT_DIR");
  try {
    rosbag::Bag sensor_bag(arguments.sensor_bag, rosbag::bagmode::Read);
    rosbag::Bag rover_bag(arguments.rover_bag, rosbag::bagmode::Read);
    rosbag::Bag reference_bag(arguments.reference_bag, rosbag::bagmode::Read);
    rosbag::Bag ephemeris_bag(arguments.ephemeris_bag, rosbag::bagmode::Read);

    rosbag::View formal_view;
    formal_view.addQuery(sensor_bag,
                         rosbag::TopicQuery({arguments.imu_topic, arguments.lidar_topic}),
                         start_time, end_time);
    formal_view.addQuery(rover_bag, rosbag::TopicQuery({arguments.rover_observations_topic}),
                         start_time, end_time);
    formal_view.addQuery(reference_bag,
                         rosbag::TopicQuery({arguments.reference_observations_topic}),
                         start_time, end_time);
    // Preload only records before the formal interval; later ephemerides are time-dependent.
    rosbag::View ephemeris_preload_view(
        ephemeris_bag, rosbag::TopicQuery({arguments.ephemerides_topic}),
        ros::Time(), start_time);
    formal_view.addQuery(ephemeris_bag, rosbag::TopicQuery({arguments.ephemerides_topic}),
                         start_time, end_time);

    // Match GRLINS: algorithm construction through normal output finalization is measured.
    using TotalClock = std::chrono::system_clock;
    const auto total_start = TotalClock::now();

    auto node_options = std::make_shared<gici::NodeOptionHandle>(yaml_node);
    if (!node_options->valid) {
      std::cerr << "Invalid configuration: " << arguments.config << std::endl;
      return 2;
    }

    std::unique_ptr<gici::RosNodeHandle> node_handle(
        new gici::RosNodeHandle(nh, node_options));
    // This runner drives callbacks directly and deliberately does not call ros::spin().
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

    // This executable is the direct-bag benchmark, so preserve LiDAR-rover chronology here only.
    estimator->enableDirectChronologicalAdmissionBarrier(true);

    // Existing DCB processing belongs to algorithm initialization and is included in Total.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    size_t ephemeris_count = 0;
    size_t ephemeris_preload_count = 0;
    for (const auto& instance : ephemeris_preload_view) {
      auto message = instance.instantiate<gici_ros::GnssEphemerides>();
      if (!message) throw std::runtime_error("Invalid ephemerides message in rosbag.");
      reference_stream->feedGnssEphemerides(message);
      ++ephemeris_count;
      ++ephemeris_preload_count;
    }

    gici_ros::GnssAntennaPositionPtr antenna_message(new gici_ros::GnssAntennaPosition());
    antenna_message->pos = {arguments.base_ecef_x, arguments.base_ecef_y,
                            arguments.base_ecef_z};
    reference_stream->feedGnssAntennaPosition(antenna_message);

    size_t imu_count = 0;
    size_t lidar_count = 0;
    size_t rover_count = 0;
    size_t reference_count = 0;
    ros::Time last_record_time;
    bool has_record_time = false;
    // Limit diagnostic I/O to one snapshot per 0.5 s of bag record time.
    double next_diag_time = arguments.start_time_s + 0.5;
    const auto formal_loop_start = std::chrono::steady_clock::now();
    BackpressureStats backpressure_stats;
    for (const auto& instance : formal_view) {
      if (has_record_time && instance.getTime() < last_record_time) {
        throw std::runtime_error("Merged rosbag view has descending record time.");
      }
      last_record_time = instance.getTime();
      has_record_time = true;

      if (instance.getTopic() == arguments.ephemerides_topic) {
        auto message = instance.instantiate<gici_ros::GnssEphemerides>();
        if (!message) throw std::runtime_error("Invalid ephemerides message in rosbag.");
        reference_stream->feedGnssEphemerides(message);
        ++ephemeris_count;
      }
      else if (instance.getTopic() == arguments.imu_topic) {
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

      // Throttle only after this message entered the original GLINS pipeline.
      applyDirectBackpressure(estimator, &backpressure_stats);

      const double input_record_time = instance.getTime().toSec();
      if (input_record_time >= next_diag_time) {
        const auto snapshot = estimator->directPipelineSnapshot();
        LOG(INFO) << std::fixed << std::setprecision(6)
                  << "[DIRECT_PIPELINE] input=" << input_record_time
                  << " latest_imu=" << snapshot.latest_imu_timestamp
                  << " addin=" << snapshot.addin_size
                  << " addin_front=" << snapshot.oldest_addin_timestamp
                  << " lidar_q=" << snapshot.lidar_frontend_size
                  << " lidar_front=" << snapshot.oldest_lidar_frontend_timestamp
                  << " lidar_active=" << snapshot.active_lidar_timestamp
                  << " align=" << snapshot.align_size
                  << " align_front=" << snapshot.oldest_align_timestamp
                  << " backend_q=" << snapshot.backend_size
                  << " backend_front=" << snapshot.oldest_backend_timestamp
                  << " backend_active=" << snapshot.active_backend_timestamp;
        next_diag_time += 0.5;
      }
    }
    const auto formal_loop_end = std::chrono::steady_clock::now();

    // Keep normal alignment latency until no ingress or LiDAR frontend work can add data.
    const auto ingress_drain_start = std::chrono::steady_clock::now();
    const auto ingress_deadline = ingress_drain_start + std::chrono::seconds(60);
    bool ingress_quiescent = false;
    while (std::chrono::steady_clock::now() < ingress_deadline) {
      if (estimator->readyForFinalAlignmentFlush()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (estimator->readyForFinalAlignmentFlush()) {
          ingress_quiescent = true;
          break;
        }
      }
      std::this_thread::sleep_for(kBackpressureSleep);
    }
    if (!ingress_quiescent) {
      throw std::runtime_error(
          "Timed out while draining direct-input/addin/LiDAR frontend before final alignment flush.");
    }
    const double ingress_quiesce_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - ingress_drain_start)
            .count();

    const auto chronological_stats_at_eof = estimator->chronologicalAdmissionStats();
    estimator->notifyInputFinished();
    const auto drain_start = std::chrono::steady_clock::now();
    const auto drain_deadline = drain_start + std::chrono::seconds(60);
    bool idle_once = false;
    bool drained = false;
    while (std::chrono::steady_clock::now() < drain_deadline) {
      if (estimator->pipelineIdle()) {
        if (idle_once) {
          drained = true;
          break;
        }
        idle_once = true;
      }
      else {
        idle_once = false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto drain_end = std::chrono::steady_clock::now();

    if (!drained) {
      std::cerr << "Timed out while draining the estimator pipeline." << std::endl;
      gici::SpinControl::kill();
      ros::shutdown();
      return 3;
    }

    const auto peaks = estimator->pipelineQueuePeaks();

    // Stop and join all workers before the explicit, idempotent recorder flush.
    gici::SpinControl::kill();
    estimator.reset();
    imu_stream.reset();
    lidar_stream.reset();
    rover_stream.reset();
    reference_stream.reset();
    node_handle.reset();
    gici::ExperimentRecorder::instance().flush();

    const auto total_end = TotalClock::now();
    const double total_running_time_s =
        std::chrono::duration<double>(total_end - total_start).count();
    const double formal_input_loop_time_s =
        std::chrono::duration<double>(formal_loop_end - formal_loop_start).count();
    const double drain_time_s =
        std::chrono::duration<double>(drain_end - drain_start).count();

    // Writing Total itself is necessarily outside its own measured interval.
    if (result_dir != nullptr && result_dir[0] != 0) {
      std::ofstream total_file(std::string(result_dir) + "/total_running_time.txt");
      if (!total_file.is_open()) {
        std::cerr << "Unable to write total running time to " << result_dir << std::endl;
      }
      else {
        total_file << std::fixed << std::setprecision(9)
                   << "total_running_time_s=" << total_running_time_s << "\n";
      }
    }

    std::cout << "total_running_time_s=" << total_running_time_s << std::endl
              << "formal_input_loop_time_s=" << formal_input_loop_time_s << std::endl
              << "backpressure_enter_count=" << backpressure_stats.enter_count << std::endl
              << "backpressure_wait_time_s=" << backpressure_stats.wait_time_s << std::endl
              << "backend_lag_max_s=" << backpressure_stats.backend_lag_max_s << std::endl
              << "backend_backpressure_enter_count="
              << backpressure_stats.backend_enter_count << std::endl
              << "backend_backpressure_wait_time_s="
              << backpressure_stats.backend_wait_time_s << std::endl
              << "backpressure_max_addin=" << backpressure_stats.max_addin << std::endl
              << "ingress_quiesce_time_s=" << ingress_quiesce_time_s << std::endl
              << "drain_time_s=" << drain_time_s << std::endl
              << "ephemerides=" << ephemeris_count << ", imu=" << imu_count
              << ", lidar=" << lidar_count << ", rover_gnss=" << rover_count
              << ", reference_gnss=" << reference_count << std::endl
              << "ephemerides_preload=" << ephemeris_preload_count
              << ", ephemerides_formal=" << ephemeris_count - ephemeris_preload_count
              << std::endl
              << "queue_peak_addin=" << peaks.addin
              << ", queue_peak_lidar_frontend=" << peaks.lidar_frontend
              << ", queue_peak_backend=" << peaks.backend << std::endl
              << "chronological_barrier_block_count="
              << chronological_stats_at_eof.rover_gnss_block_count << std::endl
              << "chronological_barrier_max_gap_s="
              << chronological_stats_at_eof.max_block_gap_s << std::endl
              << "pending_lidar_admission_peak="
              << chronological_stats_at_eof.pending_lidar_peak << std::endl
              << "pending_lidar_at_eof="
              << chronological_stats_at_eof.pending_lidar << std::endl;

    // Bag close/destruction is intentionally outside the algorithm Total interval.
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

  ros::shutdown();
  return 0;
}
