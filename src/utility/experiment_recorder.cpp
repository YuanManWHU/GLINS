/**
 * @Function: Experiment result recording and timing instrumentation
 **/
#include "gici/utility/experiment_recorder.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <thread>

#include <glog/logging.h>

#include "gici/estimate/estimator_types.h"
#include "gici/gnss/gnss_common.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/utility/transform.h"

namespace gici {
namespace {

double gpsSowFromUtc(const double utc_timestamp)
{
  if (!std::isfinite(utc_timestamp)) return std::numeric_limits<double>::quiet_NaN();
  const double gps_timestamp = gnss_common::utcTimeToGpsTime(utc_timestamp);
  int week = 0;
  return time2gpst(gnss_common::doubleToGtime(gps_timestamp), &week);
}

const char* timingScopeToString(const TimingScope scope)
{
  return scope == TimingScope::Detail ? "detail" : "coarse";
}

const char* timingTriggerToString(const TimingTrigger trigger)
{
  switch (trigger) {
    case TimingTrigger::Imu: return "imu";
    case TimingTrigger::Gnss: return "gnss";
    case TimingTrigger::Lidar: return "lidar";
    case TimingTrigger::None: return "none";
  }
  return "none";
}

const char* timingModuleToString(const TimingModule module)
{
  switch (module) {
    case TimingModule::Initialization: return "initialization";
    case TimingModule::LidarInitialization: return "lidar_initialization";
    case TimingModule::LidarPreprocess: return "lidar_preprocess";
    case TimingModule::GnssAuxRtk: return "gnss_aux_rtk";
    case TimingModule::GnssFactor: return "gnss_factor";
    case TimingModule::LidarFactor: return "lidar_factor";
    case TimingModule::Optimization: return "optimization";
    case TimingModule::GnssPost: return "gnss_post";
    case TimingModule::LidarPost: return "lidar_post";
    case TimingModule::Marginalization: return "marginalization";
    case TimingModule::LidarFrontendTotal: return "lidar_frontend_total";
    case TimingModule::BackendAdd: return "backend_add";
    case TimingModule::BackendEstimate: return "backend_estimate";
  }
  return "unknown";
}

}  // namespace

ExperimentRecorder& ExperimentRecorder::instance()
{
  static ExperimentRecorder recorder;
  return recorder;
}

ExperimentRecorder::ExperimentRecorder() : start_time_(std::chrono::steady_clock::now())
{
  const char* result_dir = std::getenv("GLINS_RESULT_DIR");
  if (result_dir == nullptr || result_dir[0] == 0) return;

  result_dir_ = result_dir;
  std::error_code error;
  std::filesystem::create_directories(result_dir_, error);
  if (error || !std::filesystem::is_directory(result_dir_, error)) {
    LOG(WARNING) << "Unable to create GLINS result directory: " << result_dir_;
    result_dir_.clear();
    return;
  }
  enabled_ = true;
}

ExperimentRecorder::~ExperimentRecorder()
{
  flush();
}

uint64_t ExperimentRecorder::nextEventId()
{
  return next_event_id_.fetch_add(1, std::memory_order_relaxed);
}

double ExperimentRecorder::monotonicMilliseconds() const
{
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  return std::chrono::duration<double, std::milli>(elapsed).count();
}

void ExperimentRecorder::warnNavRecordOnce(const char* message)
{
  if (!nav_warning_emitted_.exchange(true, std::memory_order_relaxed)) {
    LOG(WARNING) << message;
  }
}

void ExperimentRecorder::recordNav(const Solution& solution,
                                   const ImuEstimatorBaseOptions& imu_options)
{
  if (!enabled_) return;
  if (solution.coordinate == nullptr) {
    warnNavRecordOnce("Skipping NAV record because the solution coordinate is unavailable.");
    return;
  }

  const double sow = gpsSowFromUtc(solution.timestamp);
  const Eigen::Vector3d lla = solution.coordinate->convert(
      solution.pose.getPosition(), GeoType::ENU, GeoType::LLA);
  const Eigen::Vector3d velocity_enu = solution.speed_and_bias.head<3>();

  Eigen::Matrix3d R_NW;
  R_NW << 0.0, 1.0, 0.0,
          1.0, 0.0, 0.0,
          0.0, 0.0, -1.0;
  const Eigen::Quaterniond q_BI =
      eulerAngleToQuaternion(imu_options.body_to_imu_rotation * D2R);
  const Eigen::Matrix3d R_WB = solution.pose.getEigenQuaternion().toRotationMatrix();
  Eigen::Quaterniond q_NI(R_NW * R_WB * q_BI.toRotationMatrix());
  if (!std::isfinite(q_NI.norm()) || q_NI.norm() <= 0.0) {
    warnNavRecordOnce("Skipping NAV record because the solution attitude is invalid.");
    return;
  }
  q_NI.normalize();
  const Eigen::Vector3d rpy = quaternionToEulerAngle(q_NI) * R2D;

  NavRecord record;
  record.sow = sow;
  record.latitude = lla(0) * R2D;
  record.longitude = lla(1) * R2D;
  record.height = lla(2);
  record.vn = velocity_enu.y();
  record.ve = velocity_enu.x();
  record.vd = -velocity_enu.z();
  record.roll = rpy(0);
  record.pitch = rpy(1);
  record.yaw = rpy(2);

  const double values[] = {record.sow, record.latitude, record.longitude, record.height,
                           record.vn, record.ve, record.vd, record.roll, record.pitch,
                           record.yaw};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      warnNavRecordOnce("Skipping NAV record because it contains non-finite values.");
      return;
    }
  }

  std::lock_guard<std::mutex> lock(nav_mutex_);
  nav_records_.push_back(record);
}

void ExperimentRecorder::recordTiming(const TimingEvent& event)
{
  if (!enabled_ || !std::isfinite(event.data_timestamp) ||
      !std::isfinite(event.wall_start_ms) || !std::isfinite(event.duration_ms)) {
    return;
  }

  std::lock_guard<std::mutex> lock(timing_mutex_);
  timing_events_.push_back(event);
}

void ExperimentRecorder::flush()
{
  std::lock_guard<std::mutex> flush_lock(flush_mutex_);
  if (!enabled_ || flushed_) return;
  flushed_ = true;

  std::vector<NavRecord> nav_records;
  std::vector<TimingEvent> timing_events;
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    nav_records = nav_records_;
  }
  {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    timing_events = timing_events_;
  }

  std::ofstream nav_file(result_dir_ + "/glins.nav");
  if (!nav_file.is_open()) {
    LOG(WARNING) << "Unable to write GLINS NAV records to " << result_dir_;
  } else {
    nav_file << std::fixed << std::setprecision(10);
    for (const auto& record : nav_records) {
      nav_file << "0 " << record.sow << " " << record.latitude << " " << record.longitude
               << " " << record.height << " " << record.vn << " " << record.ve << " "
               << record.vd << " " << record.roll << " " << record.pitch << " " << record.yaw
               << "\n";
    }
  }

  std::ofstream timing_file(result_dir_ + "/timing_events.csv");
  if (!timing_file.is_open()) {
    LOG(WARNING) << "Unable to write GLINS timing records to " << result_dir_;
    return;
  }
  timing_file << "event_id,scope,module,trigger,data_timestamp,gps_sow,wall_start_ms,duration_ms,thread_id\n";
  for (const auto& event : timing_events) {
    timing_file << event.event_id << "," << timingScopeToString(event.scope) << ","
                << timingModuleToString(event.module) << ","
                << timingTriggerToString(event.trigger) << "," << std::fixed
                << std::setprecision(9) << event.data_timestamp << "," << std::setprecision(10)
                << event.gps_sow << "," << std::setprecision(6) << event.wall_start_ms << ","
                << event.duration_ms << "," << event.thread_id << "\n";
  }
}

ScopedModuleTimer::ScopedModuleTimer(const TimingModule module, const double data_timestamp,
                                     const TimingTrigger trigger, const TimingScope scope)
    : module_(module), trigger_(trigger), scope_(scope), data_timestamp_(data_timestamp)
{
  ExperimentRecorder& recorder = ExperimentRecorder::instance();
  enabled_ = recorder.enabled();
  if (!enabled_) return;

  start_time_ = std::chrono::steady_clock::now();
  event_id_ = recorder.nextEventId();
  wall_start_ms_ = recorder.monotonicMilliseconds();
}

ScopedModuleTimer::~ScopedModuleTimer()
{
  if (!enabled_) return;

  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  TimingEvent event;
  event.event_id = event_id_;
  event.scope = scope_;
  event.module = module_;
  event.trigger = trigger_;
  event.data_timestamp = data_timestamp_;
  event.gps_sow = gpsSowFromUtc(data_timestamp_);
  event.wall_start_ms = wall_start_ms_;
  event.duration_ms = std::chrono::duration<double, std::milli>(elapsed).count();
  event.thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
  ExperimentRecorder::instance().recordTiming(event);
}

}  // namespace gici
