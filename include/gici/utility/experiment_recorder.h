/**
 * @Function: Experiment result recording and timing instrumentation
 **/
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace gici {

struct Solution;
struct ImuEstimatorBaseOptions;

enum class TimingScope : uint8_t {
  Detail = 0,
  Coarse = 1
};

enum class TimingTrigger : uint8_t {
  None = 0,
  Imu,
  Gnss,
  Lidar
};

enum class TimingModule : uint8_t {
  Initialization = 0,
  LidarInitialization,
  LidarPreprocess,
  GnssAuxRtk,
  GnssFactor,
  LidarFactor,
  Optimization,
  GnssPost,
  LidarPost,
  Marginalization,
  LidarFrontendTotal,
  BackendAdd,
  BackendEstimate
};

struct TimingEvent {
  uint64_t event_id = 0;
  TimingScope scope = TimingScope::Detail;
  TimingModule module = TimingModule::Initialization;
  TimingTrigger trigger = TimingTrigger::None;
  double data_timestamp = 0.0;
  double gps_sow = 0.0;
  double wall_start_ms = 0.0;
  double duration_ms = 0.0;
  uint64_t thread_id = 0;
};

struct NavRecord {
  double sow = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double height = 0.0;
  double vn = 0.0;
  double ve = 0.0;
  double vd = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

// A disabled recorder is intentionally a near-no-op so baseline estimation is unchanged.
class ExperimentRecorder {
public:
  static ExperimentRecorder& instance();

  bool enabled() const { return enabled_; }

  void recordNav(const Solution& solution, const ImuEstimatorBaseOptions& imu_options);
  void recordTiming(const TimingEvent& event);

  uint64_t nextEventId();
  double monotonicMilliseconds() const;
  void flush();

private:
  ExperimentRecorder();
  ~ExperimentRecorder();

  ExperimentRecorder(const ExperimentRecorder&) = delete;
  ExperimentRecorder& operator=(const ExperimentRecorder&) = delete;

  void warnNavRecordOnce(const char* message);

  bool enabled_ = false;
  bool flushed_ = false;
  std::string result_dir_;
  std::chrono::steady_clock::time_point start_time_;
  std::atomic<uint64_t> next_event_id_{0};
  std::atomic<bool> nav_warning_emitted_{false};
  std::mutex nav_mutex_;
  std::mutex timing_mutex_;
  std::mutex flush_mutex_;
  std::vector<NavRecord> nav_records_;
  std::vector<TimingEvent> timing_events_;
};

// Records a single module scope without changing the wrapped algorithm path.
class ScopedModuleTimer {
public:
  ScopedModuleTimer(TimingModule module, double data_timestamp, TimingTrigger trigger,
                    TimingScope scope = TimingScope::Detail);
  ~ScopedModuleTimer();

  ScopedModuleTimer(const ScopedModuleTimer&) = delete;
  ScopedModuleTimer& operator=(const ScopedModuleTimer&) = delete;

private:
  bool enabled_ = false;
  uint64_t event_id_ = 0;
  TimingModule module_;
  TimingTrigger trigger_;
  TimingScope scope_;
  double data_timestamp_ = 0.0;
  double wall_start_ms_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace gici
