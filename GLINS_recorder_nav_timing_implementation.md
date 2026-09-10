# GLINS 实验记录功能实现方案：Recorder → NAV → Timing Instrumentation

## 1. 目标与边界

本文档针对当前已在 RobNav `building02 / street00 / street01 / street02` 四组数据上正常运行的 GLINS RRR 基线，设计三部分新增功能：

1. **Recorder**：统一管理实验输出，不修改现有 YAML、不改变估计器参数和数据流。
2. **NAV**：以 5 Hz 保存最终 IMU 导航状态，格式与现有 GRLINS `.nav` 保持一致。
3. **Timing instrumentation**：记录 GLINS 自身主要计算模块每次运行对应的数据时刻、执行起点和运行时间，为后续模块耗时统计和总运行时间分析提供原始数据。

本阶段只做**旁路观测功能**。以下内容明确保持不变：

- 不关闭现有 ROS visualization output；
- 不修改现有 `output_tags`；
- 不修改 Ceres 线程数；
- 不把 GLINS 改成单线程；
- 不改变 RRR 因子、状态、边缘化、数据关联、地图更新等算法逻辑；
- 正式实验时只是不启动 RViz；
- 不在本阶段实现运行脚本中的 `process CPU time` 和 `timing_summary.csv` 汇总；这些在下一阶段基于本阶段输出实现。

源码基线：

```text
Repository : Garfield-cn/GLINS
Commit     : 7fc2a1f8ba0a3a88018e691b76826c19675f51bc
```

用户侧 NAV 格式参考当前 `YuanManWHU/grlins_ws` 的写法：固定格式、第一列为 `0`，随后为 SOW、BLH、NED velocity 和 Euler attitude；当前代码整体使用 `std::fixed << std::setprecision(10)` 输出。

---

## 2. 总体数据流

实现后的旁路记录关系如下：

```text
                         MultiSensorEstimating
                         ┌───────────────────────────┐
5 Hz output timestamp ─► │ updateSolution()          │
                         │   └─ recordNav()          │ ─────► glins.nav
                         │
LiDAR scan ─────────────► │ runLidarFrontend()       │
                         │   └─ lidar_preprocess     │
                         │
backend measurement ────► │ processEstimator()       │
                         │   ├─ backend_add [coarse] │
                         │   └─ backend_estimate     │
                         └─────────────┬─────────────┘
                                       │
                                       ▼
                         RtkImuLidarRrrEstimator
             ┌─────────────────────────────────────────────┐
             │ initialization                             │
             │ lidar_initialization                       │
             │ gnss_aux_rtk                               │
             │ gnss_factor                                │
             │ lidar_factor                               │
             │ optimization [gnss/lidar]                  │
             │ gnss_post                                  │
             │ lidar_post                                 │
             │ marginalization [gnss/lidar]               │
             └──────────────────────┬──────────────────────┘
                                    │
                                    ▼
                              Recorder buffer
                                    │
                         graceful shutdown / flush
                                    │
                                    ▼
                           timing_events.csv
```

核心原则是：

```text
算法代码负责“算”
Recorder 只负责“观察和记录”
```

Recorder 不参与任何估计判断，不向 estimator 返回会改变算法路径的状态。

---

# Part A. Recorder

## 3. 新增文件

新增：

```text
include/gici/utility/experiment_recorder.h
src/utility/experiment_recorder.cpp
```

当前 GLINS 根目录 `CMakeLists.txt` 已通过：

```cmake
aux_source_directory(src/utility DIR_utility)
```

自动收集 `src/utility` 下的 `.cpp`，因此新增 `experiment_recorder.cpp` 后**不需要额外修改根 CMake 源文件列表**。

---

## 4. Recorder 启用方式

不增加 YAML 参数。

运行脚本启动 GLINS 前设置环境变量：

```bash
export GLINS_RESULT_DIR=/path/to/results/robnav_rrr/building02/<run>
```

Recorder 初始化时读取：

```cpp
const char* result_dir = std::getenv("GLINS_RESULT_DIR");
```

行为定义：

```text
GLINS_RESULT_DIR 未设置或为空
    → Recorder disabled
    → recordNav()/recordTiming() 立即返回
    → 普通 GLINS 行为不受影响

GLINS_RESULT_DIR 已设置
    → 创建/检查结果目录
    → 在内存中记录 NAV 和 timing event
    → 程序正常退出时统一写文件
```

这样不需要为了实验修改公共 YAML。

---

## 5. Recorder 类设计

建议接口：

```cpp
namespace gici {

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

  // Parent/coarse timers: only for sanity check, never added to detail sum.
  LidarFrontendTotal,
  BackendAdd,
  BackendEstimate
};

struct TimingEvent {
  uint64_t event_id = 0;

  TimingScope scope;
  TimingModule module;
  TimingTrigger trigger;

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

class ExperimentRecorder {
public:
  static ExperimentRecorder& instance();

  bool enabled() const;

  void recordNav(
      const Solution& solution,
      const ImuEstimatorBaseOptions& imu_options);

  void recordTiming(const TimingEvent& event);

  uint64_t nextEventId();

  double monotonicMilliseconds() const;

  void flush();

private:
  ExperimentRecorder();
  ~ExperimentRecorder();

  ExperimentRecorder(const ExperimentRecorder&) = delete;
  ExperimentRecorder& operator=(const ExperimentRecorder&) = delete;

  // buffers / mutex / output path / status...
};

}  // namespace gici
```

### 5.1 为什么 timing module 使用 enum

运行过程中避免反复构造：

```cpp
std::string("optimization")
```

以及字符串比较。

实际 CSV 写出时再：

```text
TimingModule::Optimization → "optimization"
```

降低 instrumentation 本身的动态分配开销。

---

## 6. RAII 模块计时器

新增：

```cpp
class ScopedModuleTimer {
public:
  ScopedModuleTimer(
      TimingModule module,
      double data_timestamp,
      TimingTrigger trigger,
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

  std::chrono::steady_clock::time_point start_;
};
```

使用方式：

```cpp
{
  ScopedModuleTimer timer(
      TimingModule::Optimization,
      timestamp,
      TimingTrigger::Lidar);

  optimize();
}
```

无论正常离开 scope 还是函数提前 `return`，析构函数都会记录本次 duration。

---

## 7. 时间源

模块运行时间统一使用：

```cpp
std::chrono::steady_clock
```

不使用：

```cpp
ros::Time::now()
std::chrono::system_clock
```

因为模块运行时间是 elapsed time，必须避免系统时钟校时造成跳变。

### 7.1 一个 timing event 保存两个不同意义的时间

#### `data_timestamp`

传感器/状态自身的 Unix/ROS 时间：

```text
1745893401.265058
```

用于回答：

> 这个模块是哪一帧/哪个 GNSS epoch 触发的？

#### `wall_start_ms`

相对于 Recorder 创建时刻的 monotonic elapsed time：

```text
12534.218 ms
```

用于后续分析不同线程之间是否并行、事件是否重叠。

因此 `timing_events.csv` 不仅能统计平均耗时，还能恢复 GLINS 的 frontend/backend 并发关系。

---

## 8. 线程安全和运行时开销

GLINS 的 LiDAR frontend 与 backend 是不同线程，因此 Recorder 必须 thread-safe。

推荐：

```cpp
std::mutex timing_mutex_;
std::mutex nav_mutex_;

std::vector<TimingEvent> timing_events_;
std::vector<NavRecord> nav_records_;
```

`recordTiming()` 的 critical section 仅做：

```cpp
lock
vector.push_back(event)
unlock
```

不要在该 critical section 内：

- 做坐标转换；
- 做 SOW 转换；
- 写文件；
- flush 文件。

`event_id` 建议使用：

```cpp
std::atomic<uint64_t>
```

在 `ScopedModuleTimer` 构造时分配，因此 `event_id` 表示**模块开始顺序**，不是完成顺序。

`thread_id` 可保存：

```cpp
std::hash<std::thread::id>{}(std::this_thread::get_id())
```

仅用于诊断线程归属。

---

## 9. 文件写入策略

### 9.1 运行期间

只写内存：

```text
NAV          → nav_records_
Timing event → timing_events_
```

不逐帧执行：

```cpp
ofstream << ...
flush();
```

原因是逐事件文件 I/O 会反过来污染待测运行时间。

四组序列的规模完全可以放在内存：

- NAV：5 Hz，仅数千到约一万条；
- timing：主要是 1 Hz GNSS 和 10 Hz LiDAR 触发的模块，数量可控；
- 不记录 200 Hz 的普通 IMU buffer push timing，因此不会产生几十万条无意义 event。

### 9.2 退出时

统一生成：

```text
$GLINS_RESULT_DIR/glins.nav
$GLINS_RESULT_DIR/timing_events.csv
```

`flush()` 必须设计成 **idempotent**：

```text
第一次调用 → 写文件
后续调用   → 直接返回
```

---

## 10. Recorder flush 时机

在：

```cpp
MultiSensorEstimating::~MultiSensorEstimating()
```

现有 frontend/backend/measurement thread 全部 `join()` 完成以后调用：

```cpp
ExperimentRecorder::instance().flush();
```

原因是必须先保证：

```text
LiDAR frontend 不再产生 event
backend 不再产生 event
```

才能写最终文件。

Recorder 自身析构函数再调用一次 `flush()` 作为兜底；由于 `flush()` idempotent，不会重复写。

正式运行必须通过正常 `SIGINT` 退出，不使用：

```bash
kill -9
```

否则 C++ destructor 无法执行，内存中的记录可能来不及落盘。

---

# Part B. NAV

## 11. NAV 输出位置

修改：

```text
src/fusion/multisensor_estimating.cpp
```

函数：

```cpp
bool MultiSensorEstimating::updateSolution()
```

当前流程已经是：

```text
从 output_timestamps_ 取一个输出时刻
        ↓
getPoseEstimateAt(timestamp)
        ↓
getSpeedAndBiasEstimateAt(timestamp)
        ↓
读取 GNSS solution status
        ↓
pop output timestamp
        ↓
return true
```

NAV recorder 插在本次 solution 已全部成功生成之后：

```cpp
ExperimentRecorder::instance().recordNav(
    solution_,
    imu_base_options_);
```

推荐放在：

```cpp
output_timestamps_.pop_front();
```

之后、`return true;` 之前。

这样只有：

```text
updateSolution() == true
```

的完整导航解会进入 NAV。

---

## 12. NAV 输出频率

不在 Recorder 内再次做 downsample。

沿用现有：

```yaml
output_align_tag: str_ros_imu
output_downsample_rate: 40
```

IMU 为 200 Hz：

```text
200 / 40 = 5 Hz
```

所以 `updateSolution()` 本身已经提供目标 5 Hz 输出节奏。

Recorder 必须遵循现有 output control，不单独建立新的 0.2 s 采样逻辑。

---

## 13. NAV 文件格式

严格 11 列，无 header：

```text
0 sow latitude longitude height vn ve vd roll pitch yaw
```

与现有 GRLINS writer 最接近的实现是整行：

```cpp
std::fixed << std::setprecision(10)
```

因此示例：

```text
0 303619.2000000000 30.5297649350 114.3503808494 100.1529000000 1.2345670000 0.1234560000 -0.0123450000 0.1234560000 -1.2345670000 85.6789010000
```

明确要求：

```text
latitude / longitude：10 位小数
```

其余浮点量也统一 10 位小数，以与现有 GRLINS NAV writer 的 `setprecision(10)` 行为一致。

第一列固定：

```text
0
```

不输出：

- GPS week；
- GNSS solution status；
- satellite number；
- covariance；
- timestamp Unix 秒。

---

## 14. GPS 周内秒 SOW

`solution.timestamp` 为 UTC/Unix 时间。

必须先：

```cpp
double gps_time =
    gnss_common::utcTimeToGpsTime(solution.timestamp);
```

再：

```cpp
gtime_t gtime =
    gnss_common::doubleToGtime(gps_time);

int week = 0;
double sow = time2gpst(gtime, &week);
```

NAV 仅保存：

```text
sow
```

不能直接对 Unix timestamp 做 `% 604800`，否则 GPS/UTC leap-second 语义错误。

---

## 15. 位置：ENU → LLA

使用 GLINS 已有坐标转换：

```cpp
Eigen::Vector3d lla =
    solution.coordinate->convert(
        solution.pose.getPosition(),
        GeoType::ENU,
        GeoType::LLA);
```

然后：

```cpp
latitude  = lla(0) * R2D;
longitude = lla(1) * R2D;
height    = lla(2);
```

最终 latitude/longitude 保留 10 位小数。

---

## 16. 速度：ENU → NED

当前 `solution.speed_and_bias.head<3>()` 是世界 ENU 速度。

```cpp
const Eigen::Vector3d v_enu =
    solution.speed_and_bias.head<3>();

vn =  v_enu.y();
ve =  v_enu.x();
vd = -v_enu.z();
```

即：

\[
egin{bmatrix}
v_n\
v_e\
v_d
\end{bmatrix}
=
egin{bmatrix}
0&1&0\
1&0&0\
0&0&-1
\end{bmatrix}
egin{bmatrix}
v_e\
v_n\
v_u
\end{bmatrix}_{GLINS}.
\]

---

## 17. 姿态：RFU/ENU → FRD/NED

这是 NAV 实现中最需要单独验证的部分。

当前已确认：

```text
GLINS world W : ENU
GLINS body  B : RFU
physical IMU I: FRD
```

当前 RobNav：

```yaml
body_to_imu_rotation: [180, 0, 90]
```

GLINS `ImuEstimatorBase::rotateImuToBody()` 使用 `q_BI = eulerAngleToQuaternion(body_to_imu_rotation)`，所以该旋转语义是：

```text
IMU I → GLINS body B
```

### 17.1 ENU → NED

```cpp
Eigen::Matrix3d R_NW;
R_NW <<
    0.0, 1.0,  0.0,
    1.0, 0.0,  0.0,
    0.0, 0.0, -1.0;
```

### 17.2 IMU → NED

若 `solution.pose.getRotation()` 表示当前 GLINS body 到 world 的旋转 \(R_{WB}\)，则：

\[
R_{NI}=R_{NW}R_{WB}R_{BI}.
\]

实现：

```cpp
Eigen::Quaterniond q_BI =
    eulerAngleToQuaternion(
        imu_options.body_to_imu_rotation * D2R);

Eigen::Matrix3d R_WB =
    solution.pose.getRotation()
        .toImplementation()
        .toRotationMatrix();

Eigen::Matrix3d R_NI =
    R_NW * R_WB * q_BI.toRotationMatrix();

Eigen::Quaterniond q_NI(R_NI);
q_NI.normalize();

Eigen::Vector3d rpy =
    quaternionToEulerAngle(q_NI) * R2D;
```

NAV：

```text
roll  = rpy(0)
pitch = rpy(1)
yaw   = rpy(2)
```

**不额外把 yaw wrap 到 `[0, 360)`。**

原因是现有 GRLINS NAV writer 也是直接输出内部 Euler degree，不额外改变角度表示；若后续 evaluator 明确要求其他范围，再统一处理。

---

## 18. NAV 记录失败不得影响估计器

`recordNav()` 必须满足：

```text
Recorder disabled      → return
solution.coordinate空  → return + warning once
非有限数值             → skip + warning
```

不得：

```cpp
CHECK(...)
LOG(FATAL)
throw
```

NAV 是实验记录功能，不应该因为输出异常把 GLINS estimator 杀死。

---

# Part C. Timing Instrumentation

## 19. Timing 的两级结构

记录两类 event。

### Detail

真正用于模块性能统计，尽量互不嵌套：

```text
initialization
lidar_initialization
lidar_preprocess
gnss_aux_rtk
gnss_factor
lidar_factor
optimization
gnss_post
lidar_post
marginalization
```

### Coarse

只用于 sanity check：

```text
lidar_frontend_total
backend_add
backend_estimate
```

Coarse timer 与 detail timer 会存在父子重叠，因此：

```text
Coarse 不能与 Detail 一起求和
```

由于 GLINS 本身为多线程，而且 Ceres 保留当前配置，`module_sum` 只作为辅助统计；正式 Total running time 后续以整个 GLINS 进程 CPU time 为主。

---

## 20. `initialization`

修改：

```text
src/fusion/rtk_imu_lidar_rrr_estimator.cpp
```

函数：

```cpp
RtkImuLidarRrrEstimator::addMeasurement()
```

在整个：

```cpp
if (!gnss_imu_initializer_->finished()) {
    ...
}
```

初始化分支入口加入：

```cpp
ScopedModuleTimer timer(
    TimingModule::Initialization,
    measurement.timestamp,
    triggerFromMeasurement(measurement),
    TimingScope::Detail);
```

一条 IMU/GNSS measurement 对 initializer 的实际处理都会形成一个 event。

初始化总 active time：

\[
T_{init}=\sum_i t_{initialization,i}.
\]

这不是“从第一条数据到初始化完成”的 rosbag wall-clock，而是真正执行初始化代码的累计时间。

---

## 21. `lidar_initialization`

同一函数中：

```cpp
if (measurement.lidar) {
  if (!lidar_initialized_) {
    ScopedModuleTimer timer(
        TimingModule::LidarInitialization,
        measurement.lidar->timefinal,
        TimingTrigger::Lidar);

    return lidarInitialization(measurement.lidar);
  }
}
```

需要单独统计它，因为 `lidarInitialization()` 包括：

- yaw covariance gate；
- 第一帧 deskew；
- first LiDAR state 插入；
- point-cloud world transform；
- map build；
- voxel map build。

---

## 22. `lidar_preprocess`

修改：

```text
src/fusion/multisensor_estimating.cpp
```

函数：

```cpp
MultiSensorEstimating::runLidarFrontend()
```

只包住：

```cpp
tree_handler_->processLidar(scan);
```

记录：

```text
module         = lidar_preprocess
data_timestamp = scan->timefinal
trigger        = lidar
scope          = detail
```

当前 `TreeHandler::processLidar()` 主要包含 VoxelGrid downsampling 和 point covariance calculation，因此名称使用 `lidar_preprocess`，不套用 GRLINS 的 `LiDAR DA`。

---

## 23. `lidar_frontend_total`（Coarse）

仍在 `runLidarFrontend()` 中，对一整个有效 LiDAR scan 的 frontend iteration 建立 parent timer，范围包括：

```text
copy measurement
processLidar
plane visualization data preparation
send scan to backend
map visualization data preparation
current cloud transform / visualization output preparation
callbacks
```

记录：

```text
scope  = coarse
module = lidar_frontend_total
```

它回答的是：

> 当前保持所有 visualization output 不变时，一帧 LiDAR 在 frontend thread 总共花了多少 wall time？

它与 `lidar_preprocess` 重叠，只做诊断，不能加入 detail module sum。

---

## 24. `gnss_aux_rtk`

修改：

```text
src/fusion/rtk_imu_lidar_rrr_estimator.cpp
```

函数：

```cpp
RtkImuLidarRrrEstimator::addMeasurement()
```

对：

```cpp
if (coordinate_ &&
    ambiguity_covariance_estimator_->addMeasurement(measurement)) {
  ambiguity_covariance_estimator_->estimate();
}
```

单独计时：

```text
module  = gnss_aux_rtk
trigger = gnss
scope   = detail
```

这是 GLINS RRR 特有的重要计算，因为后续 ambiguity fixing 使用该辅助 RTK estimator 提供的 ambiguity covariance。

---

## 25. `gnss_factor`

同一函数中，完成 rover/reference 对齐后：

```cpp
if (measurement_align_.get(
        rtk_options_.max_age, rov, ref)) {
  return addGnssMeasurementAndState(rov, ref);
}
```

包住：

```cpp
addGnssMeasurementAndState(rov, ref)
```

记录：

```text
module         = gnss_factor
data_timestamp = rov.timestamp
trigger        = gnss
scope          = detail
```

该函数主要包括：

```text
phase/code rearrangement
DD carrier/code pairing
cycle-slip detection
GNSS state insertion
ambiguity parameter blocks
frequency blocks
DD pseudorange residuals
DD phase residuals
Doppler residuals
relative frequency/ambiguity residuals
DOP
```

---

## 26. `lidar_factor`

LiDAR 已初始化时，包住：

```cpp
addLidarMeasurementAndState(measurement.lidar)
```

记录：

```text
module         = lidar_factor
data_timestamp = scan->timefinal
trigger        = lidar
scope          = detail
```

该函数主要包括：

```text
point time sorting
IMU propagation / point-wise deskew
state propagation
LiDAR state insertion
keyframe selection
plane parameter/residual construction
```

---

## 27. `backend_add`（Coarse）

修改：

```text
src/fusion/multisensor_estimating.cpp
```

函数：

```cpp
MultiSensorEstimating::processEstimator()
```

对：

```cpp
estimator_->addMeasurement(measurement)
```

包 parent timer：

```text
module  = backend_add
scope   = coarse
trigger = measurement type
```

这个 timer 会包含内部 `initialization / lidar_initialization / gnss_aux_rtk / gnss_factor / lidar_factor` 中的一个或多个，因此只用于 sanity check。

---

## 28. `optimization`

修改：

```text
src/fusion/rtk_imu_lidar_rrr_estimator.cpp
```

函数：

```cpp
RtkImuLidarRrrEstimator::estimate()
```

在 `optimize()` 前只读取：

```cpp
const IdType pending_state_type =
    states_[latest_state_index_].id.type();
```

映射：

```text
gPose → gnss
lPose → lidar
```

然后：

```cpp
{
  ScopedModuleTimer timer(
      TimingModule::Optimization,
      states_[latest_state_index_].timestamp,
      trigger,
      TimingScope::Detail);

  optimize();
}
```

后续可以分别统计：

```text
optimization / gnss
optimization / lidar
```

---

## 29. `gnss_post`

包住：

```cpp
if (new_state_type == IdType::gPose) {
    ...
}
```

记录：

```text
module  = gnss_post
trigger = gnss
scope   = detail
```

该区域包含：

```text
GNSS outlier rejection / rejection statistics
ambiguity covariance extraction
LAMBDA ambiguity resolution
solution Fixed/Float status update
continuous-unfix check
ambiguity reset logic
```

---

## 30. `lidar_post`

包住：

```cpp
if (new_state_type == IdType::lPose) {
    updateCloudMap(...)
    updateLandmarks()
    rejectExcessiveResiduals(...)
    eraseEmptyLandmarks()
    updateVoxelKey()
    updateVoxelMap(...)
    mapSlide(...)
}
```

记录：

```text
module  = lidar_post
trigger = lidar
scope   = detail
```

当前 `updateCloudMap()` 即使主要面向 visualization，也属于现有算法实际执行路径；按照本次实验原则，不关闭、不删去，直接计入 `lidar_post`。

---

## 31. `marginalization`

单独包：

```cpp
marginalization(new_state_type);
```

记录：

```text
module  = marginalization
trigger = gnss / lidar
scope   = detail
```

FGO 滑窗边缘化是 GLINS 与 MSCKF 计算结构差异的重要来源，因此必须单独保留。

---

## 32. `backend_estimate`（Coarse）

在：

```cpp
MultiSensorEstimating::processEstimator()
```

对：

```cpp
estimator_->estimate()
```

包：

```text
module  = backend_estimate
scope   = coarse
```

对于 RRR，它理论上大致覆盖：

```text
optimization
+
gnss_post 或 lidar_post
+
marginalization
+
少量 state bookkeeping
```

因此可做 sanity check：

\[
T_{backend\_estimate}
\approx
T_{optimization}
+
T_{post}
+
T_{marginalization}
+
T_{bookkeeping}.
\]

不能与这些 detail 项一起求和。

---

# Part D. Timing 文件格式

## 33. `timing_events.csv`

固定字段建议：

```text
event_id,scope,module,trigger,data_timestamp,gps_sow,wall_start_ms,duration_ms,thread_id
```

示例：

```text
0,detail,lidar_preprocess,lidar,1745893401.213022,303619.2130,12534.218431,4.832171,147829...
1,detail,lidar_factor,lidar,1745893401.265058,303619.2651,12542.926110,16.523914,938124...
2,coarse,backend_add,lidar,1745893401.265058,303619.2651,12542.924830,16.529007,938124...
3,detail,optimization,lidar,1745893401.265058,303619.2651,12559.463312,31.284551,938124...
```

其中：

```text
data_timestamp : fixed, 9 decimals
gps_sow        : fixed, 10 decimals
wall_start_ms  : fixed, 6 decimals
duration_ms    : fixed, 6 decimals
```

---

## 34. Detail module sum 的定义

后续若需要计算 `module_sum_s`，只能累加：

```text
initialization
lidar_initialization
lidar_preprocess
gnss_aux_rtk
gnss_factor
lidar_factor
optimization
gnss_post
lidar_post
marginalization
```

绝不加入：

```text
lidar_frontend_total
backend_add
backend_estimate
```

因为后三项是父级计时。

但由于 GLINS 保留多线程架构和当前 Ceres 多线程配置：

```text
module_sum_s 不是最终 Total running time 的唯一依据。
```

正式总计算量下一阶段使用 Linux process CPU time；`module_sum_s` 用于模块分解和一致性检查。

---

# Part E. 修改点汇总

## 35. `include/gici/utility/experiment_recorder.h`

新增：

```text
TimingScope
TimingTrigger
TimingModule
TimingEvent
NavRecord
ExperimentRecorder
ScopedModuleTimer
```

不依赖 ROS。

## 36. `src/utility/experiment_recorder.cpp`

实现：

```text
GLINS_RESULT_DIR 读取
enabled/disabled
steady_clock timing
UTC→GPS SOW
Solution→NAV
thread-safe vector push
glins.nav 写出
timing_events.csv 写出
idempotent flush
enum→string
```

## 37. `src/fusion/multisensor_estimating.cpp`

新增 include：

```cpp
#include "gici/utility/experiment_recorder.h"
```

修改：

```text
updateSolution()
    → recordNav()

runLidarFrontend()
    → lidar_frontend_total [coarse]
    → lidar_preprocess [detail]

processEstimator()
    → backend_add [coarse]
    → backend_estimate [coarse]

~MultiSensorEstimating()
    → threads join 后 flush()
```

不改变线程创建、队列、callback 和 visualization output。

## 38. `src/fusion/rtk_imu_lidar_rrr_estimator.cpp`

新增 include：

```cpp
#include "gici/utility/experiment_recorder.h"
```

插入：

```text
addMeasurement()
    → initialization
    → gnss_aux_rtk
    → lidar_initialization
    → gnss_factor
    → lidar_factor

estimate()
    → optimization
    → gnss_post
    → lidar_post
    → marginalization
```

不修改这些函数内部原算法内容。

---

# Part F. 实现顺序

建议严格按：

```text
A. Recorder 基础设施
        ↓
B. NAV
        ↓
C. Timing coarse
        ↓
D. Timing detail
        ↓
E. 60 s building02 验证
```

Recorder 阶段先只测试 `GLINS_RESULT_DIR / flush / timing_events.csv`。

NAV 阶段只加入 `recordNav()`，先验证轨迹格式和坐标语义。

Timing 阶段先加：

```text
lidar_frontend_total
backend_add
backend_estimate
```

再加入 detail 模块，便于出现问题时快速定位。

---

# Part G. 验证方案

## 39. NAV 验证

用 building02 约 60 s。

必须检查：

```text
每行 = 11列
第一列 = 0
median ΔSOW ≈ 0.2 s
无重复/倒序时间
latitude ≈ 30.x
longitude ≈ 114.x
latitude/longitude 小数位 = 10
```

与 ROS 输出交叉检查：

```text
NAV latitude/longitude/height
    ↔ NavSatFix

NAV vn = odom.vy
NAV ve = odom.vx
NAV vd = -odom.vz
```

姿态重点排查：

```text
90°固定偏差
180°固定偏差
roll/pitch符号错误
yaw方向错误
```

可用直行段：

\[
heading_v=atan2(v_e,v_n)
\]

与 NAV yaw 做粗对照。

---

## 40. Timing event 验证

数量级预期：

```text
lidar_preprocess ≈ LiDAR scan数
lidar_factor     ≈ LiDAR初始化后的scan数
gnss_factor      ≈ 成功配对的1 Hz GNSS epoch数
optimization     ≈ 成功触发 estimate 的 GNSS/LiDAR update 数
```

全部 duration：

```text
>= 0
finite
```

不应有：

```text
NaN
Inf
负值
```

Coarse/detail 一致性：

```text
backend_estimate
≈
optimization
+ post
+ marginalization
+ 少量未细分 bookkeeping
```

长期若 detail child 明显大于 parent，应检查 scope 是否重复覆盖。

`thread_id + wall_start_ms` 应能看出：

```text
LiDAR frontend thread
与
backend thread
```

存在并发。

---

## 41. Instrumentation 开销验证

最终做一次 60 s A/B：

```text
A: 同一 executable，GLINS_RESULT_DIR unset
B: 同一 executable，GLINS_RESULT_DIR set
```

其他配置、ROS output、rosbag rate 完全一致，均不启动 RViz。

后续用 process CPU time 评估：

\[
overhead=(CPU_B-CPU_A)/CPU_A.
\]

若 instrumentation 开销明显超过约 1–2%，优先检查：

```text
字符串动态分配
mutex竞争
event数量
文件写入时机
```

而不是修改算法或关闭 visualization output。

---

# 42. 本阶段完成判据

完成后应满足：

```text
[1] 未设置 GLINS_RESULT_DIR 时，GLINS 行为与当前 baseline 一致。

[2] 设置 GLINS_RESULT_DIR 后，正常退出生成：
    glins.nav
    timing_events.csv

[3] glins.nav：
    - 11列
    - 第一列固定0
    - GPS SOW
    - latitude longitude height
    - vn ve vd
    - roll pitch yaw
    - 5 Hz
    - fixed setprecision(10)
    - latitude/longitude 10位小数

[4] NAV 的时间、位置、速度、姿态语义全部通过独立验证。

[5] timing_events.csv 包含：
    - data_timestamp
    - gps_sow
    - wall_start_ms
    - duration_ms
    - trigger
    - thread_id
    - detail/coarse scope

[6] detail 模块完整：
    initialization
    lidar_initialization
    lidar_preprocess
    gnss_aux_rtk
    gnss_factor
    lidar_factor
    optimization
    gnss_post
    lidar_post
    marginalization

[7] 未修改现有算法参数、Ceres线程、visualization output和数据调度逻辑。

[8] 正式运行时不关闭 visualization output，只是不启动 RViz。

[9] instrumentation overhead 经 60 s A/B 验证处于可接受范围。
```

完成本阶段后，再修改 `run_robnav_rrr.py`，实现：

```text
自动创建 run directory
注入 GLINS_RESULT_DIR
正式序列 process CPU total time
等待 backend drain
timing_summary.csv
run_summary.csv
四组数据批处理
```

这样“原始记录”和“实验汇总”分开，后续论文效率表格如何调整都不需要重新设计底层计时。
