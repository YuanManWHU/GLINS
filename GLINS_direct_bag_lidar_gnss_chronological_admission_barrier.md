# GLINS Direct-Bag：LiDAR–GNSS Chronological Admission Barrier 实现方案

## 1. 目标

当前 direct-bag 已经完成两层稳定性控制：

```text
1. addin backpressure
   HIGH = 100
   LOW  = 40

2. backend data-time lag backpressure
   HIGH = 1.5 s
   LOW  = 0.8 s

3. two-stage EOF
   formal EOF
      → ingress/frontend quiescent
      → notifyInputFinished()
      → final alignment/backend drain
```

120 s `building02` 已经能够稳定运行，不再出现历史 IMU 不足、CHECK/FATAL。

但是 ROS 1× baseline 与 direct-bag 的 `timing_events.csv` 显示：

> direct-bag 中，较新的 rover GNSS state update 仍可能越过尚未完成 frontend/backend admission 的较老 LiDAR。

典型例子是 GPS SOW 181373：

```text
ROS 1× baseline：
LiDAR 181372.713
LiDAR 181372.813
GNSS  181373 reference
LiDAR 181372.913
GNSS  181373 rover → gnss_factor / optimization
```

而 direct-bag 中出现：

```text
LiDAR 181372.713
GNSS  181373 reference
GNSS  181373 rover → gnss_factor / optimization
LiDAR 181372.813 / 181372.913 后续才进入 backend
```

这与此前 NAV 在约 `181372.31 ~ 181373.91` 的短时米级差异高度吻合。

本阶段目标：

\[
\boxed{
t_{L,\mathrm{timebase}} < t_{G,\mathrm{rover}}
\Longrightarrow
\text{rover GNSS 不允许在该 LiDAR 完成 frontend 并进入有序 admission 之前越过它}
}
\]

注意：

- **只约束 rover GNSS 主状态更新**；
- reference GNSS 保持现有 direct-to-backend 行为；
- 不把整个 pipeline 串行化；
- 不改变 ROS 1× baseline；
- 不改变现有 addin/backend backpressure；
- 不修改 RRR / IMU / LiDAR / GNSS 数学模型。

---

# 2. 为什么当前 backpressure 还不能解决顺序问题

当前 direct-bag 已经把 backlog 控制住，但各类 measurement 的内部路径不同：

```text
IMU
  raw input
  → addin
  → measurement thread
  → estimator IMU buffer

Reference GNSS
  raw input
  → addin
  → measurement thread
  → backend measurements_ 直接入队

Rover GNSS
  raw input
  → addin
  → measurement thread
  → measurement_align_buffer_
  → backend

LiDAR
  raw input
  → addin
  → LiDAR frontend queue
  → processLidar()
  → processed LiDAR 再次进入 addin
  → measurement thread
  → measurement_align_buffer_
  → backend
```

所以 LiDAR 比 rover GNSS 多经过：

```text
LiDAR frontend
+
processed LiDAR 再入 addin
```

两个异步阶段。

因此即使原始 merged `rosbag::View` 是严格按 record time 输入：

```text
LiDAR(t-0.2)
LiDAR(t-0.1)
GNSS(t)
```

内部 arrival 仍可能变成：

```text
GNSS(t) 已到 alignment/backend
LiDAR(t-0.2) 仍在 frontend
LiDAR(t-0.1) 仍在 frontend/addin
```

backpressure 只能保证这种错位不无限增长，不能保证 chronological equivalence。

---

# 3. 当前代码中还有一个必须处理的结构点

`handleNonTimePropagationSensors()` 当前逻辑中：

```cpp
if (enable_input_align_) {
  if (!needTimeAlign(type_)) {
    measurement_align_buffer_.push_back(data);
  }
  else {
    // timestamp ordered insertion
  }
}
```

而 `needTimeAlign()` 当前只对 camera-related estimators 返回 true，`RtkImuLidarRrr` 不在其中。

因此对当前 RRR：

```text
measurement_align_buffer_
```

默认并不是严格按 timestamp 排序，而是按 processed measurement 的实际 arrival 顺序 `push_back()`。

这意味着仅增加：

```text
“GNSS 看到更老 pending LiDAR 就等待”
```

还不够。

因为较老 processed LiDAR 之后进入 alignment 时，仍可能被 append 在已经等待的 GNSS 后面。

所以本阶段必须同时实现：

```text
A. direct-only ordered alignment insertion
B. pending LiDAR admission tracker
C. rover GNSS chronological barrier
```

三者缺一不可。

---

# 4. 总体设计

新逻辑只在 direct-bag executable 显式开启：

```cpp
estimator->enableDirectChronologicalAdmissionBarrier(true);
```

默认：

```cpp
false
```

因此原 ROS 1× 路径完全不变。

启用后：

```text
raw LiDAR 进入 addin
    ↓
登记为 pending LiDAR admission
    ↓
LiDAR frontend
    ↓
processed LiDAR 回到 addin
    ↓
measurement thread
    ↓
按 timestamp 插入 alignment buffer
    ↓
从 pending admission tracker 中移除
    ↓
releaseAlignedMeasurements()
```

若 rover GNSS `t_g` 正准备 release：

```text
是否还有 pending LiDAR timebase < t_g？
    ├─ 是 → rover GNSS 暂停 admission
    └─ 否 → 继续
```

同时 alignment buffer 本身在 direct mode 下按 timestamp 排序，因此后到的旧 processed LiDAR 会插到 GNSS 前面。

最终：

```text
older LiDAR
→ backend
→ newer rover GNSS
```

---

# 5. 修改文件

只修改：

```text
include/gici/fusion/multisensor_estimating.h
src/fusion/multisensor_estimating.cpp
ros_wrapper/src/gici/src/gici_robnav_bag_main.cpp
```

不修改：

```text
rtk_imu_lidar_rrr_estimator.cpp
imu_estimator_base.cpp
imu_error.cpp
tree_handler.cpp
ros_stream.cpp
DataIntegration
YAML
run_robnav_rrr_direct.py
ROS 1× runner
```

---

# 6. Header：新增 direct chronological barrier API

文件：

```text
include/gici/fusion/multisensor_estimating.h
```

增加：

```cpp
#include <set>
```

public 区增加：

```cpp
struct ChronologicalAdmissionStats {
  size_t pending_lidar_peak = 0;
  size_t rover_gnss_block_count = 0;
  double max_block_gap_s = 0.0;
};

// Direct-bag only. Default is disabled, so ROS behavior is unchanged.
void enableDirectChronologicalAdmissionBarrier(bool enable);

ChronologicalAdmissionStats chronologicalAdmissionStats();
```

private/protected 增加：

```cpp
// Direct-bag chronological admission barrier.
std::atomic<bool> direct_chronological_admission_enabled_{false};

std::mutex mutex_lidar_admission_;

// Raw LiDAR remains here from first ingress until its processed scan has
// entered the timestamp-ordered alignment buffer.
std::multiset<double> pending_lidar_admission_timebases_;

size_t peak_pending_lidar_admission_ = 0;
size_t rover_gnss_block_count_ = 0;
double max_rover_gnss_block_gap_s_ = 0.0;

// Avoid counting the same blocked rover epoch on every retry.
double last_counted_blocked_rover_timestamp_ = -1.0;
```

再增加 helper：

```cpp
void registerPendingLidarAdmission(double timebase);

void completePendingLidarAdmission(double timebase);

bool getOldestPendingLidarAdmission(
    double* timebase);

bool hasPendingLidarOlderThan(
    double timestamp,
    double* oldest_timebase = nullptr);

void clearDirectAdmissionState();
```

---

# 7. 为什么使用 `std::multiset<double>`

这里不要只保存：

```text
latest raw LiDAR timestamp
```

因为 chronological barrier 关心的是：

```text
最老的仍未完成 frontend→alignment admission 的 LiDAR
```

定义：

\[
t_{L,\min}^{pending}
=
\min \{
t_{L,i}^{timebase}
\mid
L_i \text{ 尚未进入 alignment}
\}
\]

所以需要支持：

```text
insert(timebase)
erase(timebase)
minimum()
```

`std::multiset<double>` 足够，而且 LiDAR 只有 10 Hz，元素规模非常小。

---

# 8. Raw LiDAR ingress：先登记，再允许进入 addin

当前：

```cpp
void MultiSensorEstimating::estimatorDataCallback(
    EstimatorDataCluster& data)
{
  ...
  measurement_addin_buffer_.push_back(data);
}
```

修改为：

```cpp
void MultiSensorEstimating::estimatorDataCallback(
    EstimatorDataCluster& data)
{
  if (estimatorDataIllegal(data)) {
    LOG(ERROR) << "Received illegal estimator data cluster!";
    return;
  }

  // Register raw LiDAR before making it visible to the asynchronous
  // measurement/frontend pipeline.
  if (direct_chronological_admission_enabled_.load() &&
      data.lidar &&
      data.lidar->need_frontend) {
    registerPendingLidarAdmission(
        data.lidar->timebase);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_addin_);
    measurement_addin_buffer_.push_back(data);
    peak_addin_size_ =
        std::max(peak_addin_size_,
                 measurement_addin_buffer_.size());
  }
}
```

### 为什么必须在 `push_back()` 前登记

否则可能出现：

```text
raw LiDAR 已经被 measurement thread pop
但 pending tracker 还没记录
```

此时 rover GNSS barrier 会短暂看不到该 LiDAR。

所以：

```text
register
→ addin visible
```

顺序必须固定。

---

# 9. Pending LiDAR 何时才算“完成 admission”

不能在：

```text
LiDAR frontend processLidar() 完成
```

时立即移除。

也不能在：

```text
processed LiDAR 调用 estimatorDataCallback()
```

时移除。

因为此时 processed LiDAR 可能仍在：

```text
measurement_addin_buffer_
```

中，而 rover GNSS 已经可能在 alignment。

正确完成点必须是：

> **processed LiDAR 已经真正插入 `measurement_align_buffer_` 之后。**

只有此时才能保证：

```text
它已经进入 rover GNSS 同一个 chronological admission stage。
```

---

# 10. Direct mode 下 alignment buffer 必须强制 timestamp-ordered insertion

修改：

```cpp
handleNonTimePropagationSensors()
```

当前 RRR 默认：

```cpp
measurement_align_buffer_.push_back(data);
```

direct barrier 开启时改为专用排序插入：

```cpp
void MultiSensorEstimating::handleNonTimePropagationSensors(
    EstimatorDataCluster& data)
{
  const bool direct_ordered =
      direct_chronological_admission_enabled_.load();

  {
    std::lock_guard<std::mutex> lock(mutex_align_);

    if (direct_ordered) {
      auto pos = std::upper_bound(
          measurement_align_buffer_.begin(),
          measurement_align_buffer_.end(),
          data.timestamp,
          [](double timestamp,
             const EstimatorDataCluster& item) {
            return timestamp < item.timestamp;
          });

      measurement_align_buffer_.insert(pos, data);
    }
    else {
      // Keep the original upstream logic unchanged.
      if (enable_input_align_) {
        ...
      }
      else {
        measurement_align_buffer_.push_back(data);
      }
    }
  }

  // Processed LiDAR has now reached the same chronological stage as rover GNSS.
  if (direct_ordered &&
      data.lidar &&
      !data.lidar->need_frontend) {
    completePendingLidarAdmission(
        data.lidar->timebase);
  }

  releaseAlignedMeasurements();
}
```

---

# 11. 为什么 direct mode 不复用原来的 latency-rejection insertion 分支

原来的 ordered insertion 分支包含类似：

```cpp
if (data.timestamp <
    measurement_align_buffer_.back().timestamp -
    2.0 * input_align_latency_) {
  LOG(WARNING) << "Throughing data ... latency is too large";
}
```

这个逻辑是针对在线 sensor latency 设计的。

direct mode 中，processed LiDAR 晚到不是异常 sensor latency，而是：

```text
frontend asynchronous processing
```

的预期结果。

所以 direct chronological mode 应：

```text
始终允许按 timestamp 插入
```

不能因为它比当前 buffer back 老 0.2~1 s 就丢掉。

因此 direct ordered insertion 必须单独写，不能直接把：

```text
needTimeAlign(type_)
```

强行改成 true。

---

# 12. `completePendingLidarAdmission()` 实现

建议：

```cpp
void MultiSensorEstimating::completePendingLidarAdmission(
    double timebase)
{
  if (!direct_chronological_admission_enabled_.load()) {
    return;
  }

  constexpr double kTimestampTolerance = 1e-6;

  std::lock_guard<std::mutex> lock(
      mutex_lidar_admission_);

  auto it = pending_lidar_admission_timebases_.lower_bound(
      timebase - kTimestampTolerance);

  if (it == pending_lidar_admission_timebases_.end() ||
      std::abs(*it - timebase) > kTimestampTolerance) {
    LOG(WARNING)
        << "Unable to match pending LiDAR admission at "
        << std::fixed << timebase;
    return;
  }

  pending_lidar_admission_timebases_.erase(it);
}
```

理论上 `timebase` 是原值 copy，可以 exact match。

加 `1e-6 s` tolerance 只是防止以后 conversion 中发生微小浮点变化。

---

# 13. Frontend 主动丢弃 LiDAR 时必须同步清 tracker

当前 `runLidarFrontend()` 至少存在：

```text
lidar_role != Front
LiDAR loop back
```

两条 pop 后不生成 processed LiDAR 的路径。

如果 pending tracker 不清除，该 LiDAR 会永远阻塞之后 rover GNSS。

因此凡是：

```cpp
lidar_frontend_measurements_.pop_front();
```

且确定该 raw LiDAR **不会再生成 processed LiDAR** 时，都必须：

1. 保存其 `timebase`；
2. pop / unlock；
3. `completePendingLidarAdmission(timebase)`；
4. log reason。

例如：

```cpp
const double dropped_timebase =
    front_measurement.lidar->timebase;

lidar_frontend_measurements_.pop_front();
mutex_lidar_input_.unlock();

completePendingLidarAdmission(
    dropped_timebase);

LOG(WARNING)
    << "Dropped LiDAR removed from chronological admission tracker at "
    << std::fixed << dropped_timebase;
```

不要在持有：

```text
mutex_lidar_input_
```

时再拿：

```text
mutex_lidar_admission_
```

避免无必要的 nested lock。

---

# 14. Rover GNSS chronological barrier

核心逻辑放在：

```cpp
releaseAlignedMeasurements()
```

而不是 direct reader。

原因：

```text
reader 仍需继续读取 future IMU；
barrier 应只阻止 backend admission，
不能阻止原始数据继续进入 pipeline。
```

因此不会因为等待旧 LiDAR 而饿死它需要的 future IMU。

---

# 15. 如何识别需要阻塞的 GNSS

只阻塞：

```text
rover GNSS
```

不阻塞：

```text
reference GNSS
```

当前 reference GNSS 本来就有专门的：

```cpp
else if (data.gnss &&
         data.gnss_role == GnssRole::Reference) {
  measurements_.push_back(data);
}
```

路径。

这一点不要改。

理由：

- reference measurement 本身不直接触发主状态 FGO update；
- ROS baseline 中也允许 reference GNSS 在部分较老 LiDAR 之前进入 backend；
- 真正产生 `gnss_factor / optimization` 的 rover GNSS 才需要 chronological barrier。

实现时用当前枚举中对应 rover 的 role；若本地枚举名称不是 `GnssRole::Rover`，按实际定义替换。

---

# 16. `releaseAlignedMeasurements()` 的 direct-mode 逻辑

建议结构：

```cpp
void MultiSensorEstimating::releaseAlignedMeasurements()
{
  std::lock_guard<std::mutex> align_lock(
      mutex_align_);

  const bool direct_ordered =
      direct_chronological_admission_enabled_.load();

  for (auto it = measurement_align_buffer_.begin();
       it != measurement_align_buffer_.end();) {

    if (!input_finished_.load() &&
        measurement_align_buffer_.back().timestamp -
            measurement_align_buffer_.front().timestamp <
            input_align_latency_) {
      break;
    }

    EstimatorDataCluster& measurement = *it;

    // ------------------------------------------------------------
    // A. Pending older raw/frontend LiDAR blocks rover GNSS
    // ------------------------------------------------------------
    if (direct_ordered &&
        measurement.gnss &&
        measurement.gnss_role != GnssRole::Reference) {

      double oldest_pending_lidar = 0.0;

      if (hasPendingLidarOlderThan(
              measurement.timestamp,
              &oldest_pending_lidar)) {

        const double gap =
            measurement.timestamp -
            oldest_pending_lidar;

        recordChronologicalBlock(
            measurement.timestamp,
            gap);

        // Buffer is timestamp-ordered in direct mode.
        // All following measurements are no older than this GNSS.
        break;
      }
    }

    // ------------------------------------------------------------
    // B. IMU coverage
    // ------------------------------------------------------------
    double required_imu_timestamp =
        measurement.timestamp;

    if (measurement.lidar) {
      required_imu_timestamp =
          measurement.lidar->timefinal;
    }

    double latest_imu_timestamp;
    {
      std::lock_guard<std::mutex> input_lock(
          mutex_input_);
      latest_imu_timestamp =
          latest_imu_timestamp_;
    }

    if (estimatorTypeContains(
            SensorType::IMU, type_) &&
        required_imu_timestamp >
            latest_imu_timestamp) {

      if (direct_ordered) {
        // The direct buffer is chronologically ordered.
        // Do not allow newer rover GNSS to bypass this older item.
        break;
      }

      // Preserve original ROS behavior.
      ++it;
      continue;
    }

    // ------------------------------------------------------------
    // C. Normal backend admission
    // ------------------------------------------------------------
    {
      std::lock_guard<std::mutex> input_lock(
          mutex_input_);

      measurements_.push_back(measurement);

      peak_backend_size_ =
          std::max(peak_backend_size_,
                   measurements_.size());

      ...
    }

    it = measurement_align_buffer_.erase(it);
  }
}
```

---

# 17. 为什么 direct mode 下 IMU coverage 不满足时要 `break`

原代码是：

```cpp
if (required_imu_timestamp > latest_imu_timestamp) {
  ++it;
  continue;
}
```

这允许：

```text
older LiDAR 尚缺 scan-end IMU
        ↓
跳过它
        ↓
尝试 release 后面的 newer GNSS
```

这正是 chronological equivalence 不希望发生的事情。

在 direct mode 下，alignment buffer 已经 timestamp-ordered，因此：

> 当前最老 measurement 尚未具备进入 backend 的条件时，后面的 newer measurement 也不应越过它。

所以：

```text
direct mode → break
ROS mode    → 保持原 continue
```

既保证 direct chronology，又不改变 upstream ROS baseline。

---

# 18. 为什么不能只在 direct reader 遇到 GNSS 时 sleep

看起来可以做：

```text
reader 读到 GNSS(t)
↓
等待所有 LiDAR<t frontend 完成
↓
再 feed GNSS
```

但不推荐。

原因：

1. LiDAR `timefinal` 可能仍需要 GNSS 后面附近的 IMU；
2. multi-bag record time 与 sensor header time 不一定严格一致；
3. reader sleep 会阻止 future IMU 进入 pipeline；
4. 可能形成 feeder-side deadlock；
5. 会把 input ordering policy 和 frontend processing 强耦合。

因此正确位置是：

```text
backend admission
```

而不是：

```text
raw bag reader
```

reader 继续按最大可持续速度读取，barrier 只控制“谁可以先进入 backend”。

---

# 19. Direct runner：只负责开启 barrier

在：

```text
gici_robnav_bag_main.cpp
```

拿到 `MultiSensorEstimating` 后，formal input 开始前：

```cpp
estimator->enableDirectChronologicalAdmissionBarrier(
    true);
```

放在：

```text
获取 estimator
之后

正式 formal rosbag loop
之前
```

即可。

不要增加新的 CLI 参数。

这个 executable 本身就是：

```text
RobNav + RRR direct-bag benchmark
```

所以固定开启即可。

---

# 20. Backpressure 保持当前实现

以下完全不改：

```text
addin:
  HIGH = 100
  LOW  = 40

backend lag:
  HIGH = 1.5 s
  LOW  = 0.8 s
```

chronological barrier 与 backpressure 是两类不同机制：

```text
backpressure
→ 保证 bounded throughput / bounded lag

chronological barrier
→ 保证 backend measurement ordering semantics
```

不要混成一套阈值。

---

# 21. Two-stage EOF 保持当前实现

继续：

```text
formal rosbag EOF
    ↓
等待：
addin empty
measurement not busy
LiDAR frontend queue empty
LiDAR frontend not busy
    ↓
notifyInputFinished()
    ↓
final alignment flush
    ↓
pipelineIdle()
```

在进入：

```text
notifyInputFinished()
```

之前，正常情况下：

```text
pending_lidar_admission_timebases_
```

也应该已经为空。

建议把：

```text
pending tracker empty
```

加入：

```cpp
readyForFinalAlignmentFlush()
```

的条件。

即：

```cpp
bool pending_lidar_empty = true;

if (direct_chronological_admission_enabled_.load()) {
  std::lock_guard<std::mutex> lock(
      mutex_lidar_admission_);

  pending_lidar_empty =
      pending_lidar_admission_timebases_.empty();
}

return addin_empty &&
       lidar_empty &&
       pending_lidar_empty &&
       ...;
```

这样 EOF 前若 tracker 漏清理，会直接暴露为 quiesce timeout，而不是悄悄错误 release GNSS。

---

# 22. `pipelineIdle()` 也建议检查 tracker

同理，在 direct mode：

```text
pending_lidar_admission_timebases_.empty()
```

应是 full pipeline idle 的必要条件。

否则：

```text
queue 都空
但 tracker 仍残留
```

意味着 admission bookkeeping 有 bug。

---

# 23. reset / abnormal path

若 `resetProcessors()` 因 estimator diverged 被调用：

```text
pending LiDAR admission tracker
```

必须同步清空。

建议在 `resetProcessors()` 开头或末尾：

```cpp
clearDirectAdmissionState();
```

但只清：

```text
pending tracker / direct admission counters that represent live pipeline state
```

统计 counters 是否清零要区分：

- live state：必须清；
- run-level diagnostic stats：建议保留累计，不清。

可以拆成：

```cpp
clearPendingDirectAdmissions();
```

而不是把整个 stats 都 reset。

---

# 24. Chronological barrier 诊断统计

建议输出：

```text
chronological_barrier_block_count
chronological_barrier_max_gap_s
pending_lidar_admission_peak
pending_lidar_at_eof
```

定义：

```text
block_count
    rover GNSS epoch 首次因 pending older LiDAR 被阻塞的次数

max_gap_s
    max(t_gnss - oldest_pending_lidar_timebase)

pending_lidar_admission_peak
    tracker 最大元素数

pending_lidar_at_eof
    notifyInputFinished 前应为 0
```

不要每次 retry 都重复计数同一个 GNSS epoch。

例如：

```cpp
if (std::abs(
        measurement.timestamp -
        last_counted_blocked_rover_timestamp_) >
    1e-9) {

  ++rover_gnss_block_count_;

  last_counted_blocked_rover_timestamp_ =
      measurement.timestamp;
}
```

---

# 25. 不建议在正式 benchmark 中高频打印 barrier 日志

第一次 120 s 验证可以：

```cpp
LOG(INFO)
    << "[CHRONO_BARRIER] rover_gnss="
    << measurement.timestamp
    << " oldest_pending_lidar="
    << oldest_pending_lidar
    << " gap="
    << gap;
```

但正式 full benchmark 应关闭或降为：

```text
VLOG / summary only
```

避免日志 I/O 影响 wall time。

---

# 26. 第一轮验证：仍然使用 building02 120 s

实现后首先运行：

```bash
python3 ros_wrapper/src/gici/scripts/robnav/run_robnav_rrr_direct.py \
  building02 --duration 120
```

基本要求：

```text
run_valid = YES

IMU       = 24005
LiDAR     = 1200
rover     = 120
reference = 120

LiDAR frontend = 1200

无：
First IMU measurement ... not old enough
CHECK
FATAL
```

并继续检查：

```text
queue_peak_addin ≈ 100~少量 overshoot
backend_lag_max_s ≲ 2 s
```

chronological barrier 不应该破坏已经验证通过的稳定性机制。

---

# 27. 核心验证 1：GPS SOW 181373 的 backend 顺序

这是第一条必须检查的 correctness criterion。

修复后应接近 ROS baseline：

```text
LiDAR 181372.713
LiDAR 181372.813
reference GNSS 181373   # 可以在部分 LiDAR 前后穿插
LiDAR 181372.913
rover GNSS 181373
    → gnss_factor
    → optimization
```

关键不是 reference GNSS 的位置。

关键是：

\[
\boxed{
\text{rover GNSS 181373 的 gnss_factor/optimization 之前，
所有 timebase<181373 的 LiDAR 都应已 backend_add}
}
\]

---

# 28. 核心验证 2：全局 overtaking count

不要只人工看 181373。

对整个 120 s timing file 做自动检查。

定义 rover GNSS update 时刻：

```text
detail,gnss_factor,gnss
```

因为：

```text
只有真正触发 rover state update 的 GNSS 才会产生 gnss_factor。
```

对于每个：

\[
(t_g,\; w_g)
\]

其中：

```text
t_g = GNSS data timestamp
w_g = gnss_factor wall_start_ms
```

检查是否存在：

```text
backend_add,lidar
```

满足：

\[
t_l < t_g
\]

但是：

\[
w_l > w_g
\]

如果存在，说明：

> rover GNSS state update 时，仍有更老 LiDAR 后续才进入 backend。

期望：

```text
overtaking_count = 0
```

---

# 29. 推荐验证脚本

```python
import pandas as pd

df = pd.read_csv("timing_events.csv")

gnss = df[
    (df["scope"] == "detail") &
    (df["module"] == "gnss_factor") &
    (df["trigger"] == "gnss")
].copy()

lidar = df[
    (df["scope"] == "coarse") &
    (df["module"] == "backend_add") &
    (df["trigger"] == "lidar")
].copy()

violations = []

for _, g in gnss.iterrows():
    bad = lidar[
        (lidar["data_timestamp"] < g["data_timestamp"]) &
        (lidar["wall_start_ms"] > g["wall_start_ms"])
    ]

    if len(bad):
        violations.append({
            "gnss_timestamp": g["data_timestamp"],
            "gnss_sow": g["gps_sow"],
            "num_older_lidar_after_gnss": len(bad),
            "oldest_lidar_timestamp": bad["data_timestamp"].min(),
            "latest_lidar_timestamp": bad["data_timestamp"].max(),
        })

print("rover GNSS updates:", len(gnss))
print("overtaking epochs :", len(violations))

for v in violations[:20]:
    print(v)
```

### 判据

```text
overtaking epochs = 0
```

才认为 chronological admission barrier 真正生效。

---

# 30. 核心验证 3：NAV 一致性

修复前：

```text
稳定段很接近
但 181372.31 ~ 181373.91 有短时米级差异
```

修复后重新比较：

```text
ROS 1× baseline NAV
vs
direct-bag NAV
```

仍按 GPS SOW 插值/对齐。

重点检查：

```text
3D position median
P95
RMSE
max

181372 ~ 181374 局部最大差异
```

期望：

> 原来的约 1–2 m initialization transient 明显缩小或消失。

如果 overtaking 已经变为 0，但该段仍有明显差异，再继续检查：

```text
初始化阶段其他 measurement ordering
```

而不是继续改 chronological barrier。

---

# 31. 不要求 bitwise identical

即使 backend ordering 修复后，也不要求：

```text
NAV 每一位完全相同
```

因为：

```text
frontend/backend thread scheduling
output timestamp capture
floating-point execution interleaving
```

仍可能造成很小差异。

我们的目标是：

```text
算法 measurement chronology 等价
+
轨迹结果在数值意义上一致
```

而不是：

```text
binary deterministic replay
```

---

# 32. Full building02 之前的通过条件

120 s 必须同时满足：

```text
[稳定性]
run_valid = YES
无 FATAL / CHECK / historical-IMU error
queue_peak_addin 仍受控
backend lag 仍受控

[顺序]
overtaking epochs = 0

[tracker]
pending_lidar_at_eof = 0

[结果]
181372~181374 原短时 NAV 差异显著改善
整体位置/速度/姿态差异保持厘米级/小角度级
```

满足后再跑完整：

```text
building02
```

---

# 33. Full building02 验证

完整 building02 要额外确认：

1. 正常跨过此前约 `+1517 s` 的 LiDAR `timefinal` / IMU coverage 故障位置；
2. 无历史 IMU 不足；
3. `overtaking epochs = 0`；
4. `pending_lidar_at_eof = 0`；
5. `pipeline_drained = true`；
6. NAV / timing 正常 flush；
7. `total_running_time_s` 正常生成。

完整 building02 成功后，再进入：

```text
street00
street01
street02
```

正式 benchmark。

---

# 34. 最终数据流

修复后 direct mode 的核心逻辑应为：

```text
raw LiDAR L(tL)
    ↓
register pending(tL)
    ↓
addin
    ↓
LiDAR frontend
    ↓
processed LiDAR L(tL)
    ↓
addin
    ↓
timestamp-ordered alignment insertion
    ↓
remove pending(tL)
    ↓
alignment buffer

raw rover GNSS G(tG)
    ↓
addin
    ↓
timestamp-ordered alignment insertion
    ↓
release attempt
    ↓
存在 pending tL < tG ?
    ├─ YES → hold rover GNSS
    │          reader/frontends继续运行
    │
    └─ NO
         ↓
    alignment front 是否有更老 LiDAR
    等待 IMU coverage？
         ├─ YES → hold chronological stream
         └─ NO  → release to backend
```

Reference GNSS：

```text
保持原路径
→ 不受该 barrier 限制
```

---

# 35. 与 backpressure 的关系

最终 direct-bag 有三层不同职责：

### Layer 1：Addin backpressure

```text
100 / 40
```

解决：

```text
raw reader 远快于 measurement thread
```

---

### Layer 2：Backend lag backpressure

```text
1.5 / 0.8 s
```

解决：

```text
measurement/frontend producer 远快于 backend
```

---

### Layer 3：Chronological LiDAR–rover-GNSS admission barrier

无时间阈值。

解决：

```text
LiDAR frontend 路径更长
导致 newer rover GNSS 越过 older LiDAR
```

三层不要合并。

---

# 36. 最终修改边界

本阶段实际代码修改应收敛为：

```text
MultiSensorEstimating:
  + direct chronological mode flag
  + pending LiDAR admission multiset
  + raw LiDAR register
  + processed LiDAR aligned completion
  + dropped LiDAR cleanup
  + direct-only timestamp ordered alignment insertion
  + rover GNSS pending-LiDAR barrier
  + direct-only strict IMU-coverage chronological stop
  + EOF/pipeline idle tracker check
  + diagnostics

gici_robnav_bag_main:
  + enableDirectChronologicalAdmissionBarrier(true)
  + print final chronological stats
```

明确不改：

```text
existing addin 100/40
existing backend lag 1.5/0.8
existing two-stage EOF
existing LiDAR timefinal IMU gate
input_align_latency value
RRR estimator math
Ceres
sensor converters
ROS baseline
```

---

# 37. 最终判据

本阶段真正的完成判据不是：

```text
程序不崩
```

而是：

\[
\boxed{
\forall\ \text{rover GNSS update }G(t_G),
\quad
\nexists\ L(t_L<t_G)
\text{ 在 }G\text{ update 后才进入 backend}
}
\]

也就是：

```text
overtaking epochs = 0
```

在此基础上再要求：

```text
ROS 1× vs direct NAV
不再出现由 measurement ordering 引起的短时米级差异。
```

达到这两点以后，direct-bag 才可以认为同时满足：

```text
稳定性
+
吞吐率
+
measurement chronology equivalence
```

然后才进入正式 full-sequence wall-time benchmark。
