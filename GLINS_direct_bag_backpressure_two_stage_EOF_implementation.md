# GLINS Direct-Bag Backpressure + 两阶段 EOF：具体实现方案

## 1. 修改目标

根据 `building02 --duration 80` 的诊断结果：

```text
addin peak              = 3873
lidar_frontend peak     = 1
backend snapshot peak   = 19
input - latest_imu max  = 17.625 s
backend warning max     = 191
```

因此当前首先需要修复两个问题：

1. direct rosbag reader 无 backpressure，`measurement_addin_buffer_` 无界增长；
2. formal rosbag EOF 后立即 `notifyInputFinished()`，但此时 addin 中仍有大量尚未分发的数据，导致 alignment latency hold 被过早取消。

本轮只修改 direct-bag 调度，不修改：

```text
IMU history 删除策略
ImuError
LiDAR timefinal gate
input_align_latency
RRR / GNSS / LiDAR 数学模型
Ceres 参数
ROS 1× runner
```

第一轮只做 **addin backpressure + 两阶段 EOF**。暂不把 LiDAR/frontend/backend queue 直接写入 throttle 条件；先验证 primary bottleneck 修复后的实际行为。

---

# 2. 需要修改的文件

```text
include/gici/fusion/multisensor_estimating.h
src/fusion/multisensor_estimating.cpp
ros_wrapper/src/gici/src/gici_robnav_bag_main.cpp
```

不需要修改：

```text
imu_estimator_base.cpp
imu_error.cpp
rtk_imu_lidar_rrr_estimator.cpp
ros_stream.cpp
run_robnav_rrr_direct.py   # 第一轮可不改
YAML
```

---

# 3. MultiSensorEstimating：新增 direct-input flow-control 状态

## 3.1 header 中新增结构

在 `MultiSensorEstimating` public 区增加：

```cpp
struct DirectInputFlowState {
  size_t addin_size = 0;

  // Timestamp of the newest raw IMU already accepted from the direct reader.
  double latest_input_imu_timestamp = 0.0;

  // timefinal of the newest raw LiDAR already accepted from the direct reader.
  double latest_input_lidar_timefinal = 0.0;
};

// Lightweight state used only by the direct-bag feeder.
DirectInputFlowState directInputFlowState();

// True only when no raw/processed frontend measurement can still be inserted
// into measurement_addin_buffer_ / alignment from the ingress side.
bool readyForFinalAlignmentFlush();
```

保留现有：

```cpp
void notifyInputFinished();
bool pipelineIdle();
```

但修改 direct runner 的调用时机：不能 formal EOF 后立刻调用 `notifyInputFinished()`。

---

## 3.2 header 中新增两个 atomic timestamp

在 direct-bag pipeline state 附近增加：

```cpp
std::atomic<double> latest_input_imu_timestamp_{0.0};
std::atomic<double> latest_input_lidar_timefinal_{0.0};
```

这里的语义是：

```text
latest_input_imu_timestamp_
    = direct reader 已经成功送入 estimatorDataCallback 的最新 raw IMU timestamp

latest_input_lidar_timefinal_
    = direct reader 已经成功送入 estimatorDataCallback 的最新 raw LiDAR timefinal
```

注意它们不同于现有：

```cpp
latest_imu_timestamp_
```

后者表示 measurement thread 已经真正处理到的 IMU 时间。

这两个新值只用于决定“reader 现在是否安全暂停”。

---

# 4. estimatorDataCallback：维护 input watermark

当前：

```cpp
void MultiSensorEstimating::estimatorDataCallback(
    EstimatorDataCluster& data)
{
  if (estimatorDataIllegal(data)) {
    ...
    return;
  }

  mutex_addin_.lock();
  measurement_addin_buffer_.push_back(data);
  peak_addin_size_ = ...;
  mutex_addin_.unlock();
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

  // Raw direct-input watermarks.
  // IMU never re-enters this callback from a frontend.
  if (data.imu) {
    latest_input_imu_timestamp_.store(
        data.timestamp, std::memory_order_relaxed);
  }

  // Only raw LiDAR has need_frontend=true.
  // Processed LiDAR re-enters estimatorDataCallback with need_frontend=false,
  // so it must not advance the raw-input watermark again.
  if (data.lidar && data.lidar->need_frontend) {
    latest_input_lidar_timefinal_.store(
        data.lidar->timefinal, std::memory_order_relaxed);
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

为什么使用 LiDAR `timefinal`：

```text
reader 暂停时，必须确保已送入的最新 raw LiDAR 所需 future IMU
也已经送入 addin。
```

否则若刚读到 LiDAR 就因为 addin 达到 HIGH 而暂停，可能人为制造：

```text
LiDAR timefinal > newest enqueued IMU
```

---

# 5. directInputFlowState()

在 `multisensor_estimating.cpp` 增加：

```cpp
MultiSensorEstimating::DirectInputFlowState
MultiSensorEstimating::directInputFlowState()
{
  DirectInputFlowState state;

  {
    std::lock_guard<std::mutex> lock(mutex_addin_);
    state.addin_size = measurement_addin_buffer_.size();
  }

  state.latest_input_imu_timestamp =
      latest_input_imu_timestamp_.load(
          std::memory_order_relaxed);

  state.latest_input_lidar_timefinal =
      latest_input_lidar_timefinal_.load(
          std::memory_order_relaxed);

  return state;
}
```

这个接口只锁 `mutex_addin_`，不扫描其他队列。

原因：

```text
正式 full-sequence benchmark 会调用很多次 backpressure check，
不应每条 message 都锁 addin / align / lidar / backend 四个 mutex。
```

完整 `DirectPipelineSnapshot` 继续只用于诊断日志。

---

# 6. readyForFinalAlignmentFlush()

formal rosbag EOF 后，不能立刻取消 alignment latency。

要先确认：

```text
measurement_addin_buffer_ 已空
measurement thread 当前不在处理已 pop 的 measurement
LiDAR frontend queue 已空
LiDAR frontend 当前不在处理 scan
```

因为 raw LiDAR frontend 完成后还会：

```text
processed LiDAR
→ estimatorDataCallback()
→ measurement_addin_buffer_
```

所以必须等 LiDAR frontend 也完全 quiescent。

实现：

```cpp
bool MultiSensorEstimating::readyForFinalAlignmentFlush()
{
  // First busy check.
  if (measurement_busy_.load() ||
      lidar_frontend_busy_.load()) {
    return false;
  }

  bool addin_empty = false;
  bool lidar_empty = false;

  {
    std::lock_guard<std::mutex> lock(mutex_addin_);
    addin_empty = measurement_addin_buffer_.empty();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_lidar_input_);
    lidar_empty = lidar_frontend_measurements_.empty();
  }

  if (!addin_empty || !lidar_empty) {
    return false;
  }

  // Recheck busy flags to close the race:
  // a worker may have popped an item after the first check.
  if (measurement_busy_.load() ||
      lidar_frontend_busy_.load()) {
    return false;
  }

  return true;
}
```

对于当前 `RobNav + RRR`：

```text
camera frontend 不存在
```

因此不需要为 image frontend 增加新逻辑。

---

# 7. 为什么这里不要求 align / backend empty

`readyForFinalAlignmentFlush()` 的问题不是：

```text
整个 pipeline 是否结束？
```

而是：

> **以后还会不会有新的 measurement 从 ingress/frontend 进入 alignment？**

所以：

```text
measurement_align_buffer_
measurements_
backend_busy_
output_timestamps_
```

此时允许非空。

它们正是下一阶段 EOF flush + drain 要处理的对象。

真正全部结束仍由：

```cpp
pipelineIdle()
```

判断。

---

# 8. Direct runner：backpressure 常量

在 `gici_robnav_bag_main.cpp` 匿名 namespace 中增加：

```cpp
constexpr size_t kAddinHighWatermark = 100;
constexpr size_t kAddinLowWatermark = 40;
constexpr auto kBackpressureSleep =
    std::chrono::milliseconds(1);
```

当前 80 s 实测显示：

```text
addin≈100
对应 input-latest_imu ≈ 0.4~0.5 s
```

所以这两个值有直接数据依据，而不是任意选择。

---

# 9. Direct runner：增加 backpressure 统计

formal loop 前：

```cpp
bool backpressure_active = false;
size_t backpressure_enter_count = 0;
double backpressure_wait_time_s = 0.0;
size_t max_addin_observed = 0;
```

可选再记录：

```cpp
double max_input_coverage_gap_s = 0.0;
```

这些都是诊断量。

`backpressure_wait_time_s` 不从 Total 中扣除；它本来就应该属于真实运行时间。

---

# 10. Backpressure 的核心函数

建议在 `gici_robnav_bag_main.cpp` 匿名 namespace 中增加 helper：

```cpp
struct BackpressureStats {
  size_t enter_count = 0;
  double wait_time_s = 0.0;
  size_t max_addin = 0;
};

void applyDirectBackpressure(
    const std::shared_ptr<gici::MultiSensorEstimating>& estimator,
    BackpressureStats* stats)
{
  auto state = estimator->directInputFlowState();

  stats->max_addin =
      std::max(stats->max_addin, state.addin_size);

  // Not overloaded yet.
  if (state.addin_size < kAddinHighWatermark) {
    return;
  }

  // Do not pause the reader until the newest raw LiDAR already fed into
  // the pipeline has its scan-end IMU coverage also fed into the pipeline.
  if (state.latest_input_lidar_timefinal > 0.0 &&
      state.latest_input_imu_timestamp <
          state.latest_input_lidar_timefinal) {
    return;
  }

  ++stats->enter_count;
  const auto wait_start =
      std::chrono::steady_clock::now();

  while (gici::SpinControl::ok()) {
    state = estimator->directInputFlowState();

    stats->max_addin =
        std::max(stats->max_addin, state.addin_size);

    if (state.addin_size <= kAddinLowWatermark) {
      break;
    }

    std::this_thread::sleep_for(
        kBackpressureSleep);
  }

  stats->wait_time_s +=
      std::chrono::duration<double>(
          std::chrono::steady_clock::now() -
          wait_start).count();
}
```

这里使用 hysteresis：

```text
100 → 开始等
40  → 恢复 reader
```

避免：

```text
99 / 100 / 99 / 100
```

频繁启停。

---

# 11. formal loop 中的调用位置

现有：

```cpp
for (const auto& instance : formal_view) {
    ...
    feedXXX(message);
}
```

改成：

```cpp
BackpressureStats backpressure_stats;

for (const auto& instance : formal_view) {
  ...
  if (instance.getTopic() == arguments.ephemerides_topic) {
    ...
  }
  else if (instance.getTopic() == arguments.imu_topic) {
    ...
    imu_stream->feedImu(message);
    ++imu_count;
  }
  else if (instance.getTopic() == arguments.lidar_topic) {
    ...
    lidar_stream->feedHesaiAt128(message);
    ++lidar_count;
  }
  else if (...) {
    ...
  }

  // Check after the current message has already entered the original GLINS
  // conversion/pipeline. This preserves merged rosbag order.
  applyDirectBackpressure(
      estimator, &backpressure_stats);
}
```

这里不是：

```text
按 timestamp sleep
```

而是：

```text
仅当 GLINS 内部已经积压时等待 consumer
```

所以仍然属于：

```text
direct bag / as-fast-as-sustainable
```

---

# 12. 为什么不是“只在 IMU 后检查”

如果使用：

```text
latest_input_imu_timestamp
latest_input_lidar_timefinal
```

作为 coverage guard，就不需要把 backpressure check 限制在 IMU message 后。

当刚 feed LiDAR 时：

```text
latest_input_imu < latest_lidar_timefinal
```

helper 会自动返回，不暂停。

随后继续读 future IMU。

一旦：

```text
latest_input_imu >= latest_lidar_timefinal
```

且 addin 仍超过 HIGH，就安全进入暂停。

这比“只在 IMU 后检查”更明确。

---

# 13. formal EOF：改为两阶段

当前错误顺序：

```cpp
for (...) {
    ...
}

estimator->notifyInputFinished();
wait pipelineIdle();
```

改为：

```text
formal rosbag EOF
    ↓
Stage EOF-1:
不取消 alignment latency
先等 ingress/frontend quiescent
    ↓
Stage EOF-2:
notifyInputFinished()
允许最终 alignment flush
    ↓
Stage EOF-3:
等待完整 pipelineIdle()
```

---

# 14. Stage EOF-1：等待 ingress/frontend quiescent

增加：

```cpp
const auto ingress_drain_start =
    std::chrono::steady_clock::now();

const auto ingress_deadline =
    ingress_drain_start +
    std::chrono::seconds(60);

bool ingress_quiescent = false;

while (std::chrono::steady_clock::now() <
       ingress_deadline) {

  if (estimator->readyForFinalAlignmentFlush()) {
    // Two consecutive observations for race protection.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(2));

    if (estimator->readyForFinalAlignmentFlush()) {
      ingress_quiescent = true;
      break;
    }
  }

  std::this_thread::sleep_for(
      std::chrono::milliseconds(1));
}

if (!ingress_quiescent) {
  throw std::runtime_error(
      "Timed out while draining direct-input/addin/LiDAR frontend "
      "before final alignment flush.");
}
```

注意：

```text
这期间 input_finished_ 仍为 false
```

因此 `releaseAlignedMeasurements()` 仍按正常在线逻辑保留最后 `input_align_latency`。

---

# 15. Stage EOF-2：此时才 notifyInputFinished()

只有 EOF-1 成功后：

```cpp
estimator->notifyInputFinished();
```

此时可以明确认为：

```text
外部 raw input 已经结束
addin 已经空
measurement thread 没有拿着一条尚未分流的数据
LiDAR frontend 已经空
LiDAR frontend 没有 active scan
processed LiDAR 不会再回到 addin
```

所以以后不会再有新的 measurement 进入 alignment。

此时取消 latency hold 才具有正确语义。

---

# 16. Stage EOF-3：等待完整 pipeline idle

然后继续使用现有完整 drain：

```cpp
const auto drain_start =
    std::chrono::steady_clock::now();

const auto drain_deadline =
    drain_start + std::chrono::seconds(60);

bool idle_once = false;
bool drained = false;

while (std::chrono::steady_clock::now() <
       drain_deadline) {
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

  std::this_thread::sleep_for(
      std::chrono::milliseconds(2));
}

if (!drained) {
  throw std::runtime_error(
      "Timed out while draining the estimator pipeline.");
}
```

这里处理：

```text
alignment residual tail
backend queue
backend active optimization
output timestamps
```

---

# 17. `notifyInputFinished()` 本身不用改语义

第一轮为了最小修改，可以继续：

```cpp
void MultiSensorEstimating::notifyInputFinished()
{
  input_finished_.store(true);
}
```

只改变它的调用时机。

这样不会影响现有 ROS 模式，因为：

```text
ROS mode 根本不调用这个 direct-bag EOF API。
```

---

# 18. `runMeasurementAddin()` 保持不变

当前：

```cpp
void MultiSensorEstimating::runMeasurementAddin()
{
  SpinControl spin(1.0e-4);

  while (!quit_thread_ && SpinControl::ok()) {
    measurement_busy_.store(true);

    putMeasurements();

    if (input_finished_.load()) {
      releaseAlignedMeasurements();
    }

    measurement_busy_.store(false);
    spin.sleep();
  }
}
```

第一轮无需改。

由于 `input_finished_` 只会在 ingress/frontend 真正 quiescent 后置 true，所以这里的 EOF release 语义会自然变正确。

---

# 19. 正式 diagnostic output

本轮建议 direct executable 最后增加：

```cpp
std::cout
    << "backpressure_enter_count="
    << backpressure_stats.enter_count << std::endl
    << "backpressure_wait_time_s="
    << backpressure_stats.wait_time_s << std::endl
    << "backpressure_max_addin="
    << backpressure_stats.max_addin << std::endl
    << "ingress_quiesce_time_s="
    << ingress_quiesce_time_s << std::endl;
```

并继续输出现有：

```text
formal_input_loop_time_s
drain_time_s
queue_peak_addin
queue_peak_lidar_frontend
queue_peak_backend
```

注意：

```text
queue_peak_addin
```

应该从原来的 3873 大幅下降到大约：

```text
100 + 少量 coverage overshoot
```

---

# 20. Total running time 的处理

backpressure 和两阶段 EOF 全部位于：

```text
total_start
~
total_end
```

之间。

因此：

```text
backpressure sleep
ingress quiesce
final alignment flush
backend drain
```

全部自然计入：

```text
total_running_time_s
```

不要扣除：

```text
backpressure_wait_time_s
```

它只是用于解释 Total。

---

# 21. 第一轮不要加 backend throttle

虽然当前日志出现：

```text
backend warning max = 191
```

但诊断同时表明：

```text
formal-loop snapshot backend peak = 19
EOF 前 addin peak = 3873
```

所以 191 很可能主要由：

```text
巨大 addin backlog
+
过早 EOF flush
```

共同造成。

第一轮先只修已被直接证实的两个问题。

如果同时加入 backend high/low：

```text
无法判断问题究竟是哪一个机制修好的
也可能不必要地串行化 GLINS frontend/backend
```

因此：

```text
Stage B1:
addin backpressure + EOF fix

验证后：

Stage B2:
只有 backend 仍持续积压时，再增加 backend backpressure。
```

---

# 22. 第一轮验证：building02 80 s

编译：

```bash
cd ~/glins_ws/ros_wrapper
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

运行：

```bash
python3 src/gici/scripts/robnav/run_robnav_rrr_direct.py \
  building02 --duration 80
```

检查：

```text
1. direct executable exit code = 0
2. run_valid = YES
3. queue_peak_addin ≈ 100~120，而不是 3873
4. input-latest_imu 不再持续增长到 17.6 s
5. lidar_frontend peak 仍约 0~少量
6. backend 不再 EOF 后爆到 191
7. 不出现 First IMU measurement ... not old enough
8. 不出现 CHECK/FATAL
9. pipeline_drained = true
```

---

# 23. 建议保留低频 DIRECT_PIPELINE 日志用于第一轮

第一轮 backpressure 修复验证仍保留：

```text
每 0.5 s data-time
```

的 `DIRECT_PIPELINE`。

理想结果应从原来的：

```text
input-latest_imu:
0.095 → 17.625 s

addin:
21 → 3873
```

变成类似：

```text
input-latest_imu:
在约 0~0.6 s 范围周期变化

addin:
周期性上升到 ~100
→ backpressure
→ 回落到 ~40
→ reader 恢复
```

这才说明 hysteresis 真正生效。

---

# 24. 80 s 通过后：100 s correctness

再跑：

```bash
python3 src/gici/scripts/robnav/run_robnav_rrr_direct.py \
  building02 --duration 100
```

与已有 ROS 1× 100 s baseline 对比：

```text
NAV count
GPS SOW
ENU position difference
LiDAR frontend count
LiDAR backend-add count
```

backpressure 只应该改变：

```text
wall scheduling / queue depth
```

不应该改变：

```text
formal measurement sequence
算法数学结果
```

若 NAV 差异明显，先查顺序/EOF，不进入 full benchmark。

---

# 25. 如果 B1 后 backend 仍然积压怎么办

只有在 B1 结果满足：

```text
addin 已经稳定 <= ~120
EOF 时机正确
```

但仍出现：

```text
backend queue 持续单调增长
backend timestamp lag 持续增加
Large integration duration 持续增加
```

才进入 Stage B2。

B2 优先使用：

```text
backend data-time lag
```

而不是只看 queue count。

原因是本次诊断已经看到：

```text
backend_q 只有 8~10
但 imu-backend_front 最大可达 14.7 s
```

说明 backend queue count 不能可靠代表 timestamp lag。

B2 应另外设计：

```text
latest processed IMU timestamp
-
oldest/active backend measurement timestamp
```

的高低水位。

不要现在直接把 `backend_high=20` 写死。

---

# 26. 最终修改边界

本轮最终代码变化应只有：

```text
MultiSensorEstimating:
  + latest_input_imu_timestamp_
  + latest_input_lidar_timefinal_
  + DirectInputFlowState
  + directInputFlowState()
  + readyForFinalAlignmentFlush()

estimatorDataCallback():
  + 维护 raw input watermark

gici_robnav_bag_main:
  + addin HIGH/LOW hysteresis
  + LiDAR timefinal input-coverage guard
  + formal EOF 两阶段处理
  + backpressure diagnostics
```

明确不改：

```text
releaseAlignedMeasurements 的 LiDAR timefinal gate
input_align_latency
ImuEstimatorBase 的历史 IMU 删除规则
RRR estimator
Ceres solver
sensor converter
正式算法配置
```

---

# 27. 预期修复后的数据流

```text
rosbag::View
    ↓
快速读取
    ↓
measurement_addin_buffer_
    ↓
达到 100？
    │
    ├─ 否 → 继续 direct read
    │
    └─ 是
         ↓
   newest raw LiDAR 已有 future IMU coverage？
         │
         ├─ 否 → 继续读到 IMU coverage 满足
         │
         └─ 是
              ↓
          暂停 reader
              ↓
          measurement thread / frontend / backend
          继续正常处理
              ↓
          addin <= 40
              ↓
          恢复 reader
```

formal EOF：

```text
rosbag EOF
    ↓
仍保持正常 alignment latency
    ↓
addin empty
measurement not busy
LiDAR queue empty
LiDAR not busy
    ↓
notifyInputFinished()
    ↓
解除最后 alignment latency hold
    ↓
backend/output 完整 drain
    ↓
正常 shutdown / flush
```

这套修改的目标不是人为限速，而是将原本无界的异步 producer-consumer 系统改成 **bounded asynchronous pipeline**，使 direct-bag 仍以 GLINS 能持续处理的最大速度运行。
