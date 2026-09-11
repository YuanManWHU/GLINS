# GLINS Direct-Bag Backpressure：诊断确认与最小实现方案

## 0. 当前结论

`building02` direct-bag 失败的现象已经高度指向：

```text
direct rosbag producer
    >> LiDAR frontend / backend consumer
        ↓
异步队列持续积压
        ↓
较新的 IMU / GNSS 继续推进
        ↓
旧 LiDAR 晚到 backend
        ↓
其所需历史 IMU 已被 ImuEstimatorBase 清理
        ↓
ImuError: First IMU measurement ... not old enough
        ↓
FATAL
```

但在正式修改 flow control 前，先做一次 **最小诊断确认**，确认故障前：

```text
最新输入 / latest IMU
明显领先
oldest pending LiDAR / backend measurement
```

且 `imu_measurements_.front()` 已经越过旧 measurement 所需积分起点。

本阶段不修改 RRR、LiDAR 时间语义、IMU 模型、Ceres 参数、input_align_latency。

---

# 1. Stage A：只加诊断，不加 backpressure

目标：用 `building02 --duration 80` 稳定复现当前故障，同时输出流水线时间关系。

## A1. 增加 DirectPipelineSnapshot

在：

```text
include/gici/fusion/multisensor_estimating.h
```

增加只读诊断结构：

```cpp
struct DirectPipelineSnapshot {
  size_t addin_size = 0;
  size_t align_size = 0;
  size_t lidar_frontend_size = 0;
  size_t backend_size = 0;

  double latest_imu_timestamp = 0.0;

  double oldest_addin_timestamp = 0.0;
  double oldest_align_timestamp = 0.0;
  double oldest_lidar_frontend_timestamp = 0.0;
  double oldest_backend_timestamp = 0.0;

  double active_lidar_timestamp = 0.0;
  double active_backend_timestamp = 0.0;
};
```

增加：

```cpp
DirectPipelineSnapshot directPipelineSnapshot();
```

注意：这只是观测接口，不改变数据处理。

---

## A2. 记录 active LiDAR / backend timestamp

当前 queue 被 `pop_front()` 以后，队列本身已经看不到正在计算的 measurement。

因此增加：

```cpp
std::atomic<double> active_lidar_timestamp_{0.0};
std::atomic<double> active_backend_timestamp_{0.0};
```

### LiDAR frontend

在真正开始处理某个 scan 前：

```cpp
active_lidar_timestamp_.store(front_measurement.timestamp);
lidar_frontend_busy_.store(true);
```

整帧处理结束：

```cpp
lidar_frontend_busy_.store(false);
active_lidar_timestamp_.store(0.0);
```

要求 active timestamp 覆盖：

```text
copy scan
processLidar()
processed scan 重新入队
LiDAR frontend output callbacks
```

整个 active frontend 区间。

### backend

`processEstimator()` pop 出：

```cpp
EstimatorDataCluster measurement = measurements_.front();
```

之后立即：

```cpp
active_backend_timestamp_.store(measurement.timestamp);
```

在该 measurement 完整：

```text
coordinate setup
addMeasurement
estimate
post / marginalization
intermediate output
```

结束之后清零：

```cpp
active_backend_timestamp_.store(0.0);
```

如果函数存在多个提前 `return false`，推荐使用一个小 RAII guard，保证 active timestamp 一定清零，避免诊断状态卡死。

---

# 2. snapshot 的实现

在：

```text
src/fusion/multisensor_estimating.cpp
```

实现：

```cpp
MultiSensorEstimating::DirectPipelineSnapshot
MultiSensorEstimating::directPipelineSnapshot()
{
  DirectPipelineSnapshot s;

  {
    std::lock_guard<std::mutex> lock(mutex_addin_);
    s.addin_size = measurement_addin_buffer_.size();
    if (!measurement_addin_buffer_.empty()) {
      s.oldest_addin_timestamp =
          measurement_addin_buffer_.front().timestamp;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_align_);
    s.align_size = measurement_align_buffer_.size();
    if (!measurement_align_buffer_.empty()) {
      s.oldest_align_timestamp =
          measurement_align_buffer_.front().timestamp;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_lidar_input_);
    s.lidar_frontend_size =
        lidar_frontend_measurements_.size();
    if (!lidar_frontend_measurements_.empty()) {
      s.oldest_lidar_frontend_timestamp =
          lidar_frontend_measurements_.front().timestamp;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_input_);
    s.backend_size = measurements_.size();
    s.latest_imu_timestamp = latest_imu_timestamp_;
    if (!measurements_.empty()) {
      s.oldest_backend_timestamp =
          measurements_.front().timestamp;
    }
  }

  s.active_lidar_timestamp =
      active_lidar_timestamp_.load();

  s.active_backend_timestamp =
      active_backend_timestamp_.load();

  return s;
}
```

这里不要为了诊断扫描整个 queue，只取：

```text
size
front timestamp
active timestamp
latest IMU
```

足够。

---

# 3. direct runner 中定期输出 snapshot

在：

```text
ros_wrapper/src/gici/src/gici_robnav_bag_main.cpp
```

formal loop 中增加低频诊断。

不要每条 IMU 都打印，否则日志 I/O 会严重改变运行速度。

建议每：

```text
0.5 s data-time
```

打印一次。

例如：

```cpp
double next_diag_time =
    arguments.start_time_s + 0.5;
```

每处理完当前 instance：

```cpp
const double input_record_time =
    instance.getTime().toSec();

if (input_record_time >= next_diag_time) {
  const auto s = estimator->directPipelineSnapshot();

  LOG(INFO)
      << std::fixed << std::setprecision(6)
      << "[DIRECT_PIPELINE] input=" << input_record_time
      << " latest_imu=" << s.latest_imu_timestamp
      << " addin=" << s.addin_size
      << " addin_front=" << s.oldest_addin_timestamp
      << " lidar_q=" << s.lidar_frontend_size
      << " lidar_front=" << s.oldest_lidar_frontend_timestamp
      << " lidar_active=" << s.active_lidar_timestamp
      << " align=" << s.align_size
      << " align_front=" << s.oldest_align_timestamp
      << " backend_q=" << s.backend_size
      << " backend_front=" << s.oldest_backend_timestamp
      << " backend_active=" << s.active_backend_timestamp;

  next_diag_time += 0.5;
}
```

这里的 `input=` 只用于表示 direct reader 当前已经读到哪个 bag record time。

不用于修改算法。

---

# 4. A 阶段运行方法

直接运行：

```bash
python3 ros_wrapper/src/gici/scripts/robnav/run_robnav_rrr_direct.py \
  building02 --duration 80
```

当前故障对应的数据时间约在：

```text
start + 65.6 s
```

因此 80 s 足够。

---

# 5. A 阶段重点看什么

重点不是 GNSS cycle slip。

重点 grep：

```bash
grep -E \
"DIRECT_PIPELINE|Large backend pending|Large LiDAR frontend backlog|Large integration duration|First IMU measurement" \
direct_runner.log
```

期望看到类似演化：

```text
input                   持续快速前进
latest_imu              跟随 input 前进

oldest_lidar_frontend   越来越落后
或
oldest_backend          越来越落后

backend_q
7 → 20 → 50 → 100+

large integration
2 s → 5 s → 8 s → 12 s
```

如果在 FATAL 前确认：

```text
latest_imu - oldest_pending
```

已经达到数秒，而原正常 ROS 模式没有这种持续增长，就可以正式确认：

```text
direct producer 无界领先
```

是根因。

---

# 6. Stage B：最小 backpressure

Stage A 确认后，再加入 flow control。

原则：

```text
不恢复 1× 实时播放
不 sleep 到 sensor timestamp
不限制算法本身速度
只在 producer 明显跑到 consumer 前面时阻塞 direct reader
```

也就是说：

> GLINS 能多快处理，就多快读取；只有内部 pipeline 已经明显积压时，reader 才等待。

这个等待属于系统完成该序列所需的真实 wall time，因此应计入 Total running time。

---

# 7. 为什么第一版采用 queue backpressure，而不是改 IMU buffer

不要通过：

```text
do_not_remove_imu_measurements_=true
永久保留全部 IMU
加大固定 IMU history
取消 ImuError CHECK
```

解决。

这些只能让 crash 晚一点发生，但：

```text
旧 LiDAR
和
新 GNSS / IMU
```

仍然可能在 backend 中严重错位。

真正需要限制的是：

```text
producer-consumer skew
```

---

# 8. 第一版 backpressure 使用三个 queue

使用：

```text
measurement_addin_buffer_
lidar_frontend_measurements_
measurements_  (backend)
```

不直接以 CPU 使用率作为判据。

建议第一版高水位：

```text
addin_high          = 100
lidar_frontend_high = 5
backend_high        = 6
```

恢复低水位：

```text
addin_low           = 40
lidar_frontend_low  = 2
backend_low         = 3
```

理由：

### LiDAR

AT128：

```text
10 Hz
```

所以：

```text
5 scans ≈ 0.5 s
```

明显低于：

```text
ImuEstimatorBase
oldestState - 1.0 s
```

这一 IMU history guard 的 1 s 量级。

### Backend

RRR backend 主要接收：

```text
processed LiDAR ≈ 10 Hz
rover/reference GNSS ≈ 1 Hz + 1 Hz
```

所以：

```text
6 measurements
```

约为半秒量级 pending。

### Addin

IMU：

```text
200 Hz
```

所以：

```text
100 measurements ≈ 0.5 s IMU
```

这里只是防止最上游无限堆积。

这些阈值不是算法参数，只是 direct feeder 的 bounded-buffer policy。

---

# 9. 使用 hysteresis，避免频繁启停

不要：

```cpp
while (queue > threshold)
    sleep();
```

然后刚降一个元素立即恢复。

使用：

```text
HIGH → 进入 throttling
LOW  → 退出 throttling
```

伪代码：

```cpp
bool throttling = false;

while (...) {
  auto s = estimator->directPipelineSnapshot();

  if (!throttling) {
    if (s.addin_size >= 100 ||
        s.lidar_frontend_size >= 5 ||
        s.backend_size >= 6) {
      throttling = true;
    }
  }
  else {
    if (s.addin_size <= 40 &&
        s.lidar_frontend_size <= 2 &&
        s.backend_size <= 3) {
      throttling = false;
    }
  }

  if (!throttling) break;

  std::this_thread::sleep_for(
      std::chrono::milliseconds(1));
}
```

---

# 10. 一个重要问题：不能因为 backpressure 阻断 LiDAR 所需 future IMU

如果刚 feed 了一帧 LiDAR：

```text
timebase = t
timefinal ≈ t + 0.052 s
```

而 reader 在 `t` 处立即暂停，理论上可能阻断接下来覆盖到：

```text
t + 0.052
```

的 IMU。

因此第一版不要在：

```text
每条 message feed 后
```

无条件立即 throttle。

推荐规则：

> **仅在刚处理完 IMU message 后检查并进入 backpressure。**

因为这时 direct input 已经向前推进了一次 IMU 时间，且连续 IMU 自然为前面 LiDAR 提供 coverage。

实现：

```cpp
bool just_fed_imu = false;

if (instance.getTopic() == arguments.imu_topic) {
    ...
    imu_stream->feedImu(message);
    just_fed_imu = true;
}

...

if (just_fed_imu) {
    applyBackpressure();
}
```

但仅这一点还不能数学上保证当前 latest IMU 已经覆盖所有 pending LiDAR。

所以推荐再增加一个保护：

```text
如果 pipeline 中存在 pending LiDAR，
且其最老时间仍非常接近 latest IMU，
允许 reader 继续推进 IMU，不进入 throttle。
```

---

# 11. 更稳妥的 LiDAR coverage guard

给 snapshot 增加：

```cpp
double oldest_pending_lidar_timefinal = 0.0;
```

计算范围至少包括：

```text
lidar_frontend_measurements_
measurement_align_buffer_ 中的 LiDAR
measurements_ 中的 LiDAR
active LiDAR
active backend LiDAR（若能记录）
```

如果：

```text
oldest_pending_lidar_timefinal > latest_imu_timestamp
```

说明 pipeline 仍需要 future IMU。

这时：

```text
不要进入 throttle
```

继续从 merged bag 读取。

当：

```text
latest_imu_timestamp >= oldest_pending_lidar_timefinal
```

以后，才允许 high-water backpressure 阻塞 reader。

这样不会因为 flow control 自己制造：

```text
Last IMU measurement ... not new enough
```

死锁。

---

# 12. 第一版可以进一步简化

为了先解决当前 crash，也可以：

```text
只用 lidar_frontend_high + backend_high
```

暂时不根据 addin queue throttle。

即：

```text
lidar_frontend >= 5
OR
backend >= 6
```

且 LiDAR IMU coverage 已满足时 pause。

这样逻辑更直接：

```text
真正昂贵消费者
= LiDAR frontend + backend
```

measurement addin thread 本身很轻，不是当前瓶颈。

推荐第一轮先采用这个简化版本。

---

# 13. direct main 的推荐流程

formal loop 变成：

```text
取下一条 rosbag message
        ↓
instantiate
        ↓
feed existing RosStream callback
        ↓
如果当前是 IMU：
    snapshot
        ↓
    pending LiDAR 仍缺 future IMU？
        ├─ 是 → 继续读
        └─ 否
            ↓
        LiDAR frontend / backend 超 HIGH？
            ├─ 否 → 继续读
            └─ 是
                ↓
             等待 consumer
                ↓
             两队列都降到 LOW
                ↓
             继续读
```

这仍然是：

```text
as fast as possible
```

而不是：

```text
real-time playback
```

---

# 14. 为什么 backpressure sleep 要计入 Total

direct reader 因为：

```text
backend 算不过来
```

而等待，本质上就是：

```text
处理完整数据所需 wall time
```

所以：

```text
backpressure wait
```

必须处于：

```text
total_start
~
total_end
```

之间。

不能从 Total 中扣掉。

---

# 15. 但日志 I/O 不应污染正式 benchmark

Stage A 可以每 0.5 s data-time 打 snapshot。

Stage B correctness 验证完成后，正式 benchmark 应：

```text
关闭高频 DIRECT_PIPELINE INFO
```

只保留最终汇总：

```text
backpressure_enter_count
backpressure_wait_time_s
max lidar queue
max backend queue
```

否则大量 glog 会额外增加 wall time。

---

# 16. 建议增加 backpressure 统计

direct runner 最终输出：

```text
backpressure_enter_count
backpressure_wait_time_s
max_observed_lidar_frontend_queue
max_observed_backend_queue
```

例如：

```cpp
size_t backpressure_enter_count = 0;
double backpressure_wait_time_s = 0.0;
```

进入 HIGH 状态时：

```cpp
++backpressure_enter_count;
auto wait_start = steady_clock::now();
```

退出 LOW 状态：

```cpp
backpressure_wait_time_s +=
    duration<double>(
        steady_clock::now() - wait_start).count();
```

这些只是诊断。

Total running time 已经自然包含该等待。

---

# 17. Stage B 验证顺序

不要先跑完整 building02。

### B1

```bash
building02 --duration 80
```

要求：

```text
不再出现：
First IMU measurement ... not old enough
FATAL

backend queue 不再单调增长到 100+
LiDAR frontend queue 被限制在高水位附近
Large integration duration 不再持续增长到 10s+
```

### B2

```bash
building02 --duration 100
```

与现有 1× ROS 100 s baseline：

```text
NAV 对齐比较
LiDAR frontend count
backend add count
GNSS / IMU / LiDAR input count
```

### B3

完整：

```text
street00
```

因为已有稳定 ROS baseline。

### B4

再完整：

```text
building02
```

尤其确认此前 1517 s 的 LiDAR/IMU timefinal 故障点也正常通过。

---

# 18. 判断 backpressure 是否过强

如果 direct run 中：

```text
lidar_frontend queue 几乎总是 0
backend queue 几乎总是 0
```

同时 reader 大量等待，则阈值可能太保守，损失了 GLINS 原本允许的 frontend/backend 并行。

合理状态应该是：

```text
存在一定小 backlog
但不会持续扩大
```

例如：

```text
lidar queue 0~5
backend queue 0~6
```

并周期性回落。

---

# 19. 判断 backpressure 是否过弱

如果仍出现：

```text
Large integration duration
持续增长到 > 1~2 s

backend queue 长时间贴着 HIGH
并偶尔超过很多

oldest pending timestamp
持续落后 latest IMU > 1 s
```

则需要降低 HIGH。

优先顺序：

```text
先降低 backend_high
再降低 lidar_frontend_high
```

不要先改 estimator 或 IMU retention。

---

# 20. 暂时不建议 condition_variable

最严格的实现可以使用：

```text
condition_variable
```

由 frontend/backend pop queue 时唤醒 direct reader。

但这需要修改多个 worker thread 的通知点。

当前目标只是：

```text
RobNav + RRR direct-bag benchmark
```

第一版使用：

```text
1 ms sleep + hysteresis
```

修改更小。

若验证发现 1 ms polling 对 wall benchmark 有可见影响，再升级 condition variable。

---

# 21. 当前最小修改范围

Stage A：

```text
multisensor_estimating.h
multisensor_estimating.cpp
gici_robnav_bag_main.cpp
```

Stage B 仍然只需要这三个文件。

不需要修改：

```text
imu_error.cpp
imu_estimator_base.cpp 的 IMU 删除策略
rtk_imu_lidar_rrr_estimator.cpp
ros_stream.cpp
DataIntegration
YAML 算法参数
run_robnav_rrr.py
```

Python direct runner只需后续读取新增 summary 字段时再扩展，不是修 crash 的必要条件。

---

# 22. 当前推荐执行顺序

```text
A1  DirectPipelineSnapshot
A2  active LiDAR/backend timestamp
A3  building02 --duration 80
A4  根据 snapshot 确认 producer-consumer skew

确认后：

B1  LiDAR/backend HIGH/LOW backpressure
B2  LiDAR future-IMU coverage guard
B3  80 s crash regression
B4  100 s ROS-vs-direct consistency
B5  full street00
B6  full building02
```

在 A4 之前，不改 IMU history 和 estimator 数学逻辑。

---

# 23. 最终目标

正确的 direct-bag 模式应满足：

```text
输入不按真实时间 sleep
        ↓
reader 始终尽可能快
        ↓
GLINS consumer 能跟上时：
    不等待

consumer 跟不上时：
    bounded backpressure
        ↓
保持小规模 pipeline 并行
        ↓
防止旧 LiDAR 被新 IMU/GNSS 拉开数秒
        ↓
历史 IMU 不会在旧 measurement 使用前被回收
```

这样测得的：

```text
Total running time
```

仍然是真实的：

```text
GLINS 以自身最大可持续吞吐率处理整段 RobNav 数据的 wall-clock time
```

而不是 1× rosbag playback 时间。
