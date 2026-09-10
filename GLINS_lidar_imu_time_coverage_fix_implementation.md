# GLINS LiDAR–IMU 时序边界修复实现方案

## 1. 问题结论

在 `building02` 完整运行中，GLINS 在约 `1745894795.2 s` 附近出现 IMU coverage 不足，随后 Ceres residual/Jacobian 为 NaN，最终 `marginalization_error.cpp` 中 `H_.allFinite()` 失败并触发 FATAL。

已确认故障帧：

```text
LiDAR timebase  = 1745894795.212740898
LiDAR timefinal = 1745894795.264870882
span            = 52.130 ms
latest IMU      = 1745894795.220828295
```

因此：

```text
timebase < latest_imu_timestamp < timefinal
```

原始 IMU 数据本身连续，后续仍有：

```text
1745894795.225828171
1745894795.230827332
...
1745894795.265816450
```

最大间隔约 5 ms，所以不是 IMU 数据断流，而是 LiDAR measurement 在整帧所需 IMU 尚未到齐时提前进入 backend。

根因分为两层：

1. `EstimatorDataCluster` 对 LiDAR 使用 `timebase` 作为 `timestamp`，当前 admission gate 只保证 `latest_imu_timestamp >= timebase`。
2. `RtkImuLidarRrrEstimator::addLidarMeasurementAndState()` 实际需要 IMU 覆盖到 `timefinal`，且未检查 `getPoseEstimateAt()` / `getSpeedAndBiasEstimateAt()` 的返回值，导致失败后仍继续使用无效状态。

本次修复目标是：**只修正 LiDAR backend admission 条件，并增加防御性检查；不改算法、参数、LiDAR timestamp 语义或 runner。**

---

## 2. 修改范围

只修改：

```text
src/fusion/multisensor_estimating.cpp
src/fusion/rtk_imu_lidar_rrr_estimator.cpp
```

不修改：

```text
include/gici/estimate/estimator_types.h
YAML
input_align_latency
run_robnav_rrr.py
experiment_recorder.*
timing instrumentation
Ceres/thread 参数
LiDAR timebase/timefinal 定义
```

---

## 3. 修改一：LiDAR 进入 backend 前必须等 IMU 覆盖到 timefinal

### 3.1 修改位置

文件：

```text
src/fusion/multisensor_estimating.cpp
```

函数：

```cpp
MultiSensorEstimating::handleNonTimePropagationSensors(...)
```

现有逻辑在把 measurement 从 `measurement_align_buffer_` 移入 `measurements_` 前，类似：

```cpp
EstimatorDataCluster& measurement = *it;

if (estimatorTypeContains(SensorType::IMU, type_) &&
    measurement.timestamp > latest_imu_timestamp_) {
  it++;
  continue;
}
```

对于 LiDAR：

```text
measurement.timestamp = lidar->timebase
```

所以当前只保证：

```text
latest IMU >= timebase
```

但 RRR deskew 与状态传播真正需要：

```text
latest IMU >= timefinal
```

### 3.2 修改方案

新增局部变量：

```cpp
double required_imu_timestamp = measurement.timestamp;
if (measurement.lidar) {
  required_imu_timestamp = measurement.lidar->timefinal;
}
```

然后统一 gate：

```cpp
if (estimatorTypeContains(SensorType::IMU, type_) &&
    required_imu_timestamp > latest_imu_timestamp_) {
  it++;
  continue;
}
```

建议完整形式：

```cpp
EstimatorDataCluster& measurement = *it;

// IMU coverage required by the current measurement.
// LiDAR deskew needs IMU data up to scan end, not only scan timebase.
double required_imu_timestamp = measurement.timestamp;
if (measurement.lidar) {
  required_imu_timestamp = measurement.lidar->timefinal;
}

if (estimatorTypeContains(SensorType::IMU, type_) &&
    required_imu_timestamp > latest_imu_timestamp_) {
  it++;
  continue;
}
```

### 3.3 为什么不改 `EstimatorDataCluster.timestamp`

不要将 LiDAR 构造函数中的：

```cpp
timestamp(data.timebase)
```

改成：

```cpp
timestamp(data.timefinal)
```

因为 `timestamp` 还参与 measurement 排序、buffer ordering、input alignment 等通用时序逻辑。直接改成 `timefinal` 会改变整个 LiDAR measurement 的时间语义。

本次只修正：

```text
该 measurement 需要 IMU 覆盖到哪个时刻
```

因此应引入 `required_imu_timestamp`，而不是改变 `measurement.timestamp`。

### 3.4 为什么不增加 `input_align_latency`

当前：

```yaml
enable_input_align: true
input_align_latency: 0.1
```

不建议通过增大到 0.15/0.2 s 来规避问题，因为 latency 不能从逻辑上保证：

```text
latest_imu_timestamp >= lidar->timefinal
```

正确修复应直接表达该必要条件。

---

## 4. 修改二：RRR LiDAR 后端增加状态查询返回值检查

### 4.1 修改位置

文件：

```text
src/fusion/rtk_imu_lidar_rrr_estimator.cpp
```

函数：

```cpp
RtkImuLidarRrrEstimator::addLidarMeasurementAndState(...)
```

当前会直接调用：

```cpp
getPoseEstimateAt(curScan()->timefinal, T_WB_prior);
getPoseEstimateAt(curScan()->timebase, T_WB_base);
getSpeedAndBiasEstimateAt(curScan()->timefinal, Speedbias_prior);
getSpeedAndBiasEstimateAt(curScan()->timebase, Speedbias_base);
```

这些接口均返回 `bool`，但当前没有检查。

### 4.2 推荐修改

```cpp
const bool pose_final_ok =
    getPoseEstimateAt(curScan()->timefinal, T_WB_prior);
const bool pose_base_ok =
    getPoseEstimateAt(curScan()->timebase, T_WB_base);
const bool speed_final_ok =
    getSpeedAndBiasEstimateAt(curScan()->timefinal, Speedbias_prior);
const bool speed_base_ok =
    getSpeedAndBiasEstimateAt(curScan()->timebase, Speedbias_base);

if (!pose_final_ok || !pose_base_ok ||
    !speed_final_ok || !speed_base_ok) {
  LOG(ERROR) << "Unable to obtain LiDAR motion state for scan ["
             << std::fixed << std::setprecision(9)
             << curScan()->timebase << ", "
             << curScan()->timefinal << "].";
  return false;
}
```

也可在调试阶段把 4 个布尔值一并输出，但正式代码无需高频日志。

### 4.3 作用

这不是主修复，而是第二层保护：

```text
admission gate
    → 防止 LiDAR 在 IMU 未覆盖完整 scan 时进入 backend

return-value check
    → 即使未来仍有异常，也不允许无效状态进入 Ceres
```

---

## 5. `lidarInitialization()` 暂不扩大修改

`lidarInitialization()` 同样会查询 `scan->timefinal` 的状态，但它同样经过统一的 LiDAR admission gate。

因此首版不额外修改该函数，保持最小修改范围。

若后续验证仍在初始化阶段出现相同 coverage 问题，再补充同类返回值检查。

---

## 6. 编译

修改后执行：

```bash
cd ~/glins_ws/ros_wrapper

catkin_make --force-cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

要求：

```text
libgici.so rebuilt
gici_main relinked
gici_ros_main relinked
```

不需要修改 CMake。

---

## 7. 验证策略

### 7.1 先验证原故障区间，不立即重跑完整 1871 s

建议覆盖：

```text
1745894785 ~ 1745894805
```

即原故障点 `1745894795.264871` 前后约 10 s。

如果当前 runner 支持局部 start/duration，直接使用；如果不支持，不为了此次验证重构 runner，可继续采用手动 rosbag playback 指定区间。

### 7.2 局部验证通过条件

必须满足：

```text
1. 能处理到 > 1745894795.264871
2. 不再出现：
   Last IMU measurement ... not new enough
3. 不再出现：
   Residuals: -nan
4. 不再出现：
   Residual and Jacobian evaluation failed
5. 不再出现：
   H_.allFinite() failure
6. estimator 不异常退出
```

若局部测试没有完成初始化，则 NAV 是否输出不作为这一轮硬条件；重点是确认原时序故障不再触发。

---

## 8. 完整 building02 回归

局部故障窗口通过后，再执行：

```bash
cd ~/glins_ws/ros_wrapper

python3 src/gici/scripts/robnav/run_robnav_rrr.py building02
```

完整回归要求：

```text
rosbag_exit_code = 0
glins_exit_code  = 0
shutdown_mode    = SIGINT
run_valid        = True
```

并正常生成：

```text
glins.nav
timing_events.csv
timing_summary.csv
run_summary.csv
runtime_config.yaml
estimator_runtime.yaml
```

同时要求：

```text
无 Ceres NaN
无 marginalization NaN
无异常 FATAL
```

---

## 9. 对 runtime 实验的影响

本次修复不改变：

- LiDAR frontend 算法
- 点云降采样
- LiDAR factor 数学模型
- GNSS factor
- optimization 结构
- marginalization 结构
- solver 参数
- Ceres 线程数
- visualization output
- Recorder/timing 逻辑

仅保证：

```text
LiDAR measurement 在所需 IMU 已覆盖完整 scan 后再进入 backend
```

因此属于输入时序正确性修复，不是为了降低运行时间而做的性能优化。修复后的完整 run 可以继续作为正式 GLINS baseline。

此前崩溃的 `building02`：

```text
run_valid = false
```

不能用于任何正式 runtime 统计。

---

## 10. 修复前后时序

### 修复前

```text
LiDAR:
timebase  = 4795.212741
timefinal = 4795.264871

latest IMU = 4795.220828
        ↓
latest IMU >= timebase
        ↓
当前 gate PASS
        ↓
LiDAR 提前进入 backend
        ↓
查询 timefinal 状态失败
        ↓
无效状态继续进入 Ceres
        ↓
NaN
```

### 修复后

```text
required_imu_timestamp = timefinal = 4795.264871

latest IMU = 4795.220828
        ↓
gate FAIL
        ↓
LiDAR 保留在 align buffer
        ↓
后续 IMU 到达
        ↓
latest IMU = 4795.265816
        ↓
gate PASS
        ↓
LiDAR 进入 backend
        ↓
状态传播 / deskew / factor / optimize
```

---

## 11. 最终实现边界

必须做：

```text
A. multisensor_estimating.cpp
   LiDAR required_imu_timestamp 使用 timefinal

B. rtk_imu_lidar_rrr_estimator.cpp
   检查 getPoseEstimateAt / getSpeedAndBiasEstimateAt 返回值
```

明确不做：

```text
不改 EstimatorDataCluster timestamp
不改 input_align_latency
不改 YAML
不改 runner
不改 Recorder
不改 timing instrumentation
不改 Ceres/thread 参数
```

---

## 12. 完成判据

本阶段完成需要：

```text
[1] 两个源文件完成最小修改
[2] RelWithDebInfo 编译通过
[3] 原故障窗口能够越过 1745894795.264871
[4] 不再出现 IMU coverage error
[5] 不再出现 Ceres NaN
[6] 不再出现 H_.allFinite() failure
[7] 完整 building02 正常结束
[8] run_valid = True
[9] NAV/timing 文件正常生成
[10] 完整回归通过后冻结本修复，再继续正式效率实验
```

---

## 13. 推荐实现顺序

```text
修改 LiDAR IMU coverage gate
        ↓
增加 RRR motion-state 返回值保护
        ↓
RelWithDebInfo 编译
        ↓
原故障窗口局部回放
        ↓
确认无 coverage / NaN / marginalization 异常
        ↓
完整 building02
        ↓
通过后冻结修复
```
