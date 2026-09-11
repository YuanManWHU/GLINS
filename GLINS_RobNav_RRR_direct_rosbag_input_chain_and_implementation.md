# GLINS RobNav RRR 直接读取 rosbag：输入链梳理与最小实现方案

## 0. 目标与边界

本方案只面向：

```text
数据集：RobNav
Estimator：rtk_imu_lidar_rrr
传感器：
  IMU      /adi/adis16465/imu
  LiDAR    Hesai AT128 /hesai/at128/points
  GNSS     rover raw observations
  GNSS     reference raw observations
  GNSS     broadcast ephemerides
  DCB      BSX file
```

目标不是把 GLINS 改造成通用离线框架，而是增加一个 **RobNav + 当前 RRR 专用的 direct-bag 入口**，使 GLINS 与 GRLINS 一样：

```text
rosbag::View 直接读取
    ↓
不按传感器时间 sleep
    ↓
尽可能快处理
    ↓
等待所有 frontend/backend 工作完成
    ↓
统计 wall processing time
```

核心约束：

```text
1. 不绕过现有 RosStream 的 ROS-message → DataCluster 转换。
2. 不绕过 DataIntegration。
3. 不绕过 MultiSensorEstimating。
4. 不改 RRR 数学模型。
5. 不改 GNSS/LiDAR/IMU 配置。
6. 不改 Ceres num_threads。
7. 保留现有 ROS runner 作为 correctness baseline。
8. 新 direct-bag 入口只负责离线输入与 wall-time benchmark。
```

---

# 1. 当前 GLINS ROS 模式的完整启动链

当前 `gici_ros_main` 的核心启动结构是：

```text
gici_ros_main
    ↓
YAML::LoadFile(config)
    ↓
NodeOptionHandle
    ↓
RosNodeHandle(nh, node_option_handle)
    ↓
SpinControl::run()
    ↓
ros::spin()
```

其中真正组织 estimator、streamer 和 DataIntegration 的是：

```text
NodeHandle
RosNodeHandle
```

需要特别注意：

```text
RosNodeHandle : public NodeHandle
```

所以构造 `RosNodeHandle` 时，会先执行基础类 `NodeHandle(nodes)`。

---

# 2. NodeHandle 阶段做了什么

`NodeHandle::NodeHandle()` 主要完成三件事。

## 2.1 创建非 ROS Streaming

它遍历 YAML `streamers`：

```cpp
if (type == StreamerType::Ros) continue;
```

所以：

```text
ROS IMU streamer       不在这里创建
ROS LiDAR streamer     不在这里创建
ROS GNSS streamer      不在这里创建
```

但是：

```text
fmt_dcb_file
```

所属的 file streamer / formator 会在这里正常创建。

因此 direct-bag 模式如果继续使用 `RosNodeHandle`：

> DCB 文件加载链可以完全保留，不需要新写 DCB reader。

---

## 2.2 创建 MultiSensorEstimating

对于：

```yaml
type: rtk_imu_lidar_rrr
```

创建：

```cpp
std::make_shared<MultiSensorEstimating>(nodes, i)
```

而 `MultiSensorEstimating` 内部进一步创建：

```text
RtkImuLidarRrrEstimator
TreeHandler
SPP initializer
GNSS/IMU initializer
auxiliary RTK estimator
ambiguity resolution
```

因此 direct-bag 模式也不需要重新构造 estimator。

---

## 2.3 启动线程

`NodeHandle` 最后调用：

```cpp
streamings_[i]->start();
estimatings_[i]->start();
```

所以构造完 `RosNodeHandle` 后：

```text
MultiSensorEstimating 的主 processing thread 已经启动
```

随后 `MultiSensorEstimating::process()` 会再启动：

```text
measurement thread
LiDAR frontend thread
backend thread
```

当前 RRR 没有 image frontend。

---

# 3. RosNodeHandle 阶段做了什么

基础 `NodeHandle` 构造完成之后：

```cpp
RosNodeHandle::RosNodeHandle(...)
```

才创建 ROS streamers。

当前 YAML 中和 RRR 有关的是：

```text
str_ros_gnss_rov
str_ros_gnss_ref
str_ros_imu
str_ros_lidar

以及多个 output RosStream
```

每个 `RosStream` 根据：

```yaml
type: ros
io: input/output
format: ...
topic_name: ...
```

创建：

```text
subscriber 或 publisher
```

之后最关键的一步是：

```cpp
rebindAllStreamerToEstimator(nodes);
```

它重新建立：

```text
RosStream
    ↓
DataIntegration
    ↓
MultiSensorEstimating
```

的 callback 链。

---

# 4. 当前 RRR 配置对应的数据角色

根据当前有效配置：

```yaml
input_tags:
  - str_ros_gnss_rov
  - str_ros_gnss_ref
  - fmt_dcb_file
  - str_ros_imu
  - str_ros_lidar
```

对应 role：

```text
str_ros_gnss_rov
    → rover

str_ros_gnss_ref
    → reference
    → ephemeris

fmt_dcb_file
    → code_bias

str_ros_imu
    → major

str_ros_lidar
    → front
```

`RosNodeHandle::rebindAllStreamerToEstimator()` 会据此构造：

```text
GnssDataIntegration
ImuDataIntegration
LidarDataIntegration
```

并让这些 DataIntegration 对象注册到对应 RosStream 的：

```cpp
setDataCallback(...)
```

上。

---

# 5. 当前真正的数据输入链

整体链不是：

```text
ROS subscriber
    → MultiSensorEstimating
```

而是：

```text
ROS subscriber
    ↓
RosStream callback
    ↓
DataCluster
    ↓
DataIntegration
    ↓
EstimatorDataCluster
    ↓
MultiSensorEstimating::estimatorDataCallback()
    ↓
measurement_addin_buffer_
```

这一点决定了 direct-bag 最合适的插入位置。

---

# 6. IMU 输入链

当前 ROS 模式：

```text
sensor_msgs::Imu
    ↓
RosStream::imuCallback()
```

`imuCallback()` 负责：

```text
header.stamp       → imu->time
linear_acceleration
angular_velocity
```

形成：

```cpp
DataCluster(FormatorType::IMUPack)
```

然后：

```cpp
for (auto callback : data_callbacks_) {
    callback(tag_, data_cluster);
}
```

进入：

```text
ImuDataIntegration::dataCallback()
```

`ImuDataIntegration` 再将其转换为：

```cpp
ImuMeasurement
EstimatorDataCluster(
    epoch,
    ImuRole::Major,
    tag
)
```

最后调用：

```cpp
MultiSensorEstimating::estimatorDataCallback()
```

### 因此 IMU 完整链为

```text
sensor_msgs::Imu
    ↓
RosStream IMU converter
    ↓
DataCluster::IMU
    ↓
ImuDataIntegration
    ↓
ImuMeasurement
    ↓
EstimatorDataCluster
    ↓
measurement_addin_buffer_
```

---

# 7. LiDAR 输入链

当前 RobNav 本地版本已经针对：

```text
Hesai AT128
PointCloud2 fields:
x y z intensity ring timestamp
```

增加了适配。

该适配已经完成并经过现有 ROS runner 验证，因此 direct-bag 模式不能重新实现另一套 AT128 converter。

原则是：

> direct-bag 必须调用当前 `format: hesai_at128` subscriber 实际绑定的同一个转换函数。

转换之后形成：

```cpp
DataCluster::LiDAR
```

其中当前适配语义为：

```text
timebase   = frame/header time
timefinal  = max/final point absolute time
curvature  = point timestamp - timebase
seq        = scan sequence
cloud_ptr  = 已执行当前 AT128 sampling 规则后的 Cloud
```

随后：

```text
LidarDataIntegration::dataCallback()
    ↓
LidarMeasurement(
    timebase,
    timefinal,
    seq
)
```

并设置：

```cpp
scan.need_frontend = true;
```

然后构造：

```cpp
EstimatorDataCluster(scan, LidarRole::Front, tag)
```

进入：

```cpp
MultiSensorEstimating::estimatorDataCallback()
```

### LiDAR 完整链

```text
sensor_msgs::PointCloud2 (AT128)
    ↓
现有 Hesai-AT128 RosStream converter
    ↓
DataCluster::LiDAR
    ↓
LidarDataIntegration
    ↓
LidarMeasurement(need_frontend=true)
    ↓
EstimatorDataCluster
    ↓
measurement_addin_buffer_
```

---

# 8. GNSS observation 输入链

## 8.1 RosStream 层

当前 rover/reference observation 都使用：

```cpp
gici_ros::GnssObservations
```

`RosStream::gnssObservationsCallback()` 会把每颗星观测转换成 RTKLIB：

```cpp
obsd_t
```

并填：

```text
week / tow
sat
SNR
LLI
code
L
P
D
```

然后：

```cpp
sortobs(...)
```

并形成：

```cpp
DataCluster(FormatorType::GnssRaw)
GnssDataType::Observation
```

之后调用 `data_callbacks_`。

---

## 8.2 GnssDataIntegration 层

这里非常重要。

`GnssDataIntegration` 不是简单“转发 observation”。

它内部维护：

```text
gnss_local_
code_bias_local_
phase_bias_local_
phase_center_local_
```

并负责：

```text
广播星历维护
DCB/TGD
天线位置
SSR（如果有）
satposs
卫星位置速度钟差
观测波长
GNSS role
```

对于 observation，它生成：

```cpp
GnssMeasurement epoch
```

包括：

```text
timestamp
role
tag
satellites
sat_position
sat_velocity
sat_clock
sat_frequency
pseudorange
phaserange
doppler
SNR
LLI
reference station position
code bias
phase bias
```

最后才：

```cpp
EstimatorDataCluster estimator_data(epoch)
```

并送给：

```cpp
MultiSensorEstimating::estimatorDataCallback()
```

### GNSS observation 完整链

```text
gici_ros::GnssObservations
    ↓
RosStream::gnssObservationsCallback
    ↓
RTKLIB obs_t / DataCluster::GNSS
    ↓
GnssDataIntegration
    ↓
satposs + ephemeris + DCB + antenna role processing
    ↓
GnssMeasurement
    ↓
EstimatorDataCluster
    ↓
measurement_addin_buffer_
```

---

# 9. GNSS ephemeris 输入链

当前：

```cpp
gici_ros::GnssEphemerides
```

通过：

```cpp
RosStream::gnssEphemeridesCallback()
```

转换为 RTKLIB：

```text
eph_t
geph_t
nav_t
```

形成：

```text
DataCluster::GNSS
GnssDataType::Ephemeris
```

随后：

```text
GnssDataIntegration
```

更新其内部：

```text
gnss_local_->ephemeris
```

并更新 TGD/code-bias base。

因此：

> ephemeris 不能在 direct-bag 中绕过 RosStream/GnssDataIntegration 直接塞到 RRR。

当前 runner 的“eph preload”也应在 direct-bag 模式中继续保留，只是从：

```text
rosbag play → ROS topic
```

变成：

```text
rosbag::View → RosStream offline feed
```

---

# 10. reference antenna position 输入链

当前 base ECEF 通过：

```cpp
gici_ros::GnssAntennaPosition
```

进入：

```cpp
RosStream::gnssAntennaPositionCallback
```

形成：

```text
GnssDataType::AntePos
```

`GnssDataIntegration` 只在具有：

```text
GnssRole::Reference
```

的输入上接受它，并更新：

```text
gnss_local_->antenna
```

之后 reference observation 才能正确形成带基站位置的 `GnssMeasurement`。

所以 direct-bag 仍然需要在正式输入前：

```text
feed reference antenna ECEF
```

---

# 11. DCB 链不需要修改

当前：

```text
fmt_dcb_file
```

不是 ROS input。

基础 `NodeHandle` 已经负责：

```text
file Streaming
    ↓
formator
    ↓
GnssDataIntegration
    ↓
code_bias_local_
```

因此 direct-bag 不应该重复读取 BSX。

只需要确保：

```text
NodeHandle 创建后
DCB file streamer 有机会完成初始化
```

正式 wall timer 应在这一静态初始化之后开始。

---

# 12. MultiSensorEstimating 收到数据之后的真实链路

所有 DataIntegration 最终调用：

```cpp
MultiSensorEstimating::estimatorDataCallback(
    EstimatorDataCluster& data)
```

该函数本身非常简单，只做：

```cpp
measurement_addin_buffer_.push_back(data);
```

它 **并不会同步执行 estimator**。

因此 GLINS 和 GRLINS 的架构区别就在这里：

```text
GRLINS
bag reader
    → process(sensor)
    → 同步返回
    → next bag message

GLINS
bag reader / ROS callback
    → measurement_addin_buffer_
    → 立即返回
    → 后台多个线程异步处理
```

这也是 direct-bag 不能简单“无限快把整个 bag 全灌进去然后退出”的原因。

---

# 13. IMU 在 MultiSensorEstimating 中怎么走

measurement thread：

```text
runMeasurementAddin()
    ↓
putMeasurements()
```

对于 IMU：

```cpp
handleTimePropagationSensors(data)
```

里面：

```text
estimator_->addMeasurement(IMU)
latest_imu_timestamp_ = data.timestamp
```

所以 IMU 是：

```text
addin queue
    ↓
measurement thread
    ↓
直接进入 estimator IMU buffer
```

不经过 backend `measurements_` queue。

---

# 14. LiDAR 在 MultiSensorEstimating 中怎么走

第一次进入 `putMeasurements()` 时：

```cpp
data.lidar->need_frontend == true
```

所以：

```text
measurement_addin_buffer_
    ↓
lidar_frontend_measurements_
```

LiDAR frontend thread：

```text
runLidarFrontend()
    ↓
tree_handler_->processLidar(scan)
```

随后生成新的：

```cpp
EstimatorDataCluster measurement(*scan, role, tag)
```

这里：

```text
need_frontend = false
```

再调用：

```cpp
estimatorDataCallback(measurement)
```

于是 processed LiDAR 又回到：

```text
measurement_addin_buffer_
```

第二次经过 measurement thread 后：

```text
handleNonTimePropagationSensors
    ↓
measurement_align_buffer_
    ↓
measurements_
    ↓
backend
```

### 所以 LiDAR 实际是两次进入 addin queue

```text
raw LiDAR
    ↓
addin
    ↓
LiDAR frontend
    ↓
processed LiDAR
    ↓
addin
    ↓
alignment
    ↓
backend
```

这对 direct-bag 的 drain 判据非常重要。

---

# 15. rover GNSS 与 reference GNSS 在 backend 前并不完全相同

在 `putMeasurements()` 中：

### reference GNSS

```cpp
data.gnss_role == GnssRole::Reference
```

直接：

```text
measurements_.push_back(data)
```

即：

```text
不经过 measurement_align_buffer_
```

### rover GNSS

走：

```text
handleNonTimePropagationSensors()
    ↓
measurement_align_buffer_
    ↓
measurements_
```

RRR estimator 内部再通过：

```text
measurement_align_
```

配对 rover/reference epoch。

---

# 16. backend 链

backend thread：

```text
runBackend()
    ↓
processEstimator()
```

其核心：

```cpp
measurement = measurements_.front()
measurements_.pop_front()
```

若尚无全局坐标：

```text
SPP establishment of coordinate
```

然后：

```cpp
estimator_->addMeasurement(measurement)
```

若返回 true：

```cpp
estimator_->estimate()
```

当前 RRR：

```text
RtkImuLidarRrrEstimator
```

因此 downstream 的数学算法完全在这里。

direct-bag 模式绝不应该绕过这一层。

---

# 17. output 链

当 backend 更新成功时：

```text
backend_firstly_updated_ = true
```

并根据：

```yaml
output_align_tag: str_ros_imu
output_downsample_rate: 40
```

在约 5 Hz 保存：

```text
output_timestamps_
```

`EstimatingBase` 主线程随后：

```text
updateSolution()
    ↓
output_data_callbacks_
```

当前 RosNodeHandle 将这些 callback 绑定到：

```text
RosStream outputDataCallback
```

因此当前：

```text
solution
solution_odometry
solution_navsatfix
solution_path
cloud_current
cloud_map
landmark
```

输出链在 direct-bag 中也可以原样保留。

无需启动 RViz。

---

# 18. `rosbag::View` 可以插在哪几层

下面比较四个候选位置。

## 方案 A：rosbag::View → ROS publish → RosStream subscriber

```text
rosbag::View
    ↓
ros::Publisher
    ↓
ROS queue
    ↓
RosStream subscriber
```

优点：

```text
几乎不改 GLINS
```

缺点：

```text
高速度离线输入时 subscriber queue 可能积压或丢消息
仍经过 ROS transport
需要 spinner
吞吐受 ROS queue 行为影响
不是真正 direct feed
```

结论：

```text
不推荐用于 wall-time benchmark
```

---

## 方案 B：rosbag::View → 直接构造 DataCluster → DataIntegration

优点：

```text
绕开 ROS transport
```

缺点：

```text
必须复制 IMU converter
必须复制 AT128 converter
必须复制 GNSS raw converter
```

尤其 GNSS conversion 复杂，极易造成：

```text
ROS mode
与
offline mode
```

输入语义不一致。

结论：

```text
不推荐
```

---

## 方案 C：rosbag::View → 直接构造 EstimatorDataCluster → MultiSensorEstimating

这会绕过：

```text
GnssDataIntegration
```

从而绕过：

```text
ephemeris maintenance
satposs
DCB
antenna position
GNSS roles
```

并且需要自己重新实现：

```text
LidarMeasurement
ImuMeasurement
GnssMeasurement
```

结论：

```text
明确不可取
```

---

## 方案 D：rosbag::View → RosStream 的现有转换函数

链为：

```text
rosbag::View
    ↓
MessageInstance::instantiate<T>()
    ↓
RosStream offline feed
    ↓
现有 RosStream callback/converter
    ↓
DataCluster
    ↓
DataIntegration
    ↓
EstimatorDataCluster
    ↓
MultiSensorEstimating
```

优点：

```text
完全复用当前已验证的转换链
不走 ROS subscriber queue
不重复实现 AT128
不重复实现 GNSS raw
不绕过 DataIntegration
不改 estimator
```

结论：

```text
这是本项目的推荐方案
```

---

# 19. 最终决定的插入位置

最终建议：

> **将 `rosbag::View` 插在 RosStream subscriber callback 的“前一层”。**

也就是说：

当前：

```text
ROS subscriber
    ↓
RosStream::xxxCallback(msg)
```

新增：

```text
rosbag::View
    ↓
RosStream::feedXxx(msg)
    ↓
RosStream::xxxCallback(msg)
```

其中：

```cpp
feedXxx()
```

只是薄封装：

```cpp
void RosStream::feedImu(const sensor_msgs::ImuConstPtr& msg)
{
    imuCallback(msg);
}
```

不写第二套 converter。

---

# 20. RosStream 的最小 API 扩展

文件：

```text
ros_wrapper/src/gici/include/gici/ros_interface/ros_stream.h
```

增加 public offline ingress API，例如：

```cpp
void feedImu(const sensor_msgs::ImuConstPtr& msg);
void feedPointCloud2(const sensor_msgs::PointCloud2ConstPtr& msg);
void feedGnssObservations(const gici_ros::GnssObservationsConstPtr& msg);
void feedGnssEphemerides(const gici_ros::GnssEphemeridesConstPtr& msg);
void feedGnssAntennaPosition(const gici_ros::GnssAntennaPositionConstPtr& msg);
```

实现：

```cpp
void RosStream::feedImu(const sensor_msgs::ImuConstPtr& msg)
{
    imuCallback(msg);
}
```

其余同理。

---

# 21. AT128 特别说明

当前本地版本 YAML 已经使用：

```yaml
format: hesai_at128
```

因此实现前必须在本地确认：

```text
format: hesai_at128
```

实际 subscriber 绑定的是哪个 callback，例如可能是：

```text
hesaiAt128Callback(...)
```

也可能当前适配直接复用了：

```text
pc2Callback(...)
```

direct-bag 必须调用 **同一个实际 callback**。

不要根据 upstream `pointcloud2` 代码重新写 AT128 converter。

本方案中的：

```cpp
feedPointCloud2(...)
```

只表示“调用当前 AT128 已验证 converter”，具体函数名以本地当前实现为准。

---

# 22. RosNodeHandle 需要一个最小 getter

当前：

```cpp
getRosStreamFromTag(...)
```

是 protected。

direct-bag feeder 需要获得：

```text
str_ros_imu
str_ros_lidar
str_ros_gnss_rov
str_ros_gnss_ref
```

对应对象。

建议增加 public：

```cpp
std::shared_ptr<RosStream> getRosStream(const std::string& tag)
{
    return getRosStreamFromTag(tag);
}
```

或者把现有 getter 移到 public。

推荐新增 wrapper，避免改变原 protected helper 的结构。

---

# 23. 新增专用 executable，而不是修改 `gici_ros_main`

新增：

```text
ros_wrapper/src/gici/src/gici_robnav_bag_main.cpp
```

不要修改：

```text
gici_ros_main
```

原因：

```text
现有 ROS runner 已验证通过
需要保留 correctness baseline
direct-bag 是 benchmark 专用入口
二者应能并存
```

最终：

```text
gici_ros_main
    → 原 ROS topic 模式

gici_robnav_bag_main
    → RobNav direct rosbag 模式
```

---

# 24. 新 executable 的启动结构

推荐：

```cpp
int main(...)
{
    ros::init(...);

    // 1. load YAML
    // 2. NodeOptionHandle
    // 3. RosNodeHandle
    // 4. get input RosStreams
    // 5. SpinControl::run()

    // 6. wait for static file input (DCB)
    // 7. direct ephemeris preload
    // 8. direct antenna-position feed

    // 9. open formal bags / create merged rosbag::View

    // 10. start wall timer
    // 11. iterate view and feed existing RosStream conversion
    // 12. notify end of input
    // 13. wait pipeline drain
    // 14. stop wall timer

    // 15. graceful shutdown / Recorder flush
}
```

不调用：

```cpp
ros::spin()
```

因为正式输入不依赖 subscriber callback queue。

---

# 25. 为什么仍然使用 RosNodeHandle

虽然 direct-bag 不使用 ROS subscriber 输入，但仍建议构造：

```cpp
RosNodeHandle
```

因为这样可以原样复用：

```text
YAML parsing
MultiSensorEstimating construction
RRR estimator construction
DCB file streamer
DataIntegration role binding
output RosStream binding
Recorder
所有当前配置
```

输入 subscriber 虽然会被创建，但由于不做 `ros::spin()` 且没有外部 publisher，其 callback 不会参与 formal input。

因此无需再造一套 OfflineNodeHandle。

---

# 26. 正式 bag 的组织方式

当前 formal input 主要来自三个 bag：

```text
sensor bag
rover observation bag
reference observation bag
```

ephemeris bag：

```text
计时前 preload
```

DCB：

```text
由现有 file streamer 载入
```

antenna：

```text
计时前 direct feed
```

---

# 27. 合并多 bag 时间序列

推荐：

```cpp
rosbag::Bag sensor_bag;
rosbag::Bag rover_bag;
rosbag::Bag reference_bag;

rosbag::View view;

view.addQuery(sensor_bag, ...);
view.addQuery(rover_bag, ...);
view.addQuery(reference_bag, ...);
```

formal topics：

```text
sensor:
  IMU
  AT128

rover:
  observations

reference:
  observations
```

然后：

```cpp
for (const rosbag::MessageInstance& m : view)
```

按 bag record time 顺序读取。

首次实现建议增加检查：

```text
current_record_time >= last_record_time
```

若出现倒序则直接报错。

---

# 28. 每种消息如何 direct feed

伪代码：

```cpp
for (const auto& msg : view) {

    if (msg.getTopic() == imu_topic) {
        auto p = msg.instantiate<sensor_msgs::Imu>();
        imu_stream->feedImu(p);
    }
    else if (msg.getTopic() == lidar_topic) {
        auto p = msg.instantiate<sensor_msgs::PointCloud2>();
        lidar_stream->feedPointCloud2(p);
    }
    else if (/* rover observations */) {
        auto p = msg.instantiate<gici_ros::GnssObservations>();
        rover_stream->feedGnssObservations(p);
    }
    else if (/* reference observations */) {
        auto p = msg.instantiate<gici_ros::GnssObservations>();
        ref_stream->feedGnssObservations(p);
    }
}
```

这里：

```text
instantiate<T>()
ROS-msg → DataCluster conversion
```

均计入 wall processing time。

这与 GRLINS direct read 的语义更接近。

---

# 29. ephemeris preload

正式 wall timer 开始前：

```cpp
rosbag::Bag eph_bag;
rosbag::View eph_view(...);

for (...) {
    auto eph = msg.instantiate<gici_ros::GnssEphemerides>();
    ref_stream->feedGnssEphemerides(eph);
}
```

因为 callback/DataIntegration 是同步执行的：

```text
feedGnssEphemerides() 返回时
该 ephemeris 已经进入 GnssDataIntegration 的 gnss_local_
```

不需要 ROS topic publish。

---

# 30. antenna prefeed

当前 base ECEF：

```text
[-2267218.02758,
  5009581.77187,
  3221148.16948]
```

可以由 Python wrapper 传入 executable，也可以作为当前 RobNav profile 参数传入。

构造：

```cpp
gici_ros::GnssAntennaPosition
```

并：

```cpp
ref_stream->feedGnssAntennaPosition(...)
```

在 formal timer 开始之前完成。

不建议将基站坐标硬编码到 estimator。

---

# 31. wall timer 的定义

建议使用：

```cpp
std::chrono::steady_clock
```

开始：

```text
ephemeris preload 完成
antenna feed 完成
静态配置/DCB已就绪
formal rosbag::View 即将开始
```

即：

```cpp
wall_start = steady_clock::now();
```

结束：

```text
formal view 已读完
所有正式输入已消费
LiDAR frontend 已清空
alignment 已清空
backend 已清空
最后一次 active computation 已完成
```

然后：

```cpp
wall_end = steady_clock::now();
```

定义：

```text
wall_processing_time_s =
    wall_end - wall_start
```

---

# 32. 这个 wall time 包含什么

包含：

```text
rosbag 文件读取
ROS message instantiate/deserialization
IMU converter
AT128 converter
GNSS converter
GnssDataIntegration / satposs
LiDAR frontend
measurement alignment
RRR addMeasurement
FGO optimization
GNSS post
LiDAR post
marginalization
正常运行期的 output preparation/callback
线程间同步/调度
```

不包含：

```text
程序启动
YAML parsing
静态 DCB preload
ephemeris preload
antenna prefeed
最终 Recorder 写盘 flush
```

这一定义应在后续与 GRLINS 的计时起止边界统一。

---

# 33. direct-bag 最大的技术点不是读 bag，而是 drain

GRLINS 是同步：

```text
process(sensor) 返回
→ 当前 sensor 处理结束
```

GLINS 是异步：

```text
feed
→ queue
→ frontend/backend thread
```

所以：

```text
rosbag::View 循环结束
```

不等于：

```text
算法处理结束
```

必须建立明确的 end-of-input + pipeline drain 机制。

---

# 34. GLINS 需要检查的队列

当前 RRR 至少涉及：

```text
measurement_addin_buffer_
measurement_align_buffer_
lidar_frontend_measurements_
measurements_
output_timestamps_
```

此外还要避免：

```text
队列已被 pop
但对应线程仍正在计算
```

所以只检查：

```text
queue.empty()
```

还不够。

---

# 35. 推荐增加 offline-only pipeline state

文件：

```text
include/gici/fusion/multisensor_estimating.h
src/fusion/multisensor_estimating.cpp
```

增加：

```text
input_finished_
measurement_busy_
lidar_frontend_busy_
backend_busy_
```

建议用：

```cpp
std::atomic<bool>
```

并增加 public：

```cpp
void notifyInputFinished();
bool pipelineIdle();
```

---

# 36. busy flag 的语义

### measurement thread

处理一条 addin measurement 前：

```cpp
measurement_busy_ = true;
```

完成后：

```cpp
measurement_busy_ = false;
```

### LiDAR frontend

pop 出 LiDAR 之后，在：

```text
processLidar
visualization preparation
estimatorDataCallback(processed scan)
```

整个阶段：

```cpp
lidar_frontend_busy_ = true;
```

完成：

```cpp
false
```

### backend

从 `measurements_` pop 开始，到：

```text
addMeasurement
estimate
post
marginalization
```

全部结束：

```cpp
backend_busy_ = false;
```

---

# 37. measurement_align_buffer_ 的 EOF 问题

当前 alignment release 条件包含：

```text
back.timestamp - front.timestamp >= input_align_latency
```

正常在线运行时，靠未来 measurement 推动旧 measurement 释放。

但 direct-bag 到 EOF 后：

```text
没有下一条 non-IMU measurement
```

最后少量：

```text
rover GNSS
processed LiDAR
```

可能永远停在：

```text
measurement_align_buffer_
```

因此 direct-bag 必须具有：

```text
EOF flush
```

语义。

---

# 38. EOF flush 原则

收到：

```cpp
notifyInputFinished()
```

之后：

```text
不再需要为了等待未来 sensor 重排序而保留 input_align_latency
```

因此允许：

```text
忽略 latency hold 条件
```

但仍必须保留：

```text
IMU coverage condition
```

特别是 LiDAR：

```text
latest_imu_timestamp >= lidar->timefinal
```

才能释放。

也就是说 EOF flush 只是取消：

```text
“还可能有更早的未来消息到来”
```

这一在线假设。

绝不能取消：

```text
LiDAR 所需 IMU 已覆盖完整 scan
```

这一物理条件。

---

# 39. pipelineIdle() 建议条件

只有同时满足：

```text
input_finished == true

measurement_addin_buffer_.empty()
measurement_align_buffer_.empty()
lidar_frontend_measurements_.empty()
measurements_.empty()
output_timestamps_.empty()

measurement_busy_ == false
lidar_frontend_busy_ == false
backend_busy_ == false
```

并且连续稳定若干个检查周期，才认为 drain 完成。

建议：

```text
check interval = 10~20 ms
stable duration ≈ 100 ms
timeout = 60 s 或更高
```

这里的 100 ms 只用于确认 idle，不应额外计入 wall total。

与之前 CPU-idle drain 类似：

> 第一次进入稳定 idle 时保存 candidate wall_end；后续稳定窗口只用于确认，不把稳定确认时间计入 wall processing time。

---

# 40. 为什么不继续用 process CPU idle 判据

direct-bag wall benchmark 已经处于同一进程内部，可以直接知道：

```text
队列状态
busy state
```

因此不应该再用：

```text
CPU 使用率低
```

去猜测 backend 是否结束。

显式 pipeline state 更可靠。

---

# 41. 输入过快与内存积压

如果 direct reader 无限制：

```text
for (msg : view)
    feed(msg)
```

理论上 main thread 可能比 estimator 消费更快。

因为：

```text
estimatorDataCallback()
```

只是 push queue 后立即返回。

风险：

```text
measurement_addin_buffer_ 增长
LiDAR frontend queue 增长
内存增长
调度行为变化
```

---

# 42. 首版建议：监测 peak pending，暂不改变算法

为了保持最小修改，第一版可以：

```text
不主动 throttle
```

但必须记录：

```text
peak measurement_addin_buffer size
peak lidar_frontend queue size
peak backend measurements size
```

先用：

```text
100 s
完整 street00
```

验证。

如果 peak 很小/可控，则无需 flow control。

---

# 43. 如果出现大 backlog，再加高水位 flow control

只有出现：

```text
内存快速增长
queue 积压非常大
```

时，第二阶段增加 feeder high-water mark。

例如：

```text
addin queue > threshold
或
lidar frontend queue > threshold
```

则 bag reader 暂停几毫秒，等 estimator 消费。

这个等待属于：

```text
算法无法继续接受输入导致的 processing time
```

所以应计入 wall processing time。

但阈值必须足够高，避免人为限制 frontend/backend 并行。

第一版不先引入这一复杂度。

---

# 44. 新增的代码文件建议

## 必须新增

```text
ros_wrapper/src/gici/src/gici_robnav_bag_main.cpp
```

职责：

```text
load config
construct RosNodeHandle
get input streams
open bags
preload eph/antenna
create merged formal view
direct feed
wall timer
notify EOF
drain
write direct-run summary
```

---

## 可选新增

如果 main 过长，再拆：

```text
ros_wrapper/src/gici/include/gici/ros_interface/robnav_bag_feeder.h
ros_wrapper/src/gici/src/ros_interface/robnav_bag_feeder.cpp
```

首版建议：

```text
先不拆
```

因为只支持 RobNav RRR，避免做成过度通用框架。

---

# 45. 需要小改的现有文件

### 1. RosStream

```text
ros_stream.h
ros_stream.cpp
```

增加 public direct-feed wrappers。

### 2. RosNodeHandle

```text
ros_node_handle.h
```

增加 public `getRosStream(tag)` wrapper。

### 3. MultiSensorEstimating

```text
multisensor_estimating.h
multisensor_estimating.cpp
```

增加：

```text
EOF notification
pipeline idle
busy flags
EOF alignment flush
peak queue diagnostics（建议）
```

### 4. CMake

```text
ros_wrapper/src/gici/CMakeLists.txt
```

增加：

```text
rosbag dependency
gici_robnav_bag_main executable
```

---

# 46. CMake 修改方向

catkin component 加：

```cmake
rosbag
```

新增：

```cmake
add_executable(gici_robnav_bag_main
               src/gici_robnav_bag_main.cpp
               ${DIR_ROS_TOOLS})

target_link_libraries(gici_robnav_bag_main
      gici
      ${OpenCV_LIBS}
      ${PCL_LIBRARIES}
      ${catkin_LIBRARIES})
```

具体是否复用全部 `${DIR_ROS_TOOLS}`，在实现时根据本地当前 CMake 再确认。

---

# 47. 不建议直接修改当前 Python ROS runner

当前：

```text
run_robnav_rrr.py
```

已经完成：

```text
路径发现
ROS topic playback
结果目录
runtime summary
正确性验证
```

并且已真实跑通。

所以 direct-bag 首版应保持它不变。

新增 C++ executable 单独验证。

direct-bag 完成并验证后，再决定：

```text
新增 run_robnav_rrr_offline.py
```

或：

```text
给现有 runner 增加 mode
```

不要一开始就把已经稳定的 ROS runner 改复杂。

---

# 48. direct executable 的参数

为了不在 C++ 中重复 Python 的数据集 discovery，建议 executable 接受显式参数。

例如：

```text
config
sensor_bag
rover_bag
reference_bag
ephemeris_bag

imu_topic
lidar_topic
rover_obs_topic
reference_obs_topic
ephemeris_topic

base_ecef_x
base_ecef_y
base_ecef_z

start_time
duration
result_dir
```

首版可以命令行传入。

后续 Python wrapper 自动填这些参数。

---

# 49. 为什么路径 discovery 不写进 C++

因为：

```text
run_robnav_rrr.py
```

已经解决了：

```text
building02 / street00 / street01 / street02
GNSS 日期目录
rover/reference/eph bag
DCB
start offset
```

这些属于 experiment orchestration，不属于 estimator input pipeline。

C++ direct executable 应只负责：

```text
给我明确文件和 topic
我负责准确、高速地处理
```

---

# 50. 正确性验证：第一阶段 100 s

首先使用和当前 ROS runner 相同的：

```text
building02 前 100 s
```

direct-bag 跑一次。

检查：

```text
run completes
NAV generated
timing generated
no NaN
no IMU coverage error
```

同时记录：

```text
wall_processing_time_s
process_cpu_time_s（建议同时保留）
peak queues
```

---

# 51. 轨迹一致性验证

direct-bag 最重要的不是“能跑”，而是：

> direct 模式与现有 1× ROS 模式必须得到同一算法结果。

因此对同一 100 s：

```text
ROS baseline NAV
vs
direct-bag NAV
```

按 GPS SOW 对齐。

重点检查位置：

```text
latitude
longitude
height
```

建议转换到 ENU 后统计：

```text
max horizontal difference
RMS horizontal difference
max vertical difference
```

目标：

```text
应仅有数值级/异步调度可接受差异
不能出现轨迹级差异
```

如果差异明显，先查输入顺序与 EOF/queue，而不是继续跑正式 benchmark。

---

# 52. message count 一致性验证

对同一时间段统计：

```text
IMU feed count
LiDAR feed count
rover observation count
reference observation count
ephemeris preload count
```

与当前 ROS runner / bag 原始 count 对比。

尤其：

```text
LiDAR frontend count
backend add count
```

应解释一致。

---

# 53. 输入顺序验证

direct-bag 首版输出：

```text
first / last record time
count by source
number of descending record timestamps
```

要求：

```text
descending = 0
```

还建议检查每类 header time：

```text
IMU timestamp monotonically increasing
LiDAR timebase increasing
GNSS epoch nondecreasing
```

---

# 54. 完整 street00 验证

100 s 一致后，再用：

```text
street00
```

做第一次完整 direct-bag benchmark。

原因：

```text
当前 street00 已有成功 ROS baseline
sequence ≈ 1401 s
NAV / timing / process CPU 都有效
```

所以最适合做：

```text
ROS 1× correctness baseline
vs
direct-bag correctness + wall benchmark
```

---

# 55. 成功后再跑四组

只有完整 street00 满足：

```text
结果一致
无 drop
无异常 backlog
drain 正确
wall time 可复现
```

之后才跑：

```text
building02
street00
street01
street02
```

正式 wall-time 实验。

---

# 56. wall-time 重复性

论文正式结果建议每组至少：

```text
3 runs
```

记录：

```text
wall_processing_time_s
process_cpu_time_s
```

主表使用：

```text
mean wall processing time
```

可附：

```text
std
```

如果三次波动 < 数个百分点，则结果稳定。

---

# 57. 与 GRLINS 的公平性

direct-bag 完成后，两边的输入模式可以统一成：

```text
GRLINS:
rosbag::View direct read
→ process as fast as possible
→ wall elapsed

GLINS:
rosbag::View direct read
→ existing multithread pipeline
→ full drain
→ wall elapsed
```

这比：

```text
GRLINS wall
vs
GLINS process CPU
```

更直接。

但是最终论文比较前还需统一：

```text
formal sequence start
formal sequence end
timer start semantics
timer end semantics
```

特别是 GRLINS 表格中的序列长度与完整 RobNav bag duration 不完全相同，这个问题仍需要单独解决。

---

# 58. 最终推荐架构

```text
                        ┌─────────────────────┐
                        │  rosbag::View       │
                        │  RobNav direct read │
                        └─────────┬───────────┘
                                  │
                instantiate original ROS msg types
                                  │
                                  ▼
                     ┌────────────────────────┐
                     │ RosStream offline feed │
                     │ thin public wrappers   │
                     └───────────┬────────────┘
                                 │
                    existing converter callbacks
                                 │
                                 ▼
                         ┌─────────────┐
                         │ DataCluster │
                         └──────┬──────┘
                                │
                                ▼
               ┌────────────────────────────────┐
               │ DataIntegration                │
               │ GNSS / IMU / LiDAR             │
               └──────────────┬─────────────────┘
                              │
                              ▼
                    ┌──────────────────────┐
                    │ EstimatorDataCluster │
                    └──────────┬───────────┘
                               │
                               ▼
             ┌───────────────────────────────────┐
             │ MultiSensorEstimating             │
             │ measurement_addin_buffer_         │
             └──────────────┬────────────────────┘
                            │
       ┌────────────────────┼─────────────────────┐
       │                    │                     │
       ▼                    ▼                     ▼
      IMU             LiDAR frontend       GNSS / processed LiDAR
       │                    │                     │
       │                    └──────┐              │
       │                           ▼              │
       │                     addin again          │
       │                           │              │
       └───────────────────────────┼──────────────┘
                                   ▼
                              measurements_
                                   │
                                   ▼
                         RtkImuLidarRrrEstimator
                                   │
                                   ▼
                              optimize/post
                                   │
                                   ▼
                               solution/NAV
```

---

# 59. 最终实现位置结论

本方案最终建议：

## direct bag 读取位置

```text
新增：
ros_wrapper/src/gici/src/gici_robnav_bag_main.cpp
```

`rosbag::View` 只存在这里。

---

## ROS message conversion 复用位置

```text
现有：
RosStream
```

只增加薄封装：

```text
feedImu
feedHesaiAt128/PointCloud2
feedGnssObservations
feedGnssEphemerides
feedGnssAntennaPosition
```

内部调用现有 callback。

---

## downstream 不变

```text
DataIntegration
MultiSensorEstimating normal processing
RtkImuLidarRrrEstimator
TreeHandler
GNSS algorithms
timing recorder
NAV recorder
```

全部保持现有逻辑。

---

## 为 offline EOF 需要的小扩展

```text
MultiSensorEstimating
```

增加：

```text
notifyInputFinished
pipelineIdle
EOF alignment flush
busy flags
```

仅服务于 direct-bag 的“何时全部处理完成”判定。

---

# 60. 推荐实现阶段

## Stage A — 只做 direct ingress

```text
A1 RosStream public feed wrappers
A2 RosNodeHandle public stream getter
A3 gici_robnav_bag_main
A4 CMake rosbag dependency
```

先实现 100 s direct feed。

此时可用临时较长 wait 观察是否能够正常输出，但不做正式 wall benchmark。

---

## Stage B — 正确 EOF/drain

```text
B1 input_finished flag
B2 EOF align flush
B3 measurement/lidar/backend busy flags
B4 pipelineIdle
B5 candidate wall_end
```

完成后才认为 wall time 定义可靠。

---

## Stage C — 一致性验证

```text
C1 100 s ROS vs direct NAV
C2 message counts
C3 timing counts
C4 queue peak
C5 no drop / no NaN
```

---

## Stage D — 完整 street00

```text
D1 direct complete run
D2 trajectory consistency
D3 wall/process CPU
D4 repeatability
```

---

## Stage E — 正式四组

在统一 GRLINS/GLINS sequence interval 后再做。

---

# 61. 当前不应做的事情

```text
不要把 rosbag 100× publish 到 ROS topic 代替 direct-bag。

不要直接调用 MultiSensorEstimating::estimatorDataCallback
而重新构造 EstimatorDataCluster。

不要绕过 GnssDataIntegration。

不要复制一份 AT128 converter。

不要修改 EstimatorDataCluster 的 LiDAR timestamp 语义。

不要为了 direct-bag 改 input_align_latency。

不要为了 wall benchmark 修改 Ceres num_threads。

不要删除 current ROS runner。

不要一开始把 direct-bag 做成支持 GLINS 全部 estimator 的通用框架。
```

---

# 62. 最终结论

针对“只支持 RobNav + 当前 RRR”这一目标，最小且风险最低的方案不是重构 GLINS 的 Streaming 框架，而是：

```text
保留 RosNodeHandle
保留 RosStream
保留 DataIntegration
保留 MultiSensorEstimating
保留 RRR

新增一个 gici_robnav_bag_main
        ↓
rosbag::View 直接读取原始 bag
        ↓
调用 RosStream 当前 converter 的 public feed wrapper
        ↓
下游完全走原有链
```

这样 direct-bag 模式与当前 ROS 模式之间唯一实质区别是：

```text
数据如何到达 RosStream converter
```

当前：

```text
rosbag play → ROS subscriber → converter
```

新模式：

```text
rosbag::View → converter
```

从 `DataCluster` 开始，两条链完全合流。

这正是本项目为了获得可与 GRLINS 对比的 wall processing time 所需要的最小改动边界。
