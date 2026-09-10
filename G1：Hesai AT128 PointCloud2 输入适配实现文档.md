# G1：Hesai AT128 PointCloud2 输入适配实现文档

## 1. 目标

在 GLINS 现有 ROS LiDAR 输入框架中新增 **Hesai AT128 专用 `sensor_msgs::PointCloud2` 输入适配逻辑**，使 RobNav 数据集中的：

```text
/hesai/at128/points
```

能够被正确转换为 GLINS 内部 `LidarMeasurement / Cloud` 格式。

本次修改必须满足：

1. **保留现有 Livox 适配逻辑不变**；
2. **保留现有 `format: pointcloud2` 适配逻辑不变**；
3. 新增独立：
   ```yaml
   format: hesai_at128
   ```
4. AT128 的绝对 `timestamp` 转换成 GLINS 内部要求的相对点时间；
5. AT128 分支不执行现有通用 `pointcloud2` 中的：
   ```cpp
   src_point.y < -1
   ```
   点云裁剪；
6. `timefinal` 根据该帧点云实际最大时间确定，而不是假定最后一个点时间最大；
7. LiDAR ROS adapter 之后的 frontend、deskew、estimator、RRR 等模块不做任何 AT128 特殊处理。

---

# 2. 已确认的 AT128 输入格式

RobNav 数据集中 `/hesai/at128/points` 的 `PointCloud2` 字段为：

| field | datatype | offset | 含义 |
|---|---:|---:|---|
| `x` | FLOAT32 | 0 | LiDAR x |
| `y` | FLOAT32 | 4 | LiDAR y |
| `z` | FLOAT32 | 8 | LiDAR z |
| `intensity` | FLOAT32 | 12 | intensity |
| `ring` | UINT16 | 16 | laser ring |
| `timestamp` | FLOAT64 | 18 | **绝对 Unix/ROS 秒** |

经过 building02 数据验证：

```text
header.stamp = min(point.timestamp)
point timestamp span ≈ 0 ~ 0.052 s
LiDAR message period ≈ 0.1 s
LiDAR rate ≈ 10 Hz
```

因此正确的内部点时间定义为：

\[
\boxed{
t_i^{rel}
=
timestamp_i-
header.stamp
}
\]

绝对不能：

- 将 `timestamp` 直接写入 `curvature`；
- 将时间除以 `1e9`；
- 将 52 ms 人为缩放为 100 ms；
- 根据 point index 人工生成时间。

---

# 3. GLINS 当前行为

当前通用 PointCloud2 输入使用：

```cpp
PointXYZIRT
```

其字段为：

```cpp
x, y, z        float
intensity      float
ring           uint16
time           float
```

其中 `time` 的语义已经定义为：

> offset from the scan header timestamp, in seconds



当前 `pc2Callback()`：

```cpp
pcl::fromROSMsg(...)

point.curvature = src_point.time;
point.normal_z = src_point.ring;

timebase = header.stamp;
timefinal = header.stamp + last_point.time;
```

而且还包含：

```cpp
src_point.y < -1
```

过滤条件。

这些行为属于现有通用 `pointcloud2` adapter。

**本次不得通过修改 `pc2Callback()` 来适配 AT128。**

---

# 4. 总体设计

新增独立数据路径：

```text
sensor_msgs::PointCloud2
        │
        │ format: hesai_at128
        ▼
hesaiAt128Callback()
        │
        │ absolute timestamp
        │        ↓
        │ timestamp - header.stamp
        ▼
GLINS internal Cloud
Point_lidar.curvature = relative time [s]
        │
        ▼
existing LiDAR frontend / deskew / estimator
```

现有路径继续保持：

```text
format: livox
    → livoxCallback()

format: pointcloud2
    → pc2Callback()

format: hesai_at128
    → hesaiAt128Callback()
```

---

# 5. 修改范围

预计只修改以下 3 个文件：

```text
include/gici/lidar/lidar_types.h

ros_wrapper/src/gici/include/gici/ros_interface/ros_stream.h

ros_wrapper/src/gici/src/ros_interface/ros_stream.cpp
```

原则上：

```text
不修改 CMakeLists.txt
不修改 LiDAR frontend
不修改 estimator
不修改 deskew
不修改现有 PointXYZIRT
不修改 pc2Callback()
不修改 livoxCallback()
不新增 FormatorType
```

---

# 6. 修改 1：增加 AT128 原始 PointCloud2 点类型

文件：

```text
include/gici/lidar/lidar_types.h
```

在现有：

```cpp
struct EIGEN_ALIGN16 PointXYZIRT
```

附近新增：

```cpp
// Hesai AT128 PointCloud2 input.
// timestamp is an absolute Unix/ROS timestamp in seconds.
struct EIGEN_ALIGN16 PointXYZIRTAT128 {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
```

并增加 PCL registration：

```cpp
POINT_CLOUD_REGISTER_POINT_STRUCT(
    gici::PointXYZIRTAT128,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp))
```

## 要求

必须严格使用：

```cpp
double timestamp;
```

不能使用：

```cpp
float timestamp;
```

因为原始 ROS 字段是：

```text
timestamp FLOAT64
```

而其数值约为：

```text
1.745893277e9 s
```

如果先转成 float 再减 header timestamp，会发生严重有效位损失。

因此计算顺序必须是：

```cpp
double relative_time =
    src_point.timestamp - base_time;

point.curvature =
    static_cast<float>(relative_time);
```

即：

\[
\boxed{
double\ absolute
\rightarrow
double\ difference
\rightarrow
float\ relative
}
\]

而不能：

\[
double\ absolute
\rightarrow
float\ absolute
\rightarrow
difference.
\]

---

# 7. 修改 2：声明 AT128 callback

文件：

```text
ros_wrapper/src/gici/include/gici/ros_interface/ros_stream.h
```

当前已经有：

```cpp
void livoxCallback(
    const livox_ros_driver::CustomMsg::ConstPtr& msg);

void pc2Callback(
    const sensor_msgs::PointCloud2::ConstPtr& msg);
```



新增：

```cpp
void hesaiAt128Callback(
    const sensor_msgs::PointCloud2::ConstPtr& msg);
```

建议放在 `pc2Callback()` 附近。

---

# 8. 不新增 `RosDataFormat::HesaiAt128`

虽然配置增加：

```yaml
format: hesai_at128
```

但 AT128 的 ROS message 类型仍然是：

```cpp
sensor_msgs::PointCloud2
```

因此没有必要新增：

```cpp
RosDataFormat::HesaiAt128
```

内部仍可使用：

```cpp
data_format_ = RosDataFormat::PointCloud2;
```

这样可以：

- 减少修改范围；
- 不影响现有 output callback；
- 不引入新的内部数据类型；
- 明确区分“ROS transport format”和“sensor-specific parser”。

当前 `RosDataFormat::PointCloud2` 只在 PointCloud2 constructor/output 相关位置使用。

---

# 9. 修改 3：新增 `format: hesai_at128` 构造分支

文件：

```text
ros_wrapper/src/gici/src/ros_interface/ros_stream.cpp
```

当前存在：

```cpp
else if (data_format == "pointcloud2") {
    data_format_ = RosDataFormat::PointCloud2;

    if (io_type_ == StreamIOType::Input) {
        subscribers_.push_back(
            nh_.subscribe<sensor_msgs::PointCloud2>(
                topic_name_,
                queue_size_,
                boost::bind(
                    &RosStream::pc2Callback,
                    this,
                    _1)));
    }
    ...
}
```



在其附近新增：

```cpp
else if (data_format == "hesai_at128") {
  data_format_ = RosDataFormat::PointCloud2;

  if (io_type_ == StreamIOType::Input) {
    subscribers_.push_back(
        nh_.subscribe<sensor_msgs::PointCloud2>(
            topic_name_,
            queue_size_,
            boost::bind(
                &RosStream::hesaiAt128Callback,
                this,
                _1)));
  } else {
    LOG(ERROR)
        << "Setting Hesai AT128 topic as output is disabled!";
    return;
  }
}
```

## 要求

`hesai_at128` 当前定义为 **input-only adapter**。

不要尝试增加：

```text
AT128 output
AT128 PointCloud2 publisher
```

这些都不属于 G1 范围。

---

# 10. 修改 4：实现 `hesaiAt128Callback()`

建议放在：

```cpp
pc2Callback()
```

附近。

## 10.1 创建内部数据

与现有 PointCloud2 adapter 一致：

```cpp
std::shared_ptr<DataCluster> data_cluster =
    std::make_shared<DataCluster>(
        FormatorType::PointCloud2);
```

不要新增：

```cpp
FormatorType::HesaiAt128
```

因为传感器差异应该在 ROS adapter 层结束。

---

# 11. PointCloud2 → AT128 PCL cloud

创建：

```cpp
pcl::PointCloud<PointXYZIRTAT128>::Ptr cloud_ptr(
    new pcl::PointCloud<PointXYZIRTAT128>);
```

然后：

```cpp
pcl::fromROSMsg(*msg, *cloud_ptr);
```

必须依赖字段名称和 datatype：

```text
x
y
z
intensity
ring
timestamp
```

进行转换。

如果：

```cpp
cloud_ptr->empty()
```

则：

```cpp
LOG(WARNING)
    << "Received an empty Hesai AT128 PointCloud2 message; skipping it.";
return;
```

---

# 12. 时间处理逻辑

首先：

```cpp
const double base_time =
    msg->header.stamp.toSec();
```

定义：

```cpp
double max_relative_time =
    -std::numeric_limits<double>::infinity();

double min_relative_time =
    std::numeric_limits<double>::infinity();
```

因此 `ros_stream.cpp` 如当前未包含 `<limits>`，增加：

```cpp
#include <limits>
```

---

# 13. 点循环

核心逻辑应等价于：

```cpp
Cloud scan;
scan.reserve(cloud_ptr->size());

for (const auto& src_point : cloud_ptr->points) {

  // Timestamp is required for deskew.
  if (!std::isfinite(src_point.timestamp)) {
    continue;
  }

  const double relative_time =
      src_point.timestamp - base_time;

  min_relative_time =
      std::min(min_relative_time, relative_time);

  max_relative_time =
      std::max(max_relative_time, relative_time);

  // Invalid Cartesian point is not usable.
  if (!std::isfinite(src_point.x) ||
      !std::isfinite(src_point.y) ||
      !std::isfinite(src_point.z)) {
    continue;
  }

  Point_lidar point;

  point.x = src_point.x;
  point.y = src_point.y;
  point.z = src_point.z;

  point.intensity =
      src_point.intensity;

  point.curvature =
      static_cast<float>(relative_time);

  point.normal_z =
      static_cast<float>(src_point.ring);

  scan.push_back(point);
}
```

---

# 14. 明确禁止 `y < -1` 裁剪

AT128 callback 中只能进行：

```cpp
std::isfinite(x)
std::isfinite(y)
std::isfinite(z)
std::isfinite(timestamp)
```

等数据有效性检查。

**不得复制现有 `pc2Callback()` 中：**

```cpp
src_point.y < -1
```

这一判断。

即不允许：

```cpp
if (... || src_point.y < -1) {
    continue;
}
```

AT128 输入必须保留左右两侧完整点云。

---

# 15. 不进行距离裁剪

G1 adapter 本身也不要新增加：

```text
min range
max range
blind
mapping range
```

等空间滤波。

这些属于 LiDAR frontend / estimator 参数层，不属于 ROS message adapter。

因此 G1 原则是：

\[
\boxed{
\text{只解决 message schema 和 timestamp semantics}
}
\]

而不解决点云场景过滤。

---

# 16. 时间范围处理

在完成遍历后：

如果：

```cpp
!std::isfinite(max_relative_time)
```

说明该帧没有任何有效 timestamp，应：

```cpp
LOG(WARNING)
    << "Hesai AT128 scan contains no valid point timestamps; skipping it.";
return;
```

如果：

```cpp
scan.empty()
```

也应 warning 并 return。

---

# 17. `timebase`

内部：

```cpp
data_cluster->lidar->timebase =
    base_time;
```

即：

\[
\boxed{
t_{base}=header.stamp
}
\]

这与已经验证的 AT128 数据语义一致：

```text
header.stamp == min(point.timestamp)
```

---

# 18. `timefinal`

必须使用：

```cpp
data_cluster->lidar->timefinal =
    base_time + max_relative_time;
```

等价于：

\[
t_{final}
=
\max_i timestamp_i.
\]

不要使用：

```cpp
cloud_ptr->points.back().timestamp
```

也不要使用：

```cpp
last_point.timestamp
```

原因是 `timefinal` 的语义应该由时间最大值确定，而不应该依赖 PointCloud2 内点排列是否始终严格按时间排序。

GLINS 内部 `LidarMeasurement` 对 `timebase/timefinal` 的定义就是绝对 Unix/ROS timestamp。

---

# 19. `valid_num`

使用：

```cpp
data_cluster->lidar->valid_num =
    data_cluster->lidar->cloud_ptr->size();
```

不要使用原始：

```cpp
cloud_ptr->size()
```

因为 invalid XYZ / timestamp 点可能已经被删除。

---

# 20. `seq`

使用：

```cpp
data_cluster->lidar->seq =
    msg->header.seq;
```

不依赖 PCL header 的二次转换。

---

# 21. 完整 callback 逻辑参考

Codex 实现时整体结构建议如下。

```cpp
void RosStream::hesaiAt128Callback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::shared_ptr<DataCluster> data_cluster =
      std::make_shared<DataCluster>(
          FormatorType::PointCloud2);

  pcl::PointCloud<PointXYZIRTAT128>::Ptr cloud_ptr(
      new pcl::PointCloud<PointXYZIRTAT128>);

  pcl::fromROSMsg(*msg, *cloud_ptr);

  if (cloud_ptr->empty()) {
    LOG(WARNING)
        << "Received an empty Hesai AT128 PointCloud2 message; skipping it.";
    return;
  }

  const double base_time =
      msg->header.stamp.toSec();

  double min_relative_time =
      std::numeric_limits<double>::infinity();

  double max_relative_time =
      -std::numeric_limits<double>::infinity();

  Cloud scan;
  scan.reserve(cloud_ptr->size());

  for (const auto& src_point : cloud_ptr->points) {

    if (!std::isfinite(src_point.timestamp)) {
      continue;
    }

    const double relative_time =
        src_point.timestamp - base_time;

    min_relative_time =
        std::min(min_relative_time, relative_time);

    max_relative_time =
        std::max(max_relative_time, relative_time);

    if (!std::isfinite(src_point.x) ||
        !std::isfinite(src_point.y) ||
        !std::isfinite(src_point.z)) {
      continue;
    }

    Point_lidar point;

    point.x = src_point.x;
    point.y = src_point.y;
    point.z = src_point.z;
    point.intensity = src_point.intensity;

    point.curvature =
        static_cast<float>(relative_time);

    point.normal_z =
        static_cast<float>(src_point.ring);

    scan.push_back(point);
  }

  if (!std::isfinite(max_relative_time)) {
    LOG(WARNING)
        << "Hesai AT128 scan contains no valid point timestamps; skipping it.";
    return;
  }

  if (scan.empty()) {
    LOG(WARNING)
        << "Hesai AT128 scan contains no valid points; skipping it.";
    return;
  }

  data_cluster->lidar->cloud_ptr.reset(
      new Cloud(std::move(scan)));

  data_cluster->lidar->timebase =
      base_time;

  data_cluster->lidar->timefinal =
      base_time + max_relative_time;

  data_cluster->lidar->valid_num =
      data_cluster->lidar->cloud_ptr->size();

  data_cluster->lidar->seq =
      msg->header.seq;

  for (const auto& it_lidar_callback :
       data_callbacks_) {
    it_lidar_callback(tag_, data_cluster);
  }

  for (const auto& pipeline :
       pipeline_ros_to_ros_) {
    pipeline("", data_cluster);
  }
}
```

这是实现逻辑参考，不要求机械复制排版，但数据语义不得改变。

---

# 22. 关于负 relative time

当前 RobNav AT128 数据已经验证：

\[
timestamp_i-header.stamp \ge 0
\]

且：

\[
\max(dt)\approx0.052\ {\rm s}.
\]

因此 G1 **不要增加人为 clip**：

```cpp
relative_time = max(relative_time, 0.0);
```

也不要自动做绝对值。

如果后续运行时发现：

```text
relative_time < 0
```

应该作为输入数据/时间语义异常单独调查，而不是在 adapter 中静默修改数据。

同理，也不要硬编码：

```cpp
relative_time <= 0.1
```

或：

```cpp
relative_time <= 0.2
```

作为正式过滤条件。

---

# 23. 是否需要保存 ring

保持与现有 `pc2Callback()` 一致：

```cpp
point.normal_z =
    src_point.ring;
```

当前这一操作的主要意义是维持现有 PointCloud2 输入的数据传播行为。

G1 不进一步修改 downstream 对 ring 的使用方式。

---

# 24. 配置接口

修改完成后，AT128 输入应能够通过：

```yaml
- streamer:
    tag: str_ros_lidar
    type: ros
    io: input
    topic_name: /hesai/at128/points
    queue_size: 10
    format: hesai_at128
```

进入新 callback。

原有：

```yaml
format: pointcloud2
```

仍进入：

```cpp
pc2Callback()
```

原有：

```yaml
format: livox
```

仍进入：

```cpp
livoxCallback()
```

---

# 25. 后向兼容性要求

Codex 修改后必须检查：

```bash
git diff
```

确认以下内容没有被删除或改变语义：

```text
PointXYZIRT
livoxCallback()
pc2Callback()
format == "livox"
format == "pointcloud2"
```

尤其不得为了 AT128 删除现有：

```cpp
src_point.y < -1
```

这个条件。

这里的要求是：

> **保留原通用 PointCloud2 adapter 原样；只是在 AT128 adapter 中不使用这个条件。**

---

# 26. 编译验证

修改完成后：

```bash
cd ~/glins_ws/ros_wrapper

source /opt/ros/noetic/setup.zsh

rm -rf build devel

catkin_make -DCMAKE_BUILD_TYPE=Release
```

G1 的最低验收标准是：

```text
gici_ros 编译成功
gici_ros_main 编译成功
无 PointXYZIRTAT128 PCL registration error
无 timestamp datatype compilation error
```

---

# 27. G1 运行时验证指标

G2 配置生成以后，首先使用 `building02` 验证。

已知正确参考值为：

```text
topic:
/hesai/at128/points

points/frame:
约 160000

timebase:
≈ msg.header.stamp

min relative point time:
≈ 0 s

max relative point time:
≈ 0.0519 ~ 0.0521 s

timefinal - timebase:
≈ 0.052 s

LiDAR period:
≈ 0.1 s
```

必须明确区分：

\[
\boxed{
scan\ point\ time\ span\approx52ms
}
\]

和：

\[
\boxed{
scan\ message\ period\approx100ms
}
\]

两者不是同一个量。

---

# 28. 建议增加的临时诊断

为了 G3 首次运行验证，可以临时使用 `VLOG(1)` 或一次性 diagnostic 输出：

```text
AT128 seq
raw point count
valid point count
timebase
min relative time
max relative time
timefinal
```

预期类似：

```text
AT128 seq=...
raw_points=160000
valid_points≈160000
timebase=1745893277.713022
min_dt=0.000000
max_dt=0.052036
timefinal=1745893277.765058
```

不要每帧使用普通：

```cpp
LOG(INFO)
```

长期刷屏。

如果加入诊断，建议使用：

```cpp
VLOG(1)
```

或者只打印第一帧。

---

# 29. 不属于 G1 的内容

Codex **本次不要修改**：

```text
T_B_L
body_to_imu_rotation
IMU noise
GNSS lever arm
GNSS system_exclude
DCB path
RTK parameters
ambiguity resolution
dataset-specific YAML
LiDAR frontend parameters
voxel size
blind range
mapping range
```

这些属于 G2 及后续阶段。

G1 只负责：

\[
\boxed{
\text{Hesai AT128 ROS PointCloud2}
\rightarrow
\text{GLINS internal LiDAR measurement}
}
\]

---

# 30. 最终验收条件

G1 可以判定 PASS，当且仅当满足：

```text
[1] 新增 format: hesai_at128
[2] hesai_at128 使用独立 callback
[3] 原 livoxCallback 保持
[4] 原 pc2Callback 保持
[5] AT128 timestamp 使用 double 读取
[6] relative_time = timestamp - header.stamp
[7] relative time 最后才转换为 float curvature
[8] AT128 不使用 y < -1 裁剪
[9] 不增加其他空间裁剪
[10] timebase = header.stamp
[11] timefinal = max(point timestamp)
[12] valid_num = 实际保留点数
[13] 原 downstream LiDAR 流程无需修改
[14] Release 编译成功
```

实现完成以后不要直接进入完整 RRR。

下一阶段顺序仍然是：

```text
G1 编译通过
    ↓
G2 生成 RobNav 配置
    ↓
G3 building02 AT128 接口验证
    ↓
H1 LIO
    ↓
H2 standalone RTK
    ↓
H3 RRR
```