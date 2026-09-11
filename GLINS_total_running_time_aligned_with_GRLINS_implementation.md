# GLINS 总运行时间统计实现方案：严格对齐现有 GRLINS 计时口径

## 1. 目标

GRLINS 的现有运行时间统计保持不变，因此 GLINS 必须主动向 **GRLINS 当前代码的实际计时边界** 对齐。论文主指标统一为：

\[
T_{\text{total}}=\text{Total running time (wall-clock time)}
\]

不再用 `process_cpu_time_s` 作为主比较指标；它只保留作诊断。

---

## 2. 现有 GRLINS 的实际计时边界

GRLINS `imu_process_datapack_bag.cpp` 的顺序是：

```cpp
YAML::Node config = YAML::LoadFile(config_file);

rosbag::Bag rdbag(bag_file, rosbag::bagmode::Read);
rosbag::View view(rdbag, rosbag::TopicQuery(topics));

SensorPackage sensor_package;

MSCKF imu_process(config_file);
```

因此，GRLINS 在 `MSCKF` 构造前已经完成：

```text
ROS/glog 基础初始化
main 中第一次 YAML::LoadFile
bag 路径/topic 读取
rosbag::Bag 打开
rosbag::View 建立
SensorPackage 构造
```

这些不在 Total 中。

`MSCKF` 类把：

```cpp
TicToc t_process_;
```

作为成员，而 `TicToc()` 构造时立即执行：

```cpp
start = std::chrono::system_clock::now();
```

所以 `MSCKF imu_process(config_file);` 开始构造时，总计时已经启动。

这意味着 GRLINS Total 包括：

```text
MSCKF 构造
pm->load(config_file)
算法对象初始化
结果输出对象初始化
VisualizationRviz 创建
direct rosbag message 迭代
message instantiate/deserialization
sensor conversion
IMU/LiDAR/GNSS processing
状态估计
运行期输出
MSCKF 析构函数中 t_process_.toc() 之前的结果保存
```

`MSCKF::~MSCKF()` 中先进行地图/PCD及最终状态等结束阶段处理，后面才调用：

```cpp
t_process_.toc()
```

并最终写：

```text
total_process_time.txt
```

因此总时间一直持续到析构函数后段。

唯一逻辑上无法完整计入的是：

```text
写入 total_process_time.txt 中“Total 数值本身”的最后一次文件写操作
```

因为必须先获得 Total，才能把它写出去。

---

## 3. GLINS 最终采用的同口径定义

GLINS 应采用：

\[
T_{\text{GLINS,total}}
=
t_{\text{before total value is written}}
-
t_{\text{algorithm-side construction begins}}
\]

流程固定为：

```text
[不计时]
CLI 参数解析
ROS/glog 基础初始化
第一次 YAML::LoadFile
四个 bag 打开
formal/eph rosbag::View 建立
result_dir 路径字符串解析
        ↓
============== START TOTAL TIMER ==============
        ↓
NodeOptionHandle 构造
RosNodeHandle 构造
MultiSensorEstimating / RRR / TreeHandler 构造
DataIntegration / RosStream 绑定
输出对象 / Recorder 初始化
DCB 处理
        ↓
ephemeris direct preload
base antenna ECEF prefeed
        ↓
formal merged rosbag::View 直接读取
        ↓
message instantiate / conversion
RosStream
DataIntegration
MultiSensorEstimating
LiDAR frontend
GNSS / IMU processing
RRR backend
optimization / marginalization
运行期 NAV/timing/output
        ↓
formal EOF
        ↓
EOF alignment release
pipeline drain
        ↓
worker 正常 stop/join
        ↓
NAV flush
timing_events.csv flush
timing_summary.csv 生成/flush
其他正式结果输出 flush
        ↓
=============== STOP TOTAL TIMER ===============
        ↓
写 total_running_time_s 自身
写包含 Total 字段的最终 summary
```

---

## 4. 两个必须修正的地方

### 4.1 Total 不能只从 formal bag loop 开始

如果当前实现类似：

```cpp
auto wall_start = Clock::now();

for (const auto& msg : formal_view) {
    ...
}
```

则会漏掉：

```text
NodeOptionHandle
RosNodeHandle
RRR estimator construction
DataIntegration construction
DCB
ephemeris preload
base antenna prefeed
```

而 GRLINS 的计时已经从 `MSCKF` 构造阶段开始。

因此 GLINS 的 `total_start` 应放在：

```text
所有 bag 已经 open、View 已经建立
但 GLINS algorithm-side object 尚未开始构造
```

的位置。

推荐：

```cpp
using TotalClock = std::chrono::system_clock;

const auto total_start = TotalClock::now();

auto node_option_handle =
    std::make_shared<NodeOptionHandle>(yaml_node);

auto node_handle =
    std::make_unique<RosNodeHandle>(
        nh, node_option_handle);
```

### 4.2 NAV/timing/result 的最终 flush 必须在 Total 内

不能：

```cpp
waitPipelineIdle();

auto total_end = TotalClock::now();

recorder.flush();
writeTimingSummary();
```

而应该：

```cpp
waitPipelineIdle();

stopAndJoinWorkers();

recorder.flush();
writeTimingSummary();
flushOtherAlgorithmOutputs();

auto total_end = TotalClock::now();
```

然后才写 Total 本身。

---

## 5. Clock 选择

现有 GRLINS `TicToc` 使用：

```cpp
std::chrono::system_clock
```

因此 GLINS 也使用同一 clock，以尽量保持定义一致：

```cpp
using TotalClock = std::chrono::system_clock;

const auto total_start = TotalClock::now();

...

const auto total_end = TotalClock::now();

const double total_running_time_s =
    std::chrono::duration<double>(
        total_end - total_start).count();
```

正式 benchmark 时避免手动修改系统时间或发生明显 clock jump。

---

## 6. 计时开始前允许执行的操作

`gici_robnav_bag_main.cpp` 可以先执行：

```cpp
google::InitGoogleLogging(...);
ros::init(...);

Arguments args = parseArguments(...);

YAML::Node yaml_node =
    YAML::LoadFile(args.config);

rosbag::Bag sensor_bag(...);
rosbag::Bag rover_bag(...);
rosbag::Bag reference_bag(...);
rosbag::Bag ephemeris_bag(...);

rosbag::View formal_view;
formal_view.addQuery(...);

rosbag::View ephemeris_view(...);
```

这些对应 GRLINS 在 `MSCKF` 构造前已经做完的 main-level setup。

只应在这里解析 `result_dir` 路径字符串；如果 GLINS 自己执行 `create_directories()`、打开正式输出文件或初始化 Recorder，建议移到 `total_start` 之后，因为 GRLINS 的结果目录/输出对象创建发生在 `MSCKF` 计时范围内。

---

## 7. ephemeris preload 必须计入

ephemeris bag 的 **打开和 View 构造** 可以在 timer 前，但以下必须在 timer 后：

```text
遍历 ephemeris_view
instantiate<GnssEphemerides>()
feedGnssEphemerides()
RosStream conversion
GnssDataIntegration::updateEphemerides()
TGD/code-bias 更新
```

原因是这些已经属于 GLINS 算法侧 GNSS 输入处理，而不是单纯文件打开。

推荐：

```cpp
const auto total_start = TotalClock::now();

constructGlinsObjects();

for (const auto& msg : ephemeris_view) {
    feedEphemeris(msg);
}
```

---

## 8. base antenna ECEF prefeed 计入

下面链路：

```text
构造 GnssAntennaPosition
→ reference_stream->feedGnssAntennaPosition(...)
→ RosStream
→ GnssDataIntegration
→ gnss_local_->antenna
```

属于算法输入初始化，因此必须在 Total 内。

---

## 9. DCB 处理计入

当前 DCB 保持：

```text
file streamer
→ formator
→ GnssDataIntegration
```

只要 `total_start` 位于 `NodeOptionHandle/RosNodeHandle` 构造之前，DCB 正常初始化/加载就自然属于 Total。

---

## 10. formal direct-bag 阶段全部计入

以下全部属于 Total，不扣除任何模块时间：

```text
formal_view 迭代
MessageInstance 获取
instantiate<T>()
PointCloud2 deserialization
AT128 点转换和 sampling
IMU conversion
rover/reference GNSS conversion
DataCluster
DataIntegration
satposs
measurement addin
LiDAR frontend
input alignment
RRR initialization
factor construction
Ceres optimization
post-processing
marginalization
正常 NAV/timing/output
```

---

## 11. EOF 之后继续计时

GLINS 是异步 pipeline，formal EOF 时可能仍有：

```text
measurement_addin_buffer_
measurement_align_buffer_
lidar_frontend_measurements_
measurements_

measurement thread busy
LiDAR frontend busy
backend busy
```

因此：

```text
formal_view EOF != GLINS processing finished
```

EOF 后必须继续 Total：

```text
notifyInputFinished()
→ EOF alignment release
→ wait pipelineIdle()
→ stop/join
→ final output flush
→ total_end
```

---

## 12. `pipelineIdle()` 的使用

继续使用已经实现的显式 queue/busy 状态，不再根据 CPU idle 猜测。

至少确认：

```text
measurement_addin_buffer_.empty()
measurement_align_buffer_.empty()
lidar_frontend_measurements_.empty()
measurements_.empty()

measurement thread not busy
LiDAR frontend not busy
backend not busy
```

若 solution/output 还有独立 pending queue，也应纳入 idle 判据。

---

## 13. 稳定 idle 窗口不能人为放大 Total

若当前为防 race 使用：

```text
pipelineIdle 连续 100 ms 才确认 drain
```

这 100 ms 是 benchmark 自己引入的确认等待，不是算法工作。

推荐正式实现把稳定确认缩短为：

```text
poll interval: 1~5 ms
continuous idle: 2 次
```

或记录第一次 idle 的 candidate 时刻用于诊断。

但注意：**真实的 stop/join 和输出 flush 仍然必须计入 Total**。

---

## 14. worker stop/join 计入

GLINS 有：

```text
EstimatingBase thread
measurement thread
LiDAR frontend thread
backend thread
```

这些线程的正常停止和 `join()` 应位于 `total_end` 前。

正确顺序：

```text
pipeline drained
→ request normal shutdown
→ worker join
→ output finalize
→ total_end
```

---

## 15. NAV / timing / result flush 计入

必须在 `total_end` 之前完成：

```text
glins.nav
timing_events.csv
timing_summary.csv
其他正式 estimator output
正式配置启用时的地图/点云输出
```

如果 `ExperimentRecorder` 是 memory-buffered，尤其要：

```cpp
ExperimentRecorder::instance().flush();
```

之后再取 `total_end`。

否则 GLINS 会把运行期间积累的输出 I/O 推迟到 Total 之后，和 GRLINS 当前统计方式不一致。

---

## 16. Recorder 必须能够显式 finalize

如果 Recorder 只依赖 singleton destructor 在进程退出时落盘，则 main 内无法在其析构后再取得 Total。

应确保已有或增加：

```cpp
ExperimentRecorder::instance().flush();
```

或：

```cpp
ExperimentRecorder::instance().finalize();
```

要求：

```text
1. total_end 前可以主动调用；
2. flush/finalize 是幂等的；
3. destructor 再执行时不重复写数据。
```

如果当前已经存在显式 graceful flush，直接复用，不新增第二套机制。

---

## 17. 写 Total 数值本身是唯一例外

最终例如：

```text
run_summary.csv
total_process_time.txt
```

如果其中要记录：

```text
total_running_time_s
```

必须先得到 `total_end`，因此这一次“写 Total 自身”的 I/O 不可能被自己的值完整计入。

正确做法：

```cpp
// All normal algorithm outputs are already finalized.
recorder.flush();
writeTimingSummary();
flushOtherOutputs();

const auto total_end = TotalClock::now();

const double total_running_time_s =
    std::chrono::duration<double>(
        total_end - total_start).count();

// Necessarily outside the measured interval.
writeRunSummary(total_running_time_s);
```

这与 GRLINS 最后调用 `t_process_.toc()` 后再把数值写入 `total_process_time.txt` 的行为一致。

---

## 18. 推荐的 main 结构

```cpp
int main(int argc, char** argv)
{
    // ============================================================
    // A. Main-level setup: NOT TIMED
    // ============================================================
    initLogging();
    ros::init(...);

    Arguments args = parseArguments(argc, argv);

    YAML::Node yaml_node =
        YAML::LoadFile(args.config);

    rosbag::Bag sensor_bag(...);
    rosbag::Bag rover_bag(...);
    rosbag::Bag reference_bag(...);
    rosbag::Bag ephemeris_bag(...);

    rosbag::View formal_view;
    formal_view.addQuery(...);

    rosbag::View ephemeris_view(...);

    // ============================================================
    // B. Equivalent to GRLINS MSCKF construction: START
    // ============================================================
    using TotalClock = std::chrono::system_clock;
    const auto total_start = TotalClock::now();

    auto node_option_handle =
        std::make_shared<NodeOptionHandle>(yaml_node);

    auto node_handle =
        std::make_unique<RosNodeHandle>(
            nh, node_option_handle);

    SpinControl::run();

    auto imu_stream = node_handle->getRosStream(...);
    auto lidar_stream = node_handle->getRosStream(...);
    auto rover_stream = node_handle->getRosStream(...);
    auto reference_stream = node_handle->getRosStream(...);

    // ============================================================
    // C. Algorithm-side static GNSS preparation: TIMED
    // ============================================================
    for (const auto& msg : ephemeris_view) {
        feedEphemeris(msg);
    }

    feedReferenceAntennaPosition(...);

    // ============================================================
    // D. Formal direct input: TIMED
    // ============================================================
    for (const auto& msg : formal_view) {
        feedFormalMessage(msg);
    }

    // ============================================================
    // E. EOF and processing drain: TIMED
    // ============================================================
    estimating->notifyInputFinished();
    waitUntilPipelineIdle();

    // ============================================================
    // F. Normal thread shutdown: TIMED
    // ============================================================
    requestNormalShutdown();
    joinAllWorkerThreads();

    // If destruction performs the join:
    node_handle.reset();

    // ============================================================
    // G. Normal result finalize: TIMED
    // ============================================================
    ExperimentRecorder::instance().flush();

    writeTimingSummary();
    flushOtherAlgorithmOutputs();

    // ============================================================
    // H. Equivalent to GRLINS t_process_.toc(): END
    // ============================================================
    const auto total_end = TotalClock::now();

    const double total_running_time_s =
        std::chrono::duration<double>(
            total_end - total_start).count();

    // ============================================================
    // I. Write Total itself: NOT INCLUDED BY NECESSITY
    // ============================================================
    writeFinalRunSummary(total_running_time_s);

    std::cout << "Total running time: "
              << total_running_time_s
              << " s" << std::endl;

    return 0;
}
```

实际实现时要根据本地对象生命周期确认谁负责 `stop/join/flush`，不要重复 stop 或重复 flush。

---

## 19. `NodeHandle` 与 Recorder 的实际析构顺序必须检查

正式实现前确认：

```text
RosNodeHandle::~RosNodeHandle()
NodeHandle::~NodeHandle()
MultiSensorEstimating::~MultiSensorEstimating()
ExperimentRecorder::flush/finalize
```

最终必须满足：

```text
所有可能继续写 Recorder 的 worker thread 已停止并 join
        ↓
Recorder flush
        ↓
timing summary / normal output finalize
        ↓
total_end
```

不能先 flush，再让 backend 继续写 timing event。

---

## 20. 建议输出字段

direct runner 最终保留：

```text
sequence_duration_s
total_running_time_s       # 论文主指标
process_cpu_time_s         # 诊断

imu_count
lidar_count
rover_obs_count
reference_obs_count
ephemeris_count

lidar_frontend_count
lidar_backend_add_count

peak_measurement_addin
peak_lidar_frontend
peak_backend

pipeline_drained
run_valid
```

`detail_module_sum_s` 可以继续保留，但只用于耗时组成分析。因为 GLINS frontend/backend 并行，模块 wall duration 相加不能替代端到端 Total。

---

## 21. GLINS 与 GRLINS 的最终对应关系

| GRLINS 当前行为 | GLINS 对应实现 |
|---|---|
| main 级 YAML parse 在 timer 前 | top-level YAML parse 在 timer 前 |
| bag open 在 timer 前 | 四个 bag open 在 timer 前 |
| `rosbag::View` 创建在 timer 前 | formal/eph View 创建在 timer 前 |
| `MSCKF` 构造触发计时 | GLINS algorithm-side object 构造前启动 timer |
| `pm->load()` 在 timer 内 | `NodeOptionHandle` 在 timer 内 |
| 算法对象初始化在 timer 内 | RosNodeHandle/RRR/DataIntegration 在 timer 内 |
| direct bag read 在 timer 内 | merged direct bag read 在 timer 内 |
| message conversion 在 timer 内 | RosStream conversion 在 timer 内 |
| 正常算法处理在 timer 内 | frontend/backend/FGO 在 timer 内 |
| 正常结果输出在 timer 内 | NAV/timing/result output 在 timer 内 |
| 析构前段处理在 timer 内 | EOF drain + stop/join + flush 在 timer 内 |
| `t_process_.toc()` 得到 Total | `total_end` 得到 Total |
| Total 数值自身最后写盘不完整计入 | final summary 的 Total 字段写盘不计入 |

---

## 22. 明确排除项

GLINS Total 不包括：

```text
进程启动本身
glog / ros::init
CLI 参数解析
main 级第一次 YAML::LoadFile
bag/topic/path 参数解析
rosbag::Bag open
rosbag::View construction / addQuery
bag object 最终 close/destructor
写 total_running_time_s 数值本身
```

---

## 23. 明确包含项

GLINS Total 包括：

```text
NodeOptionHandle
RosNodeHandle
RRR / TreeHandler / DataIntegration construction
算法侧配置和初始化
结果输出对象初始化
DCB 处理
ephemeris preload 的遍历、转换、注入
base antenna ECEF prefeed
formal direct rosbag iteration
message instantiate/deserialization
RosStream sensor conversion
GNSS satposs / DataIntegration
IMU processing
LiDAR frontend
RRR initialization
factor construction
Ceres optimization
post-processing
marginalization
运行期 NAV/timing/output
EOF alignment flush
pipeline drain
worker shutdown/join
最终 NAV/timing/result flush
timing_summary.csv 生成
```

---

## 24. 验证顺序

先不要直接做四组正式结果。建议：

```text
第一步：building02 前 100 s
```

输出：

```text
total_running_time_s
process_cpu_time_s
formal_input_loop_time_s    # 仅诊断，可选
drain_time_s                # 仅诊断，可选
finalize_flush_time_s       # 仅诊断，可选
```

并确认：

```text
run_valid = true
无 NaN
无 IMU coverage error
NAV 正常
消息 count 正确
LiDAR frontend/backend count 合理
pipeline_drained = true
```

然后比较同一 100 s：

```text
direct-bag NAV
vs
原 1× ROS baseline NAV
```

轨迹一致后，再跑完整 `street00`，取得第一组正式可与 GRLINS 比较的 `total_running_time_s`。

---

## 25. 正式实验

direct-bag correctness 通过后，每个序列建议至少运行 3 次：

```text
building02
street00
street01
street02
```

固定：

```text
同一机器
同一 RelWithDebInfo
同一 Ceres num_threads
同一数据区间
同一配置
同一 AT128 sampling
相近系统负载
```

论文报告 `total_running_time_s` 的均值；需要时附标准差。不要挑最快的一次。

---

## 26. 论文表述建议

建议使用：

> The total running time was measured using wall-clock time. For both methods, the rosbag data were read directly without real-time playback. The timing covered estimator construction and initialization, input processing, state estimation, pending-data processing, normal thread shutdown, and normal result output.

如果需要强调输出 I/O：

> The reported total running time also includes the normal result-output operations performed by each implementation.

不要称为：

```text
pure computation time
```

因为现有 GRLINS Total 本身包含初始化和正常输出 I/O。

更合适的名称是：

```text
Total running time
```

或：

```text
End-to-end wall-clock running time
```

---

## 27. 最终实施要求

本阶段只调整 `gici_robnav_bag_main.cpp` 的计时边界以及必要的显式 finalize，不重新修改 direct input 架构。

必须满足：

```text
A. 所有 bag open / View construction 完成
B. total_start = system_clock::now()
C. NodeOptionHandle / RosNodeHandle 构造
D. DCB / eph / antenna 算法侧准备
E. formal direct read
F. EOF drain
G. worker stop/join
H. NAV/timing/result flush
I. total_end = system_clock::now()
J. 最后写 total_running_time_s 自身
```

最终论文比较固定为：

\[
oxed{
\text{Total running time}
=
\text{end-to-end wall-clock elapsed time under the existing GRLINS timing boundary}
}
\]

即：**不修改 GRLINS，而让 GLINS 的计时尽可能复制 GRLINS 当前代码的实际标准。**
