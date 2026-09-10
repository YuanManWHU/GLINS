# GLINS 正式实验自动化实现方案

## 1. 目标

本阶段在已经完成并验证通过的：

```text
Recorder
NAV
Timing instrumentation
```

基础上，只修改：

```text
src/gici/scripts/robnav/run_robnav_rrr.py
```

实现一次完整 GLINS RobNav 实验的自动化运行与结果整理。

本阶段**不再修改**：

- `experiment_recorder.*`
- `multisensor_estimating.cpp`
- `rtk_imu_lidar_rrr_estimator.cpp`
- YAML estimator 参数
- Ceres 线程设置
- ROS visualization output
- GLINS 原有线程结构
- LiDAR/GNSS/IMU 数据处理逻辑

正式运行时保持现有所有输出逻辑，只是不启动 RViz。

---

## 2. 最终目标

运行：

```bash
python3 src/gici/scripts/robnav/run_robnav_rrr.py building02
```

自动完成：

```text
解析 dataset / bag / GNSS / DCB / 播放时间
        ↓
创建本次 run 目录
        ↓
设置 GLINS_RESULT_DIR
        ↓
启动 GLINS
        ↓
定位真正的 gici_ros_main PID
        ↓
预载 ephemeris
        ↓
预载 antenna_position
        ↓
记录 process CPU start
        ↓
正式 rosbag playback
        ↓
固定等待 1 s callback 窗口
        ↓
记录 process CPU end
        ↓
SIGINT 正常关闭 GLINS
        ↓
Recorder flush
        ↓
得到：
    glins.nav
    timing_events.csv
        ↓
脚本生成：
    timing_summary.csv
    run_summary.csv
    estimator_runtime.yaml
    runtime_config.yaml
```

---

# Part A. 输出目录管理

## 3. 结果根目录

建议固定：

```text
/home/slam/glins_ws/results/robnav_rrr
```

脚本中：

```python
RESULT_ROOT = Path("/home/slam/glins_ws/results/robnav_rrr")
```

也可以相对于 workspace 推导：

```python
WORKSPACE_ROOT = Path("/home/slam/glins_ws")
RESULT_ROOT = WORKSPACE_ROOT / "results" / "robnav_rrr"
```

优先使用后者，减少硬编码重复。

---

## 4. 每次运行目录

运行开始后创建：

```python
run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

run_dir = RESULT_ROOT / dataset / run_id
run_dir.mkdir(parents=True, exist_ok=False)
```

目录结构：

```text
results/robnav_rrr/
└── building02/
    └── 20260910_213000/
        ├── glins.nav
        ├── timing_events.csv
        ├── timing_summary.csv
        ├── run_summary.csv
        ├── estimator_runtime.yaml
        └── runtime_config.yaml
```

每次 run 使用新目录，不覆盖旧实验结果。

---

## 5. `GLINS_RESULT_DIR`

启动 GLINS 时给其子进程注入：

```python
glins_env = os.environ.copy()
glins_env["GLINS_RESULT_DIR"] = str(run_dir)
```

然后：

```python
glins_proc = subprocess.Popen(
    glins_launch_cmd,
    env=glins_env,
    ...
)
```

不得全局修改当前 shell 环境。

这样：

```text
当前脚本启动的 GLINS → Recorder enabled
其他终端/其他 GLINS   → 不受影响
```

---

# Part B. GLINS 进程定位

## 6. 为什么不能直接使用 `roslaunch.pid`

当前脚本启动的是：

```text
roslaunch
```

而真正运行估计算法的是：

```text
devel/lib/gici_ros/gici_ros_main
```

因此：

```python
glins_proc.pid
```

通常只是：

```text
roslaunch PID
```

不能用于统计算法 CPU time。

必须定位真正的：

```text
gici_ros_main PID
```

---

## 7. 启动前记录已有进程

实现：

```python
def find_gici_ros_main_pids():
    ...
```

通过扫描：

```text
/proc/<pid>/cmdline
```

匹配：

```text
gici_ros_main
```

启动 GLINS 前：

```python
existing_gici_pids = find_gici_ros_main_pids()
```

启动后循环等待：

```python
deadline = time.monotonic() + 10.0

while time.monotonic() < deadline:
    current = find_gici_ros_main_pids()
    new_pids = current - existing_gici_pids

    if len(new_pids) == 1:
        gici_pid = next(iter(new_pids))
        break

    if len(new_pids) > 1:
        raise RuntimeError("Multiple new gici_ros_main processes found")

    time.sleep(0.1)
```

超时：

```python
raise RuntimeError("Unable to locate gici_ros_main PID")
```

---

## 8. PID 进一步校验

找到 PID 后读取：

```text
/proc/<pid>/cmdline
```

至少打印：

```text
GLINS PID
command line
```

若命令行中可以看到当前配置文件路径，则进一步确认它属于本次 run。

建议日志：

```text
GLINS process
  roslaunch pid : 12345
  gici pid      : 12378
  cmdline       : .../gici_ros_main ...
```

避免正式实验中误统计其他 GLINS 进程。

---

# Part C. Process CPU Time

## 9. 总运行时间定义

正式 GLINS 总运行时间定义为：

\[
T_{\text{GLINS}}
=
T_{\text{CPU,end}}
-
T_{\text{CPU,start}}
\]

这里的 CPU time 是：

```text
gici_ros_main 整个进程所有线程累计 CPU 时间
```

包括：

- GLINS frontend thread
- backend thread
- measurement thread
- Ceres 工作线程
- 当前已有 ROS output / visualization preparation
- 其他在 `gici_ros_main` 内实际执行的计算

不包括：

- rosbag 的 CPU
- RViz
- shell 等待
- `sleep()`
- rosbag 按真实时间播放造成的空闲等待

因此适合作为多线程 GLINS 的总体计算量指标。

---

## 10. `/proc/<pid>/stat`

Linux：

```text
/proc/<pid>/stat
```

字段：

```text
14 : utime
15 : stime
```

以 clock tick 为单位。

实现：

```python
CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])

def read_process_cpu_time(pid: int) -> float:
    path = Path(f"/proc/{pid}/stat")

    with path.open("r") as f:
        fields = f.read().split()

    utime_ticks = int(fields[13])
    stime_ticks = int(fields[14])

    return (utime_ticks + stime_ticks) / CLK_TCK
```

返回单位：

```text
seconds
```

---

## 11. CPU start 的准确位置

现有运行流程保持：

```text
启动 GLINS
    ↓
ephemeris preload
    ↓
antenna_position preload
    ↓
等待输入链准备
    ↓
正式 sensor/GNSS playback
```

CPU start 必须放在：

```text
preload 全部完成
正式 playback 即将开始
```

之间：

```python
cpu_start = read_process_cpu_time(gici_pid)

play_proc = subprocess.Popen(formal_play_cmd)
```

这样：

```text
ephemeris preload
antenna preload
```

不计入正式运行时间。

而：

```text
GNSS/IMU initialization
LiDAR initialization
正常 RRR estimation
```

都计入。

---

# Part D. 正式播放与固定收尾窗口

## 12. 正式 playback

继续复用已经验证通过的实际 rosbag 记录时间、全局播放起点、sensor duration 与 topic filtering。若 rosbag 返回码非零，本次 run 失败。

---

## 13. 为什么不再使用 CPU drain

GLINS 的多个工作线程会高频 polling；即使没有 estimator workload，process CPU 也不会接近零。因此，基于绝对 CPU delta 的 idle 判据会等待至 timeout，并把这段 polling CPU 混入 `process_cpu_time_s`。

同时，`lidar_frontend_total - backend_add/lidar` 不是 backend queue 长度：初始化阶段的 LiDAR measurement 可能在调用 `estimator_->addMeasurement()` 前被正常丢弃。该差值不能用于 drain 或有效性判定。

---

## 14. 固定 post-play callback 窗口

正式播放结束后采用固定的短窗口：

```python
POST_PLAY_WAIT_S = 1.0

play_proc.wait()
time.sleep(POST_PLAY_WAIT_S)
cpu_end = read_process_cpu_time(gici_pid)
```

这 1 s 只为 ROS callback 和已有队列提供有限的收尾机会；它不是“检测到 backend 已空闲”的结论。固定上界避免原先 30 s timeout 污染 total，且所有实验使用相同口径。

---

## 15. LiDAR frontend/backend 差值

脚本记录：

```text
lidar_frontend_minus_backend_add
```

它只是诊断信息，不参与 `run_valid`，也不用于判断是否需要继续等待。

---

# Part E. CPU end 与正常退出

## 16. CPU end

固定 1 s 窗口结束后立即读取：

```python
cpu_end = read_process_cpu_time(gici_pid)
process_cpu_time_s = cpu_end - cpu_start
```

CPU end 必须在 SIGINT 前读取，避免 shutdown、Recorder flush 与 C++ destruction 的额外 CPU 时间混入正式算法 runtime。

---

## 17. 正常关闭 GLINS

顺序为：

```text
CPU end
    ↓
SIGINT
    ↓
等待 Recorder flush
```

`run_valid` 仅接受 `shutdown_mode == "SIGINT"`；使用 SIGTERM、SIGKILL 或 GLINS 提前退出的 run 均无效。

---

## 18. 等 Recorder 文件出现

GLINS 退出后检查 `glins.nav` 与 `timing_events.csv` 均存在、非空且可解析。

---

# Part F. Timing Summary

## 19. 不使用 pandas 作为运行脚本依赖

虽然当前机器已经安装 pandas，但正式 runner 没必要依赖它。

使用 Python 标准库：

```python
csv
statistics
math
```

这样脚本可移植性更好。

---

## 20. 读取 `timing_events.csv`

定义：

```python
@dataclass
class TimingEvent:
    event_id: int
    scope: str
    module: str
    trigger: str
    data_timestamp: float
    gps_sow: float
    wall_start_ms: float
    duration_ms: float
    thread_id: str
```

读取：

```python
def read_timing_events(path):
    events = []

    with path.open(newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            events.append(
                TimingEvent(
                    event_id=int(row["event_id"]),
                    scope=row["scope"],
                    module=row["module"],
                    trigger=row["trigger"],
                    data_timestamp=float(row["data_timestamp"]),
                    gps_sow=float(row["gps_sow"]),
                    wall_start_ms=float(row["wall_start_ms"]),
                    duration_ms=float(row["duration_ms"]),
                    thread_id=row["thread_id"],
                )
            )

    return events
```

读取后检查：

```text
duration finite
duration >= 0
```

若存在非法 event：

```text
run_valid = false
```

---

## 21. Group key

按：

```python
(scope, module, trigger)
```

分组。

例如：

```text
detail / optimization / gnss
detail / optimization / lidar
detail / marginalization / gnss
detail / marginalization / lidar
```

不提前合并。

---

## 22. 汇总指标

每组计算：

```text
count
total_ms
mean_ms
std_ms
median_ms
p95_ms
max_ms
```

输出：

```text
timing_summary.csv
```

格式：

```text
scope,module,trigger,count,total_ms,mean_ms,std_ms,median_ms,p95_ms,max_ms
```

---

## 23. 标准差

建议 sample standard deviation：

```python
statistics.stdev(values)
```

当：

```text
count == 1
```

则：

```text
std_ms = 0.0
```

---

## 24. P95

自己实现线性插值 percentile：

```python
def percentile(values, p):
    if not values:
        return math.nan

    x = sorted(values)

    if len(x) == 1:
        return x[0]

    pos = (len(x) - 1) * p
    lower = math.floor(pos)
    upper = math.ceil(pos)

    if lower == upper:
        return x[lower]

    weight = pos - lower
    return x[lower] * (1.0 - weight) + x[upper] * weight
```

调用：

```python
p95_ms = percentile(values, 0.95)
```

这样无需 NumPy/pandas。

---

# Part G. Detail Module Sum

## 25. Detail 模块集合

定义固定集合：

```python
DETAIL_MODULES = {
    "initialization",
    "lidar_initialization",
    "lidar_preprocess",
    "gnss_aux_rtk",
    "gnss_factor",
    "lidar_factor",
    "optimization",
    "gnss_post",
    "lidar_post",
    "marginalization",
}
```

计算：

```python
detail_module_sum_ms = sum(
    e.duration_ms
    for e in events
    if e.scope == "detail" and e.module in DETAIL_MODULES
)
```

再：

```python
detail_module_sum_s = detail_module_sum_ms / 1000.0
```

它只作为：

```text
内部计算分解的辅助总和
```

不直接替代正式：

```text
process_cpu_time_s
```

---

## 26. 不在这里做“论文模块合并”

本阶段禁止提前构造：

```text
LiDAR processing
GNSS processing
FGO processing
```

因为哪些模块最终放论文还未决定。

正式完整 building02 结果出来后，再根据：

```text
total time
mean time
占 process CPU 比例
理论模块意义
```

讨论合并方式。

---

# Part H. LiDAR frontend/backend 诊断

## 27. LiDAR frontend/backend count

从 timing events 统计 `lidar_frontend_total/lidar` 与 `backend_add/lidar`，并记录：

```text
lidar_frontend_minus_backend_add
=
lidar_frontend_count - lidar_backend_add_count
```

该值不是 backend queue pending 数。measurement 可能在初始化阶段、调用 `estimator_->addMeasurement()` 前被正常过滤，因此该差值只用于后续源码与 event 分布分析。

---

## 28. 诊断使用原则

`lidar_frontend_minus_backend_add` 不参与 post-play 等待，也不参与 `run_valid`。不得将它解释为“还需 drain 的 LiDAR 帧数”。

---

# Part I. NAV Summary

## 29. NAV 只做轻量检查

用户当前主要关心位置，因此 runner 不做复杂姿态验证。

读取：

```text
glins.nav
```

统计：

```text
nav_count
nav_first_sow
nav_last_sow
latitude_min
latitude_max
longitude_min
longitude_max
height_min
height_max
```

并检查：

```text
11 columns
first column == 0
finite
SOW strictly increasing
```

不在 runner 中分析：

```text
velocity accuracy
attitude accuracy
```

---

# Part J. `run_summary.csv`

## 30. 字段设计

一行一个 run：

```text
dataset
run_id

bag_sensor_start
bag_sensor_end
play_sensor_start
play_sensor_end
sequence_duration_s

gici_pid

process_cpu_start_s
process_cpu_end_s
process_cpu_time_s

detail_module_sum_s
post_play_wait_s

nav_count
nav_first_sow
nav_last_sow

lidar_frontend_count
lidar_backend_add_count
lidar_frontend_minus_backend_add

rosbag_exit_code
glins_exit_code
shutdown_mode

run_valid
```

---

## 31. `process_cpu_time_s`

正式论文总运行时间候选：

```text
process_cpu_time_s
```

定义：

\[
T_{\text{total}}
=
T_{\text{CPU,end}}
-
T_{\text{CPU,start}}.
\]

不要保存 rosbag wall-clock 为“total runtime”。

如果需要可以另存：

```text
playback_wall_s
```

但只能作为运行诊断，不用于论文效率比较。

---

# Part K. 配置快照与运行 manifest

## 32. 两类 YAML 的职责

每个 run 目录必须保存两类不同用途的 YAML，不能复用同一个文件名：

```text
estimator_runtime.yaml
    → 本次实际传给 gici_ros_main 的估计器配置快照。
    → 从基础 YAML 复制后，仅注入当前 DCB 路径。

runtime_config.yaml
    → 实验 manifest，不是估计器配置。
    → 记录本次数据、代码、时间范围、PID 与运行参数。
```

脚本应将生成的估计器 YAML 写入：

```text
<run_dir>/estimator_runtime.yaml
```

并将该路径传给 `gici_ros_main`。原始基础 YAML 不被修改。

`runtime_config.yaml` 用于回答：

> 这次结果到底是哪份代码、哪组数据、哪个配置、哪个时间范围跑出来的？

---

## 33. 建议字段

```yaml
run:
  dataset: building02
  run_id: 20260910_213000
  result_dir: /home/slam/glins_ws/results/robnav_rrr/building02/20260910_213000

glins:
  workspace: /home/slam/glins_ws
  repository: Garfield-cn/GLINS
  git_commit: ...
  git_dirty: not_checked
  build_type: RelWithDebInfo
  config_file: .../estimator_runtime.yaml
  base_config_file: ...

data:
  sensor_bag: ...
  rover_bag: ...
  reference_bag: ...
  ephemeris_bag: ...
  dcb_file: ...

timing:
  bag_sensor_start: ...
  bag_sensor_end: ...
  play_sensor_start: ...
  play_sensor_end: ...
  sequence_duration_s: ...
  play_global_start: ...
  play_start_offset_s: ...

runtime:
  gici_pid: ...
  rviz_started: false
```

---

## 34. Git commit

运行时：

```python
subprocess.check_output(
    ["git", "show", "-s", "--format=%H", "HEAD"],
    cwd=WORKSPACE_ROOT,
    text=True
).strip()
```

---

## 35. Git dirty

当前 runner 仅记录可通过 `git show` 获取的 commit；受工作区 Git 操作权限约束，manifest 固定写入：

```yaml
git_dirty: not_checked
```

因此正式论文实验前，应在固定 commit 的干净工作区中运行，或另行授权加入 dirty-state 记录。

---

# Part L. Run Valid 判据

## 36. 基础有效条件

```python
run_valid = all([
    not interrupted,
    rosbag_exit_code == 0,
    process_cpu_time_s is not None and process_cpu_time_s > 0.0,
    nav_exists and nav_count > 0 and nav_is_valid,
    timing_exists and timing_event_count > 0,
    timing_all_finite and timing_all_nonnegative,
    shutdown_mode == "SIGINT",
])
```

`post_play_wait_s` 与 `lidar_frontend_minus_backend_add` 都不参与 `run_valid`。

---

# Part M. 异常和清理

## 38. 主流程必须使用 `try/finally`

结构：

```python
glins_proc = None
play_proc = None

try:
    ...
finally:
    if play_proc and play_proc.poll() is None:
        stop rosbag

    if glins_proc and glins_proc.poll() is None:
        stop GLINS gracefully
```

避免：

```text
脚本异常
→ rosbag继续跑
→ rosrun 包装进程残留
→ gici_ros_main残留
```

污染下一次实验。

---

## 39. Ctrl+C

捕获：

```python
except KeyboardInterrupt:
```

打印：

```text
Interrupted by user
```

然后进入 `finally` 正常关闭。

这种 run：

```text
run_valid = false
```

---

## 40. GLINS 提前退出

正式 playback 过程中定期或结束后检查：

```python
if glins_proc.poll() is not None:
```

若 GLINS 在 rosbag 结束前退出：

```text
run_valid = false
```

并记录：

```text
failure_reason = "glins_exited_early"
```

---

# Part N. 日志输出

## 41. 运行开始

建议：

```text
================================================================================
RobNav GLINS RRR Run
================================================================================
dataset              : building02
run id               : 20260910_213000
result dir           : ...
sensor bag           : ...
rover bag            : ...
reference bag        : ...
ephemeris bag        : ...
DCB                   : ...
bag sensor start     : ...
bag sensor end       : ...
play sensor start    : ...
play sensor end      : ...
duration             : ...
```

---

## 42. PID

```text
================================================================================
GLINS Process
================================================================================
rosrun pid            : ...
gici_ros_main pid     : ...
```

---

## 43. 正式计时

```text
================================================================================
Formal Playback
================================================================================
process CPU start     : ...
```

播放完成：

```text
rosbag playback       : finished
post-play callback wait: 1.0 s
```

---

## 44. 结束

```text
================================================================================
Run Summary
================================================================================
sequence duration     : 1871.455 s
process CPU time      : xxx.xxx s
detail module sum     : xxx.xxx s
post-play wait        : 1.0 s

NAV records           : xxxx
LiDAR frontend        : xxxxx
LiDAR backend add     : xxxxx
LiDAR frontend - backend add : x

run valid             : YES
result dir            : ...
```

---

# Part O. 实现顺序

## 45. Phase 1：结果目录 + 环境变量

先实现：

```text
run_dir
GLINS_RESULT_DIR
runtime_config基础内容
```

验证 Recorder 自动输出到正确 run 目录。

---

## 46. Phase 2：PID + process CPU

实现：

```text
find gici_ros_main PID
read /proc/<pid>/stat
CPU start/end
```

先不加 post-play 等待。

60 s 测试确认：

```text
process_cpu_time_s > 0
```

---

## 47. Phase 3：固定 post-play 等待

加入：

```text
POST_PLAY_WAIT_S = 1.0
post_play_wait_s
```

确认 CPU end 在固定窗口后、SIGINT 前读取。

---

## 48. Phase 4：summary

最后加入：

```text
timing_summary.csv
run_summary.csv
NAV basic summary
LiDAR frontend/backend count
run_valid
```

避免一开始一次改太多。

---

# Part P. 60 s 自动化验证

## 49. 命令

如果当前 runner 已支持：

```bash
python3 src/gici/scripts/robnav/run_robnav_rrr.py building02 --duration 60
```

则直接使用。

如果脚本当前参数不是这个形式，保持现有 CLI，只增加等价的 60 s 运行方式，不为了实验重新设计参数接口。

---

## 50. 验证重点

必须确认：

```text
[1] 自动创建 run_dir
[2] glins.nav 正确进入 run_dir
[3] timing_events.csv 正确进入 run_dir
[4] 找到唯一 gici_ros_main PID
[5] process_cpu_time_s > 0
[6] rosbag结束后固定等待 1 s，再记录 CPU end
[7] GLINS SIGINT 正常退出
[8] timing_summary.csv 生成
[9] run_summary.csv 生成
[10] estimator_runtime.yaml 生成且作为 GLINS 实际加载配置
[11] runtime_config.yaml（实验 manifest）生成
[12] run_valid = true
```

---

# Part Q. 60 s 通过后的下一步

## 51. 完整 building02

自动化验证通过后，不立即四组批量运行。

先完整：

```bash
python3 src/gici/scripts/robnav/run_robnav_rrr.py building02
```

得到第一组正式完整 timing 数据。

---

## 52. 完整 building02 之后分析什么

重点看：

```text
process_cpu_time_s
detail_module_sum_s

各 detail 模块：
    count
    total_ms
    mean_ms
    p95_ms

尤其：
    lidar_preprocess
    gnss_aux_rtk
    gnss_factor
    lidar_factor
    optimization
    gnss_post
    lidar_post
    marginalization
```

并计算每个 detail 模块的：

\[
ratio_i =
\frac{T_i}{T_{\text{process CPU}}}.
\]

这里的 ratio 只用于理解计算组成，不要求所有 detail ratio 加起来严格等于 100%。

---

## 53. 然后再讨论论文模块

完整 building02 出来后，再决定最终论文只展示哪些一级模块。

原则：

```text
详细记录 ≠ 详细展示
```

底层 profiling 继续完整保存，但论文主表大概率只保留：

```text
3~5 个主要模块
+
Total
```

具体模块根据实际占比和 GRLINS 表格结构再定。

---

# Part R. 最终四组实验

## 54. 数据集

论文正式结果：

```text
building02
street00
street01
street02
```

每组至少运行：

```text
glins.nav
timing_events.csv
timing_summary.csv
run_summary.csv
estimator_runtime.yaml
runtime_config.yaml
```

全部保持：

```text
同一机器
同一 build mode
同一 GLINS代码版本
同一配置
不启动 RViz
不关闭原有 visualization output
```

---

# 55. 本阶段完成判据

本阶段实现完成的条件：

```text
[1] 只修改 run_robnav_rrr.py，不修改 estimator / YAML。

[2] 每次 run 自动建立独立结果目录。

[3] GLINS_RESULT_DIR 自动注入。

[4] 能可靠定位 gici_ros_main PID。

[5] 正式播放前记录 process CPU start。

[6] rosbag结束后固定等待 1 s callback 窗口。

[7] 固定窗口后、SIGINT 前记录 process CPU end。

[8] GLINS 通过 SIGINT 正常退出并触发 Recorder flush。

[9] 自动生成：
    glins.nav
    timing_events.csv
    timing_summary.csv
    run_summary.csv
    estimator_runtime.yaml
    runtime_config.yaml

[10] timing_summary 只做底层模块统计，不提前做论文模块合并。

[11] run_summary 明确给出：
    process_cpu_time_s
    detail_module_sum_s
    post-play wait 时间
    NAV数量
    frontend/backend数量
    run_valid

[12] 60 s building02 自动化测试通过。

[13] 完整 building02 跑通后，再进入“论文最终展示哪些模块”的讨论。
```
