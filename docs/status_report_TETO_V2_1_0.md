# TETO 项目当前进展现状汇报｜V2.1.0

生成时间：2026-05-31  
项目路径：`/home/newusername/teto_vlm`

## 1. 项目一句话定位

TETO 当前不是机器人控制系统，而是 VLM/语义理解到 Isaac Sim 仿真执行之间的分层中间件与验证框架。当前遵循安全分层：VLM/LLM 只输出结构化语义任务或中间表示，不直接生成 URScript、joint angles、tcp_pose_world 或高频控制命令。

换句话说，TETO 现在重点解决的是：

- VLM/语义任务如何被规范化、缓存、回放、验证；
- 仿真执行链路是否能被最小化触发；
- 每次执行是否能输出可审计 report 和 evidence；
- robot asset 是否能以受控方式进入 Isaac World。

它还没有进入真实机器人控制、ROS2/MoveIt 集成或 UR5 运动控制阶段。

## 2. 当前稳定版本与仓库状态

当前稳定版本：

- `TETO V2.1.0`
- latest commit：`94e1e8e`
- commit message：`TETO V2.1.0 accepted: robot asset loader contract`

本次自查执行 `git status`：

```text
On branch master
nothing to commit, working tree clean
```

本次自查执行 `git log --oneline -8`：

```text
94e1e8e TETO V2.1.0 accepted: robot asset loader contract
5b44fdc TETO V2.0.3 accepted: generic execution evidence export
9a565ff TETO V2.0.2 accepted: simulation object pose update smoke test
296c21a TETO V2.0.1 accepted: spawn cube in Isaac world
973294f TETO V2.0.0 follow-up: stabilize true Isaac report writing
6e2d6bd TETO V2.0.0 accepted: first simulation execution
9c596ed TETO V1.9.0 accepted: simulation bridge contract
5568b67 TETO V1.8.1 accepted: execution readiness replay display
```

`git remote -v` 当前无输出，本次自查没有执行 push、commit、reset、rebase 或代码修改。

## 3. 版本路线总览

### V1.x：Semantic Middleware Era

V1.x 主要建立语义中间件层：

- 图像识别 / batch recognition 的结构化输出；
- robot task JSON inspection；
- semantic replay；
- planner gateway input contract；
- 2D grounding / scene snapshot / scene cache replay；
- execution readiness contract；
- simulation bridge contract。

这一阶段明确不调用 Isaac Sim API，不接 ROS2、MoveIt、UR5、RTDE，也不生成 URScript、joint angles、trajectories 或 tcp_pose_world。

### V2.0.0：First Simulation Execution

V2.0.0 第一次打通最小 Isaac Runtime 执行链路：

- 创建 `SimulationApp`；
- 创建 `World`；
- 执行 `world.reset`；
- 运行少量 simulation steps；
- 输出结构化 `simulation_execution_result.json`；
- true Isaac report PASS。

这是从纯语义中间件进入 Isaac Runtime 的第一个稳定验收点。

### V2.0.1：Spawn Cube in Isaac World

V2.0.1 在 Isaac World 中加入一个最小可观察对象：

- spawn 默认 cube fixture；
- report 中记录 `cube_spawned`、`cube_prim_path`、`cube_position`、`cube_size`；
- dry-run 模式模拟相同结构；
- 不移动 cube，不保存截图/视频，不接机器人控制。

### V2.0.2：Generic Simulation Object Pose Update Smoke Test

V2.0.2 将 “cube 移动” 收敛为 generic simulation object pose update smoke test：

- `--move-object` 是正式主入口；
- `--move-cube` 只是 backward-compatible alias；
- 内部使用 `SimulationObjectSpec`、`spawn_simulation_object`、`update_simulation_object_pose`；
- cube 只是 default fixture，不是长期主线 feature；
- report 记录 object initial / target / final position 与 displacement。

### V2.0.3：Generic Execution Evidence Export

V2.0.3 建立 generic evidence export pipeline。每次 execution report 写出后，同目录生成：

- `simulation_execution_result.json`
- `summary.md`
- `demo_command.txt`
- `pose_delta.md`
- `evidence_manifest.json`

manifest 中保留截图/视频 placeholder：

- `screenshot_before_path: null`
- `screenshot_after_path: null`
- `video_path: null`

本阶段不生成截图、不录像，只建立 evidence/export 管线。

### V2.1.0：Robot Asset Loader Contract / Availability Smoke Test

V2.1.0 建立 robot asset loader contract，而不是 UR5 控制功能：

- 支持 robot asset check / load；
- 支持 `--robot-asset-path`、`--robot-type`、`--robot-prim-path`；
- 无本地 asset 时 check mode 是 PASS diagnostic；
- 显式无效 load path 时是 FAIL；
- report 增加 `robot_asset_available`、`robot_asset_loaded`、`robot_prim_exists`、`robot_asset_status` 等字段；
- evidence summary/manifest 增加 Robot Asset section。

## 4. 当前 Isaac 环境

Isaac Sim 安装路径：

```text
/home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64
```

Isaac Python 路径：

```text
/home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh
```

`python.sh` 已确认存在且可执行。

Isaac 版本文件：

```text
5.1.0-rc.19+release.26219.9c81211b.gl
```

NVIDIA driver 状态：

- `/proc/driver/nvidia/version` 显示当前 kernel module 为 `580.159.03`。
- 当前 shell 下执行 `nvidia-smi` 返回无法与 NVIDIA driver 通信。
- 最近 true Isaac headless execution 仍然可以生成 PASS report。

历史环境结论：

- 之前 `595.71.05 + RTX 5060 Ti` 会导致 Isaac Runtime / GUI 崩溃；
- 切换到 `580.159.03` 后，`SimulationApp`、`World`、`world.reset`、GUI、true Isaac execution 均已恢复；
- NGX / DLSS 类 warning 当前不阻塞 Runtime / GUI / report 生成。

本次复查日志中仍可见 NVML driver warning，但没有阻止 true Isaac report PASS。

## 5. Official UR5e Asset Local Load Verification

官方 UR5e root asset 本地路径：

```text
/home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd
```

本地 root asset 已存在，大小约 `4.8K`。

相关 configuration dependency 已存在：

```text
/home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_base.usd
/home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_physics.usd
/home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_robot_schema.usd
/home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_sensor.usd
```

最近成功 run：

```text
outputs/simulation_runs/run_20260531_114132
```

该 run 的 `simulation_execution_result.json` 关键字段：

```text
status: PASS
mode: isaac
error.code: OK
robot_asset_available: true
robot_asset_loaded: true
robot_prim_exists: true
robot_asset_status: LOADED
robot_asset_blocking_reason: null
robot_asset_path: /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd
robot_prim_path: /World/TETO_Robot
```

同目录 evidence artifacts 齐全：

```text
simulation_execution_result.json
summary.md
demo_command.txt
pose_delta.md
evidence_manifest.json
```

`summary.md` 包含 `## Robot Asset` section。`evidence_manifest.json` 包含 `robot_asset` 字段，并且截图/视频仍然只是 null placeholder：

```text
screenshot_before_path: null
screenshot_after_path: null
video_path: null
```

最近复测日志中未发现以下缺依赖或加载失败信息：

```text
Could not open asset
Could not load sublayer
PhysicsUSD CreateJoint no bodies defined
```

因此当前可以记录为：official UR5e asset local load verified。

## 6. 当前明确没有做的事情

当前仍然没有：

- ROS2；
- MoveIt；
- RTDE；
- URScript；
- real UR5；
- joint angles；
- tcp_pose_world；
- actual_TCP_pose 控制链路；
- 真机运动控制。

当前已完成的范围仅限于：

- Isaac Runtime 最小执行；
- World / reset / steps；
- generic simulation object pose update smoke test；
- evidence export；
- robot asset availability / loading contract；
- 官方 UR5e USD asset 的本地加载验证。

## 7. 当前成果的意义

V2.0.x 证明 TETO 已经具备从结构化语义任务进入 Isaac Runtime 的最小闭环：

- 可以创建 Isaac `SimulationApp` 和 `World`；
- 可以执行 reset 与 steps；
- 可以 spawn / update 仿真对象；
- 可以输出稳定 report；
- 可以为每次运行生成 evidence artifacts。

V2.1.0 进一步证明：

- robot asset loading 已经有通用 contract；
- 官方 UR5e asset 可以作为本地 USD 被 Isaac 成功加载；
- `/World/TETO_Robot` prim 可以在 stage 中存在并被 report 记录；
- evidence pipeline 可以记录 robot asset metadata。

这一步对后续很关键，因为它把 “UR5e 是否能进入 Isaac World” 从不确定环境问题，推进为可复现、可报告、可审计的本地加载结果。

但这仍不等价于机器人控制。当前只是 robot asset preparation / loading 阶段，还没有读取 articulation DOF、没有生成 joint target、没有进行 motion planning、没有和真实 UR5 控制链路连接。

## 8. 下一步建议

建议保守推进到 `TETO V2.1.1` 或 `TETO V2.1.2`：UR5e prim inspection。

目标应保持只读检查 `/World/TETO_Robot` 下面的 prim 结构，例如：

- links；
- joints；
- articulation root；
- collision / visual prim；
- physics schema；
- possible DOF metadata。

推荐原则：

- 只 inspect，不 control；
- 只输出 report/evidence，不生成控制命令；
- 不生成 joint targets；
- 不生成 tcp_pose_world；
- 不接 ROS2 / MoveIt / RTDE / URScript；
- 不连接 UR5 真机。

建议最小实现方式：

- 在 V2.1.0 robot asset loader contract 后增加只读 prim traversal；
- report 新增 `robot_prim_inspection_requested`、`robot_link_count`、`robot_joint_count`、`articulation_root_found` 等诊断字段；
- evidence summary 增加 Robot Prim Inspection section；
- dry-run 保持纯 Python，不依赖 Isaac；
- true Isaac 仅在加载 asset 后做 USD stage prim inspection。

## 9. Codex 自查命令与结果摘要

本次执行过的关键命令：

```bash
git status
git log --oneline -8
git remote -v
test -x /home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh
nvidia-smi
cat /proc/driver/nvidia/version
cat /home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/VERSION
rg -n "V2\\.1\\.0|V2\\.0\\.3|V2\\.0\\.2|V2\\.0\\.1|V2\\.0\\.0|V1\\.|UR5e|UR5|Robot Asset|Isaac|580|595|NGX|DLSS|ROS2|MoveIt|RTDE|URScript|joint angles|tcp_pose_world" README.md teto_V1.py src/batch_recognition.py src/simulation_runtime.py src/evidence_exporter.py scripts/run_first_simulation_execution.py tests
find outputs/simulation_runs -name simulation_execution_result.json -printf '%T@ %p\n' | sort -nr | head -10
ls -lh /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_base.usd /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_physics.usd /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_robot_schema.usd /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_sensor.usd
python3 -m json.tool outputs/simulation_runs/run_20260531_114132/simulation_execution_result.json
python3 -m json.tool outputs/simulation_runs/run_20260531_114132/evidence_manifest.json
ls -1 outputs/simulation_runs/run_20260531_114132
rg -n "## Robot Asset|robot asset available|robot asset loaded|robot asset status|robot asset blocking reason" outputs/simulation_runs/run_20260531_114132/summary.md
rg -n "Could not open asset|Could not load sublayer|PhysicsUSD CreateJoint no bodies defined" /tmp/teto_ur5e_retest.log
git ls-files outputs/simulation_runs
git ls-files /home/newusername/Storage/isaac_assets
git status --short
```

关键结果摘要：

- git 工作区初始状态 clean；
- latest commit 为 `94e1e8e TETO V2.1.0 accepted: robot asset loader contract`；
- Isaac `python.sh` 存在且可执行；
- Isaac version 为 `5.1.0-rc.19+release.26219.9c81211b.gl`；
- `/proc/driver/nvidia/version` 显示 NVIDIA kernel module `580.159.03`；
- 当前 shell 下 `nvidia-smi` 无法通信，但最近 true Isaac execution report 为 PASS；
- UR5e root asset 与 configuration dependency 均存在；
- 最近 UR5e load run `run_20260531_114132` 为 `PASS / LOADED`；
- evidence artifacts 齐全；
- manifest 中 screenshot/video placeholder 仍为 null；
- 未发现 UR5e 缺依赖 warning；
- `outputs/simulation_runs` 没有被 git 跟踪；
- `/home/newusername/Storage/isaac_assets` 位于仓库外，不属于 TETO repo 跟踪范围。

