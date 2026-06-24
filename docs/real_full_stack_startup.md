# TETO Real UR5e Full-Stack Startup

The real UR5e is expected at `192.168.20.35`. Configure the robot teach
pendant to connect back to the wired PC interface at `192.168.20.36:50002`.
The wrapper passes `reverse_ip:=192.168.20.36` and keeps the previously
successful `headless_mode:=false` behavior by default.

Start the UR driver, wait for the controller manager, ensure
`tcp_pose_broadcaster` is loaded and active, bring up the existing Qwen/MoveIt
operator dependencies, and enter the TETO console with:

```bash
bash scripts/start_teto_real_full_stack.sh
```

This startup can start real-robot-related services: the UR driver,
`tcp_pose_broadcaster`, Qwen readiness, MoveIt readiness, controller
activation, and then the real operator console. It must not be run without an
operator physically present in the lab and explicit permission for the real
UR5e stack.

The default console path is unified segmented real mode. It is no longer the
old per-command manual `y` confirmation flow. The legacy manual flow exists
only when explicitly requested through the legacy manual mode described below.

Default real motion uses autonomous segmented execution with measured
per-segment gates. Autonomous does not mean ungated. Before each segment can
call the real execution path, the B1 real backend and unified operator require:

- Dashboard state reachable and acceptable
- scaled controller active
- joint state available
- MoveIt available
- fresh current TCP pose
- D455 snapshot freshness, RGB/depth sync, and newer-than-previous guard
- bounded relative motion contract
- cartesian safety gateway contract

After each segment, the unified operator requires measured verification:

- after TCP pose
- position error
- direction projection
- orientation change
- post-motion verification result

Dry-run, plan-only, Isaac, fake, or synthetic evidence must not be accepted as
REAL_PATH success evidence.

Inspect current status without starting anything:

```bash
bash scripts/start_teto_real_full_stack.sh --status
```

Status mode is status-only. It prints wrapper, ROS, controller, TCP topic, and
port information and does not start Qwen, MoveIt, the UR driver, RealSense, or
the operator console.

Stop only processes recorded as started by this wrapper:

```bash
bash scripts/start_teto_real_full_stack.sh --stop
```

If an owned launch process does not respond to the graceful stop, inspect its
PID and command with `--status`, then force-stop only that recorded process
group with:

```bash
bash scripts/start_teto_real_full_stack.sh --force-stop-owned
```

To bring up the stack without entering the console, add `--no-console`. To use
a different robot address, pass `--robot-ip IP`. The PC address can be changed
with `--reverse-ip IP`.

`--no-console` can still start hardware-related services because it performs
the real full-stack bringup before exiting. Treat it as a lab-only command, not
as an offline test command.

The downstream Qwen real operator startup script also has a status-only mode:

```bash
bash scripts/start_teto_qwen_real_operator.sh --status
```

That status-only mode does not start Qwen, launch MoveIt, switch controllers,
or open the operator console. The normal full-stack startup command above can
start those dependencies.

To request the old guarded real-small-motion console with per-command `y`
confirmation, use the explicit legacy path:

```bash
bash scripts/start_teto_qwen_real_operator.sh --legacy-manual-console
```

Legacy manual mode is not the default B1/B2 real operator path.

Logs and PID files are stored under `outputs/real_full_stack/`. If
`/controller_manager`, `/controller_manager/list_controllers`, or
`/tcp_pose_broadcaster/pose` does not appear before its timeout, startup fails
closed and prints controller state, relevant ROS topics, and UR driver log
diagnostics.

## Configuration package timeout

The installed `ur_client_library` waits only one second for a configuration
package from the robot primary interface. A failure resembling:

```text
Could not get configuration package within timeout
```

can occur even after Dashboard and RTDE connect successfully. Those successful
connections prove basic reachability but do not prove that the primary
interface delivered the robot configuration and kinematics package.

Local logs show that the same command succeeded on June 7, 2026 after two
earlier attempts failed with this exact timeout:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.20.35 \
  launch_rviz:=false
```

No different old launch arguments were found. The wrapper therefore performs
three bounded, clean attempts by default. It stops the wrapper-owned process
group between attempts and fails closed if all attempts fail. Change the
wrapper's controller-manager wait with `--driver-timeout SECONDS`, or the
attempt count with `--driver-attempts COUNT`. This does not alter the driver's
hard-coded internal one-second configuration-package timeout.

If repeated attempts fail, verify that the teach pendant and External Control
installation/program are in the expected state, no stale UR driver or other
primary-interface client is running, and the wired PC interface still owns
`192.168.20.36`. Review the exact launch command printed by the wrapper and
`outputs/real_full_stack/ur_robot_driver.log`.

Protective-stop recovery, unlock, dashboard recovery actions, and any other
robot recovery step must be performed by a qualified human operator. They must
not be performed automatically by an LLM or by this startup wrapper.

Advanced launch arguments can be appended without shell evaluation:

```bash
bash scripts/start_teto_real_full_stack.sh \
  --ur-launch-extra "controller_spawner_timeout:=20"
```

The wrapper rejects overrides of robot identity, reverse/headless settings,
RViz, and safety-limit arguments. Use `--headless-mode true|false` explicitly
for headless behavior; the default remains `false` for the teach-pendant
External Control workflow.
