# FMS Project — Verification Command List

Goal: confirm Phase 1 (simulation) and Phase 2 (robot FSM/BT agent) are
working correctly end-to-end, especially the wheel-axis fix
(`axis xyz="0 0 -1"` on wheel joints), before starting Phase 3.

## ⚠️ Required in EVERY new terminal

`~/.bashrc` auto-sources `/opt/ros/humble/setup.bash` and
`~/mobile_robot_ws/install/setup.bash`, but **NOT** `~/fms_ws/install/setup.bash`.
If you skip this, any command touching `fms_msgs` fails with errors like:
```
The message type 'fms_msgs/msg/RobotStatus' is invalid
ModuleNotFoundError: No module named 'fms_msgs'
```
Fix — run this (or the `fms` alias, which does the same thing) in **every**
terminal before any command below:
```bash
fms
# equivalent to:
# source /opt/ros/humble/setup.bash && source ~/fms_ws/install/setup.bash
```

Convention used below:
- **T1, T2, T3, ...** = separate terminal windows/tabs. Run `fms` in each
  one first (see above), then `CYCLONEDDS_URI`/`RMW_IMPLEMENTATION` are
  already exported globally via `~/.bashrc`.
- Steps are numbered in the order you should run them. "Wait for ..."
  steps tell you what to look for in T1's log before moving to the next step.

---

## Step 0 — Build (T1)

```bash
cd ~/fms_ws
colcon build --symlink-install
```
Expect: `Summary: 4 packages finished` with no errors
(`fms_gazebo`, `fms_msgs`, `fms_navigation`, `fms_robot_agent`).

---

## Step 1 — Launch full single-robot bringup (T1)

```bash
ros2 launch fms_navigation bringup.launch.py num_robots:=1
```

Leave this running. **Wait ~2–3 minutes** until you see, for `robot_1`:
- `lifecycle_manager_localization` → "Managed nodes are active"
- `lifecycle_manager_navigation` → "Managed nodes are active"
- Gazebo GUI window open, robot spawned at its charge dock (west wall)
- RViz window open showing the robot and map

Do **not** proceed to Step 2 until both lifecycle managers report active —
sending commands earlier is the #1 cause of "action server not available"
errors (see memory: FMS Startup Behavior).

---

## Step 2 — Verify core topics are alive (T2)

```bash
ros2 topic list | grep robot_1
```
Expect to see at least:
```
/robot_1/cmd_vel
/robot_1/odom
/robot_1/tf
/robot_1/scan
/robot_1/joint_states
/robot_1/imu
/robot_1/robot_status
/robot_1/task_completion
/robot_1/task_assignment
```

Check odom is publishing:
```bash
ros2 topic hz /robot_1/odom
```
Expect ~20 Hz (matches `odom_publish_frequency`).

Check TF tree:
```bash
ros2 run tf2_tools view_frames
```
Expect chain: `map → robot_1/odom → robot_1/base_footprint → robot_1/base_link → (lidar_link, imu_link, wheels)`.

---

## Step 3 — Send a task (T2)

**The robot does NOT move on its own.** `RequestTask` (the first BT node in
`TaskExecution`) blocks until a `TaskAssignment` message arrives on
`/robot_1/task_assignment`. Without this step, odom will sit essentially
at zero forever — that is expected, not a bug.

```bash
cd ~/fms_ws
python3 src/fms_robot_agent/scripts/task_injector.py --count 1 --robots 1
```
Leave this running — it also prints periodic fleet status and waits for
the completion message.

---

## Step 4 — Verify wheel-axis fix (THE critical check) (T3 + Gazebo/RViz GUIs)

This confirms the robot moves the SAME direction in Gazebo and RViz
(no more west-wall collision while RViz shows success), now that it has
a task to execute.

4a. Watch `robot_1`'s odom position update live:
```bash
ros2 topic echo /robot_1/odom --field pose.pose.position
```
Expect `x` to start increasing from ~0 toward ~14 (east, toward the pick
station) — not drifting toward negative x (west wall).

4b. Watch BOTH:
- **Gazebo GUI**: does the robot model visibly move east, away from the
  west wall, toward the pick station shelf?
- **RViz**: does the robot icon move the same direction by the same amount?

✅ PASS if Gazebo and RViz agree and the robot moves toward positive X
(east, into the warehouse) without hitting the west wall.
❌ FAIL (axis fix not working) if the robot spins wheels but stays pinned
against the west wall in Gazebo while RViz shows it moving away.

---

## Step 5 — Verify robot agent FSM / BT cycle (T3)

```bash
ros2 topic echo /robot_1/robot_status
```
Expect `state` / `status_message` to progress through:
```
ASSIGNED → NAVIGATING → EXECUTING → NAVIGATING → EXECUTING → REPORTING → IDLE
```
(NavToPick → Pick → NavToDrop → Drop → Report, per `robot_agent.xml`).

Confirm a completion message arrives:
```bash
ros2 topic echo /robot_1/task_completion
```
The T2 `task_injector.py` from Step 3 should also print
`FINAL: 1/1 tasks completed`.

---

## Step 6 — Manual navigation goal sanity check (T4)

Only needed if Steps 4/5 already pass — this is an extra manual check,
independent of the robot agent (drives Nav2 directly).

```bash
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: -14.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```
Watch Gazebo + RViz again for matching motion.

---

## Step 7 — Fault injection / recovery test (T4)

Send another task first (T2, repeat Step 3 with `--count 1 --robots 1`),
then run this while robot is `NAVIGATING` (check Step 5's status topic):

```bash
ros2 service call /robot_1/inject_fault std_srvs/srv/SetBool "{data: true}"
```
Expect `/robot_1/robot_status` to transition: `NAVIGATING → RECOVERING → ASSIGNED` (or `IDLE` if recovery fails after `recovery_max_attempts`).

---

## Step 8 — Low-battery → charging test (separate run, T1)

Stop Step 1 (Ctrl+C), then relaunch with a low starting battery so the
`ChargeWhenLow` branch triggers immediately:

```bash
ros2 launch fms_navigation bringup.launch.py num_robots:=1
```
> Note: `initial_battery_soc` is set per-robot in
> `robot_agent.launch.py` / `robot_agent_params.yaml`. To force a quick
> test, temporarily edit `initial_battery_soc` to `15.0` in
> `src/fms_robot_agent/config/robot_agent_params.yaml`, rebuild
> (`colcon build --packages-select fms_robot_agent`), then relaunch.

In T2, watch:
```bash
ros2 topic echo /robot_1/robot_status
```
Expect: `... → CHARGING` while the robot navigates to its charge dock,
SOC (`battery_soc`) climbing back toward 95%, then `CHARGING → IDLE`.

Remember to revert `initial_battery_soc` back to `100.0` afterward.

---

## Step 9 — Multi-robot staggered startup test (T1)

```bash
ros2 launch fms_navigation bringup.launch.py num_robots:=4
```
Wait several minutes (staggered: robot_1 Nav2 at t=10s, robot_2 at t=25s,
robot_3 at t=40s, robot_4 at t=55s, agents ~15s after each).

In T2, confirm all 4 lifecycle managers go active without DDS timeout
errors:
```bash
ros2 topic echo /robot_1/robot_status --once
ros2 topic echo /robot_2/robot_status --once
ros2 topic echo /robot_3/robot_status --once
ros2 topic echo /robot_4/robot_status --once
```
Each should return a valid message (not hang).

---

## Step 10 — Task injector / throughput test (T2, while Step 9 is running)

```bash
cd ~/fms_ws
python3 src/fms_robot_agent/scripts/task_injector.py --count 5 --robots 1 2 3 4
```
Expect periodic status printouts and a final line:
```
FINAL: 20/20 tasks completed (elapsed: ...s)
```

---

## Quick reference: terminal summary

Run `fms` (or `source ~/fms_ws/install/setup.bash`) in **every** terminal
below before running any command in it.

| Terminal | Purpose | Long-running? |
|----------|---------|---------------|
| T1 | Launch bringup (Gazebo, RViz, Nav2, agents) | Yes — keep open |
| T2 | Topic checks, TF tree, task injector | Run commands one at a time |
| T3 | Watch odom / robot_status during a task | Run commands one at a time |
| T4 | Manual nav goal, fault injection | Brief, used occasionally |

---

## Order of operations summary

1. T1: `fms` → build → launch `bringup.launch.py num_robots:=1` → wait for "active"
2. T2: `fms` → check topics/TF (Step 2)
3. T2: `fms` → run `task_injector.py --count 1 --robots 1` (Step 3) — robot won't move without this
4. T3: `fms` → watch Gazebo+RViz, echo odom (Step 4) — **wheel axis fix verification**
5. T3: echo `/robot_1/robot_status` and `/robot_1/task_completion` (Step 5)
6. (optional) T4: send manual nav goal (Step 6)
7. T2 + T4: send another task, then inject fault mid-navigation (Step 7)
8. Restart with low battery, verify charging cycle (Step 8)
9. T1: relaunch with `num_robots:=4`, T2: verify all agents active (Step 9)
10. T2: run `task_injector.py` for throughput test (Step 10)

Once all 10 steps pass, Phase 1 & 2 are confirmed working and Phase 3
(fleet server: gRPC + RabbitMQ + MongoDB + task allocator) can begin.
