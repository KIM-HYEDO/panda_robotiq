# Franka Panda + Robotiq 2F-85

ROS Noetic packages that integrate a Robotiq 2F-85 gripper with a Franka Emika Panda arm and plan with MoveIt.
They do not modify `franka_ros` or `robotiq_2finger_grippers`.

| Package | Content |
|---------|---------|
| `panda_robotiq_description` | URDF/xacro of the arm with the gripper mounted on `panda_link8`, RViz display |
| `panda_robotiq_moveit_config` | MoveIt config: planning groups, collision matrix, fake controllers, demo launch |

## Install

Needs ROS Noetic, MoveIt and, in the same catkin workspace:

- [`franka_ros`](https://github.com/frankaemika/franka_ros) (`franka_description`, `franka_control`)
- [`robotiq_2finger_grippers`](https://github.com/KIM-HYEDO/robotiq_2finger_grippers) (gripper model, driver,
  action server and the RViz gripper panel)

```bash
cd ~/catkin_ws/src
git clone https://github.com/KIM-HYEDO/panda_robotiq.git
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y
catkin build panda_robotiq_moveit_config && source devel/setup.bash
```

## Quick start (no robot needed)

```bash
cd ~/catkin_ws && catkin build panda_robotiq_moveit_config && source devel/setup.bash

roslaunch panda_robotiq_description display.launch     # check the model with joint sliders
roslaunch panda_robotiq_moveit_config demo.launch      # plan and execute with fake controllers
```

In RViz, pick a planning group, drag the interactive marker and press **Plan & Execute**.

## Planning groups

| Group | Joints | Tip link | Named states |
|-------|--------|----------|--------------|
| `panda_arm` | `panda_joint1..7` | `panda_link8` (flange) | `ready`, `home` (same pose), `extended` |
| `panda_manipulator` | `panda_joint1..7` | `robotiq_tcp` | `ready`, `home` (same pose), `extended` |
| `robotiq_gripper` | `finger_joint` | | `open` (0 rad), `close` (0.7 rad) |

Plan end-effector poses with `panda_manipulator`: its tip is the middle of the finger pads.
`finger_joint` is the only actuated gripper joint, the other finger joints mimic it.

## Gripper with speed and effort

`moveit.rviz` includes the **Gripper** panel of `robotiq_2f_gripper_rviz` (the controls of the
`robotiq_2f_gripper_examples` GUI: speed and force sliders, OPEN, CLOSE, STOP, status). It talks to the
Robotiq action server directly, so it is live with `real_robot.launch` and shows "Waiting for action
server" with `demo.launch`, where the gripper is driven by MoveIt's fake controllers instead.

The MotionPlanning panel can only pick `open`/`close`. For a given speed [m/s] and effort (gripper force
in %, 0 to 100) from a script, use the helper, from the command line:

```bash
rosrun panda_robotiq_moveit_config gripper.py close --effort 30
rosrun panda_robotiq_moveit_config gripper.py width 0.04 --speed 0.05 --effort 20
rosrun panda_robotiq_moveit_config gripper.py open
rosrun panda_robotiq_moveit_config gripper.py status
```

or from Python:

```python
from panda_robotiq_moveit_config.gripper import Gripper

gripper = Gripper()                              # after rospy.init_node and moveit_commander.roscpp_initialize
gripper.close(effort=30)
gripper.move(width=0.04, speed=0.05, effort=20)
gripper.open()
```

The helper plans with MoveIt, rescales the plan to the requested speed and writes the effort into the last
point of the trajectory, so it works with `demo.launch` (fake controllers follow the timing, ignore the
effort) and on the real robot, where the Robotiq action server uses the effort as force and the duration as
speed. Without `effort` the action server uses its `trajectory_force` parameter (`gripper_force` argument
of `real_robot.launch`, default 50 %), which is also what RViz's Plan & Execute gets. RViz motions are
slower: the time parameterization gives about 1.7 s for a full close at velocity scaling 1.0, and the
RViz default 0.1 ends up at the minimum gripper speed (about 6 s).

`ready` and `extended` use `panda_joint7 = 0` instead of the stock `pi/4`. The stock value cancels the
-45 deg mount of the Franka Hand. The Robotiq gripper sits at 0 deg on the flange, so `joint7 = 0` puts its
fingers where the Franka Hand's would be: opening along the base y axis, TCP z pointing down. To open along
the base x axis instead, set `joint7` to `pi/2` in `config/arm.xacro`.

## Mounting and TCP

The gripper pose relative to the flange and the TCP are launch arguments:

```bash
roslaunch panda_robotiq_moveit_config demo.launch mount_xyz:="0 0 0.01" mount_rpy:="0 0 0.785" tcp_xyz:="0 0 0.1303"
```

| Argument | Default | Meaning |
|----------|---------|---------|
| `mount_xyz`, `mount_rpy` | `0 0 0` | `robotiq_arg2f_base_link` relative to `panda_link8`. The gripper sits directly on the flange, 45 deg from the Franka Hand mount. Set them to match your adapter plate |
| `tcp_xyz` | `0 0 0.1303` | TCP relative to the gripper base, the middle of the finger pads |

`mount_rpy` reproduces the 45 deg offset from the Franka Hand seen on the hardware. Check it at `ready`:
the real fingers must open along the base y axis. If they open along x, use `mount_rpy:="0 0 1.5708"`.
`mount_xyz` and `tcp_xyz` are placeholders until the real mounting is measured.

## Real robot: end-effector frame of franka_control

MoveIt plans with the URDF, so it does not care what the robot thinks its end effector is.
`franka_control` does: `O_T_EE`, `O_T_EE_d`, `O_dP_EE_d`, the end-effector Jacobian and every
Cartesian controller of `franka_example_controllers` use the frame configured in the robot,
`F_T_EE = F_T_NE * NE_T_EE`. `F_T_NE` is set in Desk and defaults to the Franka Hand
(yaw -45 deg, z 0.1034 m): 45 deg off and 2.7 cm short of `robotiq_tcp`.

Two ways to fix it:

- Desk (permanent): Settings > End-Effector, enter the flange-to-TCP transform (yaw 0, z 0.1303 m with
  the default mount) and the mass and centre of mass of the gripper.
- At runtime, once `franka_control.launch` is up and before a controller is started:

  ```bash
  rosrun panda_robotiq_description set_ee_frame.py
  ```

  It reads `F_T_NE` from the robot and `panda_link8 -> robotiq_tcp` from TF, and calls
  `/franka_control/set_EE_frame` so that `O_T_EE` equals the `robotiq_tcp` pose MoveIt uses.
  It needs `robot_description` to be the Robotiq URDF. The mass still has to be set in Desk
  (or with `/franka_control/set_load`).

## Real robot

```bash
roslaunch panda_robotiq_moveit_config real_robot.launch robot_ip:=<ip> comport:=/dev/ttyUSB0
```

This starts `bringup.launch` (`franka_control`, `position_joint_trajectory_controller`, the Robotiq action
server, one `/joint_states` with every joint, `set_ee_frame.py`), `move_group` with
`config/simple_moveit_controllers.yaml` and RViz. MoveIt sends arm trajectories to
`position_joint_trajectory_controller/follow_joint_trajectory` and gripper trajectories to
`robotiq_controller/follow_joint_trajectory`, the FollowJointTrajectory server of the Robotiq action server.
The gripper takes the last point of the trajectory as target, moves at the speed that reaches it in the
planned time (MoveIt velocity scaling applies), closes with the `~trajectory_force` of the action server
(default 50 %) and stops when the trajectory is cancelled.

| Argument | Default | Meaning |
|----------|---------|---------|
| `robot_ip` | | Panda controller. FCI enabled and the robot unlocked in Desk |
| `comport`, `baud` | `/dev/ttyUSB0`, `115200` | Serial port of the gripper (`dmesg \| grep tty`) |
| `gripper_sim` | `false` | Simulate the gripper, no serial port needed |
| `transmission` | `position` | Arm controller: `position` or `effort` |
| `set_ee_frame` | `true` | Run `set_ee_frame.py` once at start |

`bringup.launch` alone starts the hardware without MoveIt, e.g. for `franka_example_controllers`.

## Status

- Done: model, planning groups, collision matrix, fake-controller demo (arm, gripper and TCP goals verified headless).
- Verified without hardware: the gripper path of `real_robot.launch` (Robotiq action server in `sim` mode +
  MoveIt simple controller manager) executes `open`/`close`, respects velocity scaling and stops on cancel.
- Not verified yet: the arm path on the real Panda. It mirrors `panda_moveit_config/launch/franka_control.launch`.
- Not done: Gazebo and a pick-and-place example.
