"""Open and close the Robotiq gripper through MoveIt, with speed and effort.

MoveIt plans the finger_joint motion, so the same code works with the fake demo (demo.launch), the
simulated Robotiq action server and the real gripper. Before execution the plan is rescaled to the
requested speed and the effort is written into the last trajectory point: the Robotiq action server
uses the effort as the gripper force [%] and derives the gripper speed from the duration. The fake
controllers follow the timing and ignore the effort.

    from panda_robotiq_moveit_config.gripper import Gripper

    gripper = Gripper()
    gripper.close(effort=30)                          # 30 % force
    gripper.open()                                    # force: trajectory_force of the action server
    gripper.move(width=0.04, speed=0.05, effort=20)   # 40 mm opening at 50 mm/s
    print(gripper.width())
"""
import rospy
from moveit_commander import MoveGroupCommander

STROKE = 0.085     # [m] opening between the finger pads when fully open
MAX_ANGLE = 0.8    # [rad] finger_joint when fully closed, as in the URDF and the Robotiq driver
MAX_SPEED = 0.1    # [m/s] speed range of the gripper, as in the Robotiq driver
MIN_SPEED = 0.013


def width_to_angle(width):
    """Opening between the finger pads [m] -> finger_joint [rad]. Same linear model as the Robotiq driver."""
    return min(MAX_ANGLE, max(0.0, MAX_ANGLE * (1.0 - width / STROKE)))


def angle_to_width(angle):
    """finger_joint [rad] -> opening between the finger pads [m]."""
    return min(STROKE, max(0.0, STROKE * (1.0 - angle / MAX_ANGLE)))


class Gripper:
    """Thin wrapper around the MoveIt group of the gripper.

    rospy.init_node() and moveit_commander.roscpp_initialize() must have been called before.
    """

    def __init__(self, group='robotiq_gripper', joint='finger_joint'):
        self._group = MoveGroupCommander(group)
        self._joint = joint

    def open(self, speed=MAX_SPEED, effort=None, wait=True):
        """Go to the named state 'open'."""
        return self._execute('open', speed, effort, wait)

    def close(self, speed=MAX_SPEED, effort=None, wait=True):
        """Go to the named state 'close'. On a real gripper the motion ends early when an object is grasped."""
        return self._execute('close', speed, effort, wait)

    def move(self, width=None, angle=None, speed=MAX_SPEED, effort=None, wait=True):
        """Move to an opening `width` [m] or to a finger_joint `angle` [rad]."""
        if width is not None:
            angle = width_to_angle(width)
        if angle is None:
            raise ValueError('give width or angle')
        return self._execute({self._joint: float(angle)}, speed, effort, wait)

    def angle(self):
        """Current finger_joint [rad]."""
        return self._group.get_current_joint_values()[0]

    def width(self):
        """Current opening between the finger pads [m]."""
        return angle_to_width(self.angle())

    def stop(self):
        self._group.stop()

    def _execute(self, target, speed, effort, wait):
        """Plan to `target` (named state or joint dict), rescale to `speed`, set the effort, execute."""
        self._group.set_max_velocity_scaling_factor(1.0)
        self._group.set_max_acceleration_scaling_factor(1.0)
        if isinstance(target, str):
            self._group.set_named_target(target)
        else:
            self._group.set_joint_value_target(target)
        success, plan, _, _ = self._group.plan()
        points = plan.joint_trajectory.points
        if not success or not points:
            rospy.logerr('Gripper: planning to %s failed', target)
            return False

        # MoveIt's time parameterization is conservative for this single joint, so the plan is rescaled
        # to the requested speed [m/s] of the finger pads.
        speed = min(MAX_SPEED, max(MIN_SPEED, speed))
        travel = abs(angle_to_width(points[-1].positions[0]) - angle_to_width(points[0].positions[0]))
        planned = points[-1].time_from_start.to_sec()
        if travel > 1e-4 and planned > 0.0:
            factor = (travel / speed) / planned
            for p in points:
                p.time_from_start = rospy.Duration(p.time_from_start.to_sec() * factor)
                p.velocities = [v / factor for v in p.velocities]
                p.accelerations = [a / factor ** 2 for a in p.accelerations]
        if effort is not None:
            points[-1].effort = [float(effort)] * len(points[-1].positions)
        return self._group.execute(plan, wait=wait)
