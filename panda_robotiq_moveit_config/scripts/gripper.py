#!/usr/bin/env python3
"""Command line for the gripper through MoveIt (demo.launch or real_robot.launch must be running).

    rosrun panda_robotiq_moveit_config gripper.py open
    rosrun panda_robotiq_moveit_config gripper.py close --effort 30
    rosrun panda_robotiq_moveit_config gripper.py width 0.04 --speed 0.05 --effort 20
    rosrun panda_robotiq_moveit_config gripper.py status
"""
import argparse
import sys

import moveit_commander
import rospy

from panda_robotiq_moveit_config.gripper import MAX_SPEED, Gripper


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('open', 'close', 'width'):
        sub = commands.add_parser(name)
        if name == 'width':
            sub.add_argument('value', type=float, help='opening between the finger pads [m], 0 to 0.085')
        sub.add_argument('--speed', type=float, default=MAX_SPEED, help='[m/s], 0.013 to 0.1 (default %(default)s)')
        sub.add_argument('--effort', type=float, default=None,
                         help='gripper force [%%], 0 to 100. Default: trajectory_force of the action server')
    commands.add_parser('status')
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gripper_cli', anonymous=True)
    gripper = Gripper()

    if args.command == 'status':
        print('finger_joint %.3f rad, width %.3f m' % (gripper.angle(), gripper.width()))
        return 0
    if args.command == 'open':
        ok = gripper.open(speed=args.speed, effort=args.effort)
    elif args.command == 'close':
        ok = gripper.close(speed=args.speed, effort=args.effort)
    else:
        ok = gripper.move(width=args.value, speed=args.speed, effort=args.effort)
    print('%s -> %s, finger_joint %.3f rad, width %.3f m' % (
        args.command, 'done' if ok else 'FAILED', gripper.angle(), gripper.width()))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
