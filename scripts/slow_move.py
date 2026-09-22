#!/usr/bin/env python3
"""Move a Maxon motor slowly back and forth between two positions (CSP mode).

Setpoints are ramped linearly at a fixed speed, so the motor never gets a step.
Positions are raw encoder counts. By default the motion starts from the current
position and goes `--delta` counts away and back.

Example:
    rosrun ethercat_ros_configurator slow_move.py --delta 2048 --speed 200 --cycles 3
"""
import argparse

import rospy
from ethercat_motor_msgs.msg import MotorCtrlMessage, MotorStatusMessage

POSITION_LIMIT = 50000  # matches /limits/x used by the ethercat node


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--motor', default='Maxon_Motor_bottom')
    parser.add_argument('--delta', type=int, default=2048, help='counts to move away from the start position')
    parser.add_argument('--a', type=int, help='absolute position A (default: current position)')
    parser.add_argument('--b', type=int, help='absolute position B (default: A + delta)')
    parser.add_argument('--speed', type=float, default=200.0, help='counts per second')
    parser.add_argument('--hold', type=float, default=1.0, help='seconds to wait at each end')
    parser.add_argument('--cycles', type=int, default=1, help='number of A -> B -> A round trips')
    parser.add_argument('--rate', type=float, default=100.0, help='command rate in Hz')
    args, _ = parser.parse_known_args(rospy.myargv()[1:])

    rospy.init_node('slow_move')
    base = '/ethercat_master/%s' % args.motor
    pub = rospy.Publisher(base + '/command', MotorCtrlMessage, queue_size=1)

    rospy.loginfo('Waiting for %s/reading ...', base)
    reading = rospy.wait_for_message(base + '/reading', MotorStatusMessage, timeout=10.0)
    start = reading.actualPosition

    a = args.a if args.a is not None else start
    b = args.b if args.b is not None else a + args.delta
    for p in (a, b):
        if abs(p) >= POSITION_LIMIT:
            rospy.logerr('Target %d exceeds the +-%d limit, aborting.', p, POSITION_LIMIT)
            return
    if args.speed <= 0:
        rospy.logerr('--speed must be positive')
        return

    rate = rospy.Rate(args.rate)
    step = args.speed / args.rate
    current = float(start)

    def send(pos):
        msg = MotorCtrlMessage()
        msg.operationMode = MotorCtrlMessage.MAXON_EPOS4_OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION
        msg.targetPosition = int(round(pos))
        pub.publish(msg)

    def ramp_to(target):
        nonlocal current
        while not rospy.is_shutdown() and abs(target - current) > 1e-6:
            delta = target - current
            current += max(-step, min(step, delta))
            send(current)
            rate.sleep()

    def hold(seconds):
        end = rospy.Time.now() + rospy.Duration(seconds)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            send(current)
            rate.sleep()

    rospy.loginfo('Start %d, A=%d, B=%d, %.0f counts/s, %d cycle(s)', start, a, b, args.speed, args.cycles)
    ramp_to(a)  # get to A slowly first (no-op if already there)
    hold(args.hold)
    for i in range(args.cycles):
        if rospy.is_shutdown():
            break
        rospy.loginfo('Cycle %d/%d: A -> B', i + 1, args.cycles)
        ramp_to(b)
        hold(args.hold)
        rospy.loginfo('Cycle %d/%d: B -> A', i + 1, args.cycles)
        ramp_to(a)
        hold(args.hold)
    rospy.loginfo('Done, holding at %d', int(round(current)))
    # the node keeps re-sending the last command, so the motor holds this position


if __name__ == '__main__':
    main()
