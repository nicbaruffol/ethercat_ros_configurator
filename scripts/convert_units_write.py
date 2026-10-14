import rospy
import numpy as np
from threading import Lock

from control_utils.msg import VectorStamped, ScalarStamped
from ethercat_motor_msgs.msg import MotorCtrlMessage, MotorStatusMessage

class ConvertUnitsWriteNode:
    """
    Velocity setpoint v_sp (m/s, one per control step Ts) -> velocity command to the drive (rpm, every drive period).
    A new v_sp is not sent as a jump but as a ramp over Ts, in equal steps per drive period: the commanded acceleration
    is spread evenly over the control step, as the controller's and the Kalman filter's model assume. With Ts = drive
    period (200 Hz control) the first step is the whole step, exactly as before.
    """
    def __init__(self):
        rospy.init_node(name='convert_units_write', anonymous=True)
        self.init_topics()
        self.init_publishers()
        self.init_variables()

    def init_topics(self):
        self.motor_command_topic = rospy.get_param('Maxon_Motor/command')
        self.setpoint_topic = 'v_sp'
        self.Ts = rospy.get_param('/Ts')        # s, a new v_sp every Ts
        self.drive_period = 0.005               # s, the EtherCAT cycle (time_step in config/setup.yaml)

    def init_publishers(self):
        self.motor_command_pub = rospy.Publisher(self.motor_command_topic, MotorCtrlMessage, queue_size=1)
        self.command_msg = MotorCtrlMessage(operationMode=MotorCtrlMessage.MAXON_EPOS4_OPERATION_MODE_CYCLIC_SYNCHRONOUS_VELOCITY)

    def init_variables(self):
        self.v_sp = 0                           # rpm, the command sent last
        self.v_start = 0.0                      # m/s: the ramp from v_start to v_target, started at t_start
        self.v_target = 0.0
        self.t_start = None                     # None until the first v_sp: nothing is sent before (e.g. during homing)
        self.time = 0
        self.lock = Lock()                      # setpoint callback and timer run in different threads

    def m_to_inc(self, m):
        """
        4096 [inc/rev] / 0.005 [m/rev] = 819200 [inc/m]
        """
        return int(m * 819200.0)

    def mps_to_rpm(self, mps):
        """
        0.005 [m/rev] / 60 [s/min] = 1/12000 [m/s / rpm]
        """
        return int(mps * 12000)


    def current(self):
        """m/s, where the ramp is now. One drive period ahead, so the first step goes out at once and the target is
        reached within Ts"""
        steps = np.floor((rospy.get_time() - self.t_start) / self.drive_period + 1e-6) + 1
        return self.v_start + min(steps * self.drive_period / self.Ts, 1.0) * (self.v_target - self.v_start)

    def callback(self, msg):
        """A new setpoint: ramp to it over Ts, starting from where the previous ramp is now"""
        with self.lock:
            self.v_start = self.current() if self.t_start is not None else msg.scalar
            self.v_target = msg.scalar
            self.t_start = rospy.get_time()
            self.time = msg.header.stamp
            self.publish()

    def timer_callback(self, event):
        """Every drive period: the next step of the ramp (or the target again once it is reached)"""
        with self.lock:
            if self.t_start is not None:
                self.publish()

    def publish(self):
        self.v_sp = self.mps_to_rpm(self.current())
        self.command_msg.header.stamp = self.time
        self.command_msg.targetVelocity = self.v_sp

        self.motor_command_pub.publish(self.command_msg)

    def run(self):
        rospy.Subscriber(self.setpoint_topic, ScalarStamped, self.callback, queue_size=1)
        rospy.Timer(rospy.Duration(self.drive_period), self.timer_callback)

        rospy.spin()

if __name__ == '__main__':

    node = ConvertUnitsWriteNode()
    node.run()
