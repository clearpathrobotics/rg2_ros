#!/usr/bin/env python
"""
Main driver node for the OnRobot RG2-FT gripper
"""

import math
from rg2_driver.registers import *
from rg2_driver.modbus_sim import ModbusSimClient

# ros imports
import rospy
from std_msgs.msg import Bool, Float32, Header
from sensor_msgs.msg import JointState
from rg2_msgs.msg import GripperCommand
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped

# modbus imports
from pymodbus.client.sync import ModbusTcpClient

# threading to deal with background tasks
from threading import Lock, Thread



def command_callback(data, controller):
    controller.command_callback(data)

def zero_callback(data, controller):
    controller.zero_callback(data)

class RG2_FT_Node:
    def __init__(self):
        self.modbus_lock = Lock()
        self.load_rosparams()
        self.sequence_id = 0

        if not self.is_sim:
            rospy.loginfo('Creating PyModBus interface for {0}'.format(self))
            self.client = ModbusTcpClient(self.gripper_ip, port=self.gripper_port)
        else:
            rospy.loginfo('Running in simulation mode')
            self.client = ModbusSimClient()


    def __str__(self):
        if self.is_sim:
            return 'Simulated RG2-F/T'
        else:
            return "RG2-F/T {0}:{1}".format(self.gripper_ip, self.gripper_port)


    def load_rosparams(self):
        """
        Read the ROS params and apply them
        """
        self.gripper_name = rospy.get_param('~name', 'gripper')
        self.gripper_ip = rospy.get_param('~ip_address', '192.168.1.1')
        self.gripper_port = rospy.get_param('~tcp_port', 502)
        self.gripper_address = rospy.get_param('~device_address', 65)

        # convert the rate to an actual Rate object we can use to control timing
        # clamp the rate between 1 and 125Hz, based on the modbus spec
        self.update_rate = rospy.get_param('~update_rate', 10)
        if self.update_rate < 1:
            rospy.logwarn("Update rate should not be lower than 1Hz")
            self.update_rate = 1
        elif self.update_rate > 125:
            rospy.logwarn("Modbus TCP cannot go faster than 125Hz")
            self.update_rate = 125
        self.update_rate = rospy.Rate(self.update_rate)

        self.is_sim = rospy.get_param('~sim', False)

        self.base_link_name = '{0}_base_link'.format(self.gripper_name)
        self.joint_names = [
            '{0}_left_finger_joint_1'.format(self.gripper_name),
            '{0}_left_finger_joint_2'.format(self.gripper_name),
            '{0}_right_finger_joint_1'.format(self.gripper_name),
            '{0}_right_finger_joint_2'.format(self.gripper_name)
        ]
        self.fingertip_names = [
            '{0}_left_fingertip'.format(self.gripper_name),
            '{0}_right_fingertip'.format(self.gripper_name)
        ]


    def connect(self):
        """
        Initialize the modbus TCP connection to the gripper
        """
        self.modbus_lock.acquire()
        if self.client.connect():
            rospy.loginfo('Connected to {0}'.format(self))
        else:
            rospy.logerr('Failed to connect to {0}'.format(self))
        self.modbus_lock.release()


    def disconnect(self):
        """
        Shutdown the connection to the gripper hardware
        """
        rospy.loginfo('Disconnecting from {0}'.format(self))
        self.modbus_lock.acquire()
        self.client.close()
        self.modbus_lock.release()


    def test_connection(self):
        """
        Perform a simple diagnostic check on startup by moving the hand through its full range of motion
        """
        self.modbus_lock.acquire()

        # try reading a value from the gripper to make sure it works
        response = self.client.read_input_registers( address=RG2_FT_REGISTERS['actual_gripper_width'], count=1, unit=self.gripper_address)

        if not response.isError():
            mm = register_to_signed_int(response.registers[0]) * 0.1
            rospy.loginfo('Current gripper width is {0}mm'.format(mm))
        else:
            rospy.logwarn('Failed to read gripper position: {0}'.format(response))

        # try writing position values to move the gripper
        # this is done in 3 steps: fully-open, fully-close, fully-open, ready
        # do this all synchronously
        target_distances = [1000, 0, 1000, 850]
        for d in target_distances:
            response = self.client.write_registers(unit=self.gripper_address,
                address = RG2_FT_REGISTERS['target_force'], values = [100, d, 1])  # force, width, grip

            if not response.isError():
                rospy.loginfo('Moving gripper to position gripper {0}'.format(d))
            else:
                rospy.logwarn('Failed to move gripper: {0}'.format(response))

            rate = rospy.Rate(10)
            is_moving = True
            while is_moving:
                rate.sleep()
                response = self.client.read_input_registers(address=RG2_FT_REGISTERS['gripper_busy'], count=1, unit=self.gripper_address)
                is_moving = not response.isError() and response.registers[0] != 0

        self.modbus_lock.release()


    def start(self):
        """
        Create the subscribers and publishers, start the background worker threads that poll the gripper
        """
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.grip_width_pub = rospy.Publisher('{0}/width'.format(self.gripper_name), Float32, queue_size=1)
        self.grip_moving_pub = rospy.Publisher('{0}/moving'.format(self.gripper_name), Bool, queue_size=1)
        self.left_fingertip_pub = rospy.Publisher('{0}/left_fingertip'.format(self.gripper_name), WrenchStamped, queue_size=1)
        self.right_fingertip_pub = rospy.Publisher('{0}/right_fingertip'.format(self.gripper_name), WrenchStamped, queue_size=1)
        self.grip_pub = rospy.Publisher('{0}/grip'.format(self.gripper_name), Bool, queue_size=1)
        self.zero_pub = rospy.Publisher('{0}/is_zero'.format(self.gripper_name), Bool, queue_size=1)

        joint_state_publisher_thread = Thread(target=self.joint_state_loop_fn)
        joint_state_publisher_thread.start()

        zero_state_publisher_thread = Thread(target=self.zero_state_loop_fn)
        zero_state_publisher_thread.start()

        self.command_sub = rospy.Subscriber('{0}/cmd/move'.format(self.gripper_name), GripperCommand, command_callback, callback_args=self)
        self.zero_sub = rospy.Subscriber('{0}/cmd/zero'.format(self.gripper_name), Bool, zero_callback, callback_args=self)


    def joint_state_loop_fn(self):
        """
        Background thread that:
            1) reads the current width of the gripper and publishes that to the width topic
            2) calculates the current finger joint positions based on the width
            3) publishes the joint state to tf
            4) reads and publishes the FT sensor data from each fingertip
            5) publishes whether or not the fingers are currently moving
            6) publishes whether or not a grip is detected

        Rate is determined by the update_rate parameter
        """

        while not rospy.is_shutdown():
            self.update_rate.sleep()
            self.sequence_id = self.sequence_id+1

            # read the width from the gripper and publish it + do the IK calculates to publish joint states
            # reads 3 consecutive registers: the actual gripper width, the is-moving flag, and the is-gripped flag
            self.modbus_lock.acquire()
            response = self.client.read_input_registers(address=RG2_FT_REGISTERS['actual_gripper_width'], count=3, unit=self.gripper_address)
            self.modbus_lock.release()

            if not response.isError():
                mm = register_to_signed_int(response.registers[0]) * 0.1
                m = mm * 0.001
                moving = response.registers[1] != 0
                gripped = response.registers[2] != 0

                angles = self.do_ik_calculations(mm)

                width = Float32()
                width.data = m
                self.grip_width_pub.publish(width)

                states = JointState()
                states.name = self.joint_names
                states.position = angles
                states.effort = [0.0, 0.0, 0.0, 0.0]
                states.velocity = [0.0, 0.0, 0.0, 0.0]
                states.header = Header()
                states.header.stamp = rospy.Time.now()
                #states.header.seq = self.sequence_id
                self.joint_state_pub.publish(states)

                is_moving = Bool()
                is_moving.data = moving
                self.grip_moving_pub.publish(is_moving)

                is_gripped = Bool()
                is_gripped.data = gripped
                self.grip_pub.publish(is_gripped)

            else:
                rospy.logwarn("Failed to read gripper state: {0}".format(response))

            # Read & publish the left & right FT sensor data
            self.modbus_lock.acquire()
            response = self.client.read_input_registers(address=RG2_FT_REGISTERS['fx_l'], count=6, unit=self.gripper_address)
            self.modbus_lock.release()
            self.publish_finger(self.left_fingertip_pub, response, '{0}_left_fingertip'.format(self.gripper_name))
            self.modbus_lock.acquire()
            response = self.client.read_input_registers(address=RG2_FT_REGISTERS['fx_r'], count=6, unit=self.gripper_address)
            self.modbus_lock.release()
            self.publish_finger(self.right_fingertip_pub, response, '{0}_right_fingertip'.format(self.gripper_name))


    def zero_state_loop_fn(self):
        """
        Publishes whether or not the sensors are zero'd at 1Hz
        """

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()

            self.modbus_lock.acquire()
            response = self.client.read_input_registers(address=RG2_FT_REGISTERS['zero'], count=1, unit=self.gripper_address)
            self.modbus_lock.release()
            if not response.isError():
                zero = response.registers[0] != 0
                is_zero = Bool()
                is_zero.data = zero
                self.zero_pub.publish(is_zero)


    def do_ik_calculations(self, distance_mm):
        """
        Return the angles of the 4 finger joints in order [L1, L2, R1, R2] needed to achieve the provided separation
        between the finger joints.

        This doesn't use the TF and instead makes some assumptions about the gripper's joints that may not be
        100% indicative of reality.  This is really just to make sure the gripper's approximate state is
        visible in rviz.

        We assume the fingertips are always kept parallel, with standard fingertips
        """

        if distance_mm < 0:
            distance_mm = 0

        base_finger_offset = 10.0
        link1_length = 57.0
        link2_thickness = 10.0

        try:
            finger_angle = abs( math.asin((distance_mm/2) / link1_length) )
        except ValueError as err:
            # if we have an encoder problem just publish zeros so we have _something_ in the TF
            finger_angle = 0

        return [finger_angle, -finger_angle, finger_angle, -finger_angle]


    def publish_finger(self, publisher, response, frame):
        """
        Publish a stamped wrench on the provided publisher, given the frame ID and modbus response
        The response must have 6 registers, corresponding to: [Fx, Fy, Fz, Tx, Ty, Tz], all signed 16-bit ints,
        in 0.1N or 0.01Nm, as appropriate for Force or Torque.
        """
        if response.isError():
            rospy.logwarn("Failed to read fingertip sensor data: {0}".format(response))
            return

        fx = register_to_signed_int(response.registers[0]) * 0.1
        fy = register_to_signed_int(response.registers[1]) * 0.1
        fz = register_to_signed_int(response.registers[2]) * 0.1

        tx = register_to_signed_int(response.registers[3]) * 0.01
        ty = register_to_signed_int(response.registers[4]) * 0.01
        tz = register_to_signed_int(response.registers[5]) * 0.01

        msg = WrenchStamped()
        msg.wrench = Wrench()
        msg.wrench.force = Vector3()
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        msg.wrench.torque = Vector3()
        msg.wrench.torque.x = tx
        msg.wrench.torque.y = ty
        msg.wrench.torque.z = tz
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.header.seq = self.sequence_id

        publisher.publish(msg)


    def command_callback(self, data):
        """
        Send commands to the gripper to open/close to the specified distance with the desired force.
        Can also stop the gripper if the stop flag is True
        """
        if data.stop:
            self.modbus_lock.acquire()
            response = self.client.write_registers(address=RG2_FT_REGISTERS['control'], values=[0], unit=self.gripper_address)
            self.modbus_lock.release()

            if response.isError():
                rospy.logwarn("Failed to stop gripper: {0}".format(response))

        else:
            force = int(data.force_n * 10)
            if force > 400:
                rospy.logwarn('Desired force is too high')
                force = 400
            elif force < 0:
                rospy.logwarn('Desired force is too low')
                force = 0

            width = int(data.width_mm * 10)
            if width > 1000:
                rospy.logwarn('Desired width is too large')
                width = 1000
            elif width < 0:
                rospy.logwarn('Desired width is too small')
                width = 0

            self.modbus_lock.acquire()
            response = self.client.write_registers(unit=self.gripper_address,
                address = RG2_FT_REGISTERS['target_force'], values = [force, width, 1])
            self.modbus_lock.release()


    def zero_callback(self, data):
        """
        Write 1 to the zero register if we are zeroing the sensors, otherwise write 0 to the zero register
        Yes, that's terribly confusing, and I'm sorry you had to read that.  I didn't name the registers or
        come up with the modbus protocol
        """
        is_zero = data.data
        self.modbus_lock.acquire()
        response = self.client.write_registers(unit=self.gripper_address,
            address = RG2_FT_REGISTERS['zero'], values = [0 if not is_zero else 1])
        self.modbus_lock.release()
