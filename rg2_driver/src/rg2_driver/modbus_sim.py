#!/usr/bin/env python
"""
Backend for simulating the Modbus TCP connection.  This doesn't actually replicate the TCP traffic;
it only simulates the call/response queries needed by the main gripper controller.
"""

import rospy

from rg2_driver.registers import *
from threading import Thread

class Response:
    def __init__(self):
        self.registers = []

    def isError(self):
        return False

class ModbusSimClient:
    def __init__(self):
        self.registers = [0] * (2**16)
        self.control_thread = Thread(target=self.control_thread_fn)
        self.real_gripper_width_mm = 10.0

    def connect(self):
        self.is_running = True
        self.control_thread.start()
        return True

    def close(self):
        self.is_running = False
        self.control_thread.join()

    def read_input_registers(self, address=0, count=0, unit=0):
        result = Response()
        for i in range(count):
            result.registers.append(self.registers[address + i])
        return result

    def write_registers(self, address=0, values=[], unit=0):
        result = Response()
        for i in range(len(values)):
            self.registers[address + i] = values[i]
        return result

    def control_thread_fn(self):
        """
        Background thread that simulates the actual update rate of the gripper
        This is for very basic simulation only; we don't simulate the force feedback, just the position.  The grip_detected
        will always be false, accordingly
        """
        rate = rospy.Rate(100)
        interval = 1.0/100.0

        while self.is_running:
            rate.sleep()

            target_force = register_to_signed_int(self.registers[RG2_FT_REGISTERS['target_force']]) * 0.1           # N
            target_width = register_to_signed_int(self.registers[RG2_FT_REGISTERS['target_width']]) * 0.1           # mm
            current_width = self.real_gripper_width_mm
            control = self.registers[RG2_FT_REGISTERS['control']] != 0

            if not control:
                # stop moving!
                self.registers[RG2_FT_REGISTERS['gripper_busy']] = 0
            else:
                accel = target_force  # for simplicity assume we're throwing a mass of 1kg around
                d = accel * interval

                # move the grippers in/out as needed
                if current_width < target_width and (current_width + d) >= target_width:
                    current_width = target_width
                    self.registers[RG2_FT_REGISTERS['gripper_busy']] = 0
                    self.registers[RG2_FT_REGISTERS['control']] = 0
                elif current_width > target_width and (current_width - d) <= target_width:
                    current_width = target_width
                    self.registers[RG2_FT_REGISTERS['gripper_busy']] = 0
                    self.registers[RG2_FT_REGISTERS['control']] = 0
                elif current_width < target_width:
                    current_width = current_width + d
                else:
                    current_width = current_width - d

                # update the actual gripper width register
                self.real_gripper_width_mm = current_width
                self.registers[RG2_FT_REGISTERS['actual_gripper_width']] = int(current_width * 10)
