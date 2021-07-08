#!/usr/bin/env python
"""
Dumping ground for all modbus register addresses and status codes
"""

MODBUS_FN_CODES = {
    'read_holding_reg'      : 0x03,
    'write_single_reg'      : 0x06,
    'write_multi_reg'       : 0x10,
    'rw_multi_reg'          : 0x17,
}

MODBUS_REGISTERS = {
    'target_force'                  : 0x0000,   # w 0.1%, 0-1000
    'target_diameter'               : 0x0001,   # w 0.1mm
    'grip_type'                     : 0x0002,   # w 0=external, 1=internal
    'control'                       : 0x0003,   # w see below
    'status'                        : 0x0100,   # r bit-field, see below
    'raw_diameter'                  : 0x0101,   # r 0.1mm
    'diameter_w_fintertip_offset'   : 0x0102,   # r 0.1mm signed 2s-compliment
    'force_applied'                 : 0x0103,   # r 0.1%
    'finger_angle'                  : 0x0105,   # r 0.001 rad, 0 = fully-open
    'finger_length'                 : 0x010e,   # r 0.1mm
    'finger_position'               : 0x0110,   # r finger mount type: 1, 2, or 3
    'fingertip_offset'              : 0x0111,   # r 0.01mm
    'min_diameter'                  : 0x0201,   # r 0.1mm
    'max_diameter'                  : 0x0202,   # r 0.1mm
    'set_finger_length'             : 0x0401,   # rw 0.1mm
    'set_finger_position'           : 0x0403,   # rw set finger mount type 1, 2, or 3
    'set_fingertip_offset'          : 0x0404    # rw 0.01mm
}

RG2_FT_REGISTERS = {
    'zero'                  : 0x0000,   # rw zero force & torque sensors to cancel offset: 0x0000: unzero, 0x0001: zero
    'target_force'          : 0x0002,   # w 0.1N, 0-400
    'target_width'          : 0x0003,   # w 0.01mm, 0-1000
    'control'               : 0x0004,   # w 0x0000: stop, 0x0001: grip
    'proximity_offset_l'    : 0x0005,   # rw 0.1mm, subtracted from raw signal
    'proximity_offset_r'    : 0x0006,   # rw same as above
    'status_l'              : 0x0101,   # r 0x0000 when no error in left finger
    'fx_l'                  : 0x0103,   # r 0.1N force along the X axis of the left finger, signed
    'fy_l'                  : 0x0104,   # r 0.1N force along the Y axis of the left finger, signed
    'fz_l'                  : 0x0105,   # r 0.1N force along the Z axis of the left finger, signed
    'tx_l'                  : 0x0106,   # r 0.01Nm torque around X axis of left finger, signed
    'ty_l'                  : 0x0107,   # r 0.01Nm torque around Y axis of left finger, signed
    'tz_l'                  : 0x0108,   # r 0.01Nm torque around Z axis of left finger, signed
    'status_r'              : 0x010a,   # r same as above, but for right finger
    'fx_r'                  : 0x010c,   # r same as above, but for right finger
    'fy_r'                  : 0x010d,   # r same as above, but for right finger
    'fz_r'                  : 0x010e,   # r same as above, but for right finger
    'tx_r'                  : 0x010f,   # r same as above, but for right finger
    'ty_r'                  : 0x0110,   # r same as above, but for right finger
    'tz_r'                  : 0x0111,   # r same as above, but for right finger
    'proximity_status_l'    : 0x0112,   # r 0x0000 when no error with left proximity sensor
    'proximity_value_l'     : 0x0113,   # r 0.1mm distance to left proximity sensor, signed
    'proximity_status_r'    : 0x0115,   # r as above, but for right proximity sensor
    'proximity_value_r'     : 0x0116,   # r as above, but for right proximity sensor
    'actual_gripper_width'  : 0x0118,   # r 0.1mm current width between grippers
    'gripper_busy'          : 0x0119,   # r 1 when moving, 0 when still. will only accept new commands when 0
    'grip_detected'         : 0x011a    # r 1 when an internal or external grip is detected, otherwise 0
}

GRIP_TYPES = {
    'grip'      : 0x0001,   # start the motion with preset force & diameter
    'move'      : 0x0002,   # start the motion w/o applying force
    'stop'      : 0x0004    # stop current motion
}

# bit-masks for the status response
STATUS_CODES = {
    'busy'                  : 1<<0,
    'grip_detected'         : 1<<1,
    'force_grip_detected'   : 1<<2,
    'calibration'           : 1<<3
    # bits 4-16 unused
}

def register_to_signed_int(x, base=16):
    """
    Modbus uses 16-bit integers, which can sometimes be treated as a signed 15-bit number.
    This handles converting a signed 16-bit number to a proper int
    Optionally can also work on other bases, but that's not normally needed
    """
    if x & (1 << (base-1)):
        y = x - (1 << base)
    else:
        y = x
    return y
