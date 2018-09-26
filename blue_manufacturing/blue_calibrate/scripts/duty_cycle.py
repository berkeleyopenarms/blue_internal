#
#!/usr/bin/env python
from comms import *
import serial
import sys
import time
from math import sin, cos, pi

class Gripper:

    def __init__(self, port, n):
        #  pass in usb port as a string and a board number

        self.port = port
        self.num = n
        s = serial.Serial(port=port, baudrate=COMM_DEFAULT_BAUD_RATE, timeout=0.1)
        addresses = [n]
        client = BLDCControllerClient(s)
        self.client = client

        for address in addresses:
            client.leaveBootloader([address])
            time.sleep(0.2)
            s.reset_input_buffer()

            calibration_obj = client.readCalibration([address])

            client.setZeroAngle([address], [calibration_obj['angle']])
            client.setInvertPhases([address], [calibration_obj['inv']])
            client.setERevsPerMRev([address], [calibration_obj['epm']])
            client.setTorqueConstant([address], [calibration_obj['torque']])
            client.setPositionOffset([address], [calibration_obj['zero']])
            if 'eac_type' in calibration_obj and calibration_obj['eac_type'] == 'int8':
                print('EAC calibration available')
                try:
                    client.writeRegisters([address], [0x1100], [1], [struct.pack('<f', calibration_obj['eac_scale'])])
                    client.writeRegisters([address], [0x1101], [1], [struct.pack('<f', calibration_obj['eac_offset'])])
                    eac_table_len = len(calibration_obj['eac_table'])
                    slice_len = 64
                    for i in range(0, eac_table_len, slice_len):
                        table_slice = calibration_obj['eac_table'][i:i+slice_len]
                        client.writeRegisters([address], [0x1200+i], [len(table_slice)], [struct.pack('<{}b'.format(len(table_slice)), *table_slice)])
                except ProtocolError:
                    print('WARNING: Motor driver board does not support encoder angle compensation, try updating the firmware.')
            client.setCurrentControlMode([address])
            client.writeRegisters([address], [0x1030], [1], [struct.pack('<H', 1000)])
            # print("Motor %d ready: supply voltage=%fV", address, client.getVoltage(address))

            client.writeRegisters([address], [0x2006], [1], [struct.pack('<f', 0.0)])
            client.writeRegisters([address], [0x2000], [1], [struct.pack('<B', 2)]) # Torque control

    def get_state(self):
        data = struct.unpack('<ff', self.client.readRegisters([self.num], [0x3000], [2])[0])
        return data

    def set_cmd(self, cmd):
        self.client.writeRegisters([self.num], [0x2006], [1], [struct.pack('<f', cmd)])
