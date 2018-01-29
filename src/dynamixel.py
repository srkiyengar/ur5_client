# module: dynamixel.py
# Rajan Iyengar Neurorobotics lab, UWaterloo
# 25 June 2016
# Second version for a multi-threaded control of Reflex Palm for precision grip
# Reflex is right hand robotics and the dynamixel control is through USBtoSerial (USB2Dynamixel_Device) which is builtin
# This module deals with calls to the individual servos
# The module is adapted from the code written as in the acknowledgement below:
#
#-----------------------Acknowledgement -----------------------------------
# For the code that was obtained from http://www.hizook.com/files/users/3/lib_robotis.py_.txt:
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Travis Deyle, Advait Jain & Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
#
#---- End of Acknowledgement

import serial
import thread
import string



class USB2Dynamixel_Device():
    ''' Class that manages serial port contention between servos on same bus
    '''
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57600 ):
        try:
            self.dev_name = string.atoi( dev_name ) # stores the serial port as 0-based integer for Windows
        except:
            self.dev_name = dev_name # stores it as a /dev-mapped string for Linux / Mac

        self.mutex = thread.allocate_lock()
        self.servo_dev = None

        self.acq_mutex()
        self._open_serial( baudrate )
        self.rel_mutex()

    def acq_mutex(self):
        self.mutex.acquire()

    def rel_mutex(self):
        self.mutex.release()

    def write_serial(self, msg):
        # It is up to the caller to acquire / release mutex
        self.servo_dev.write( msg )

    def read_serial(self, nBytes=1):
        # It is up to the caller to acquire / release mutex
        rep = self.servo_dev.read( nBytes )
        return rep

    def _open_serial(self, baudrate):

        # The user needs to be added to group "dialout" for permissions to work.


        try:
            self.servo_dev = serial.Serial(self.dev_name, baudrate, timeout=1.0)
            # Closing the device first seems to prevent "Access Denied" errors on WinXP
            # (Conversations with Brian Wu @ MIT on 6/23/2010)
            self.servo_dev.close()
            self.servo_dev.setParity('N')
            self.servo_dev.setStopbits(1)
            self.servo_dev.open()

            self.servo_dev.flushOutput()
            self.servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            print e.args
            raise RuntimeError('lib_robotis: Serial port issue!\n')
        if(self.servo_dev == None):
            raise RuntimeError('lib_robotis: Serial port not found!\n')


class Robotis_Servo():
    ''' Class to use a robotis MX-28
    '''
    def __init__(self, USB2Dynamixel, servo_id, series = None ):
        ''' USB2Dynamixel - USB2Dynamixel_Device object to handle serial port.
                            Handles threadsafe operation for multiple servos
            servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
        '''



        # Error Checking
        if USB2Dynamixel is None:
            raise RuntimeError('lib_robotis: Robotis Servo requires USB2Dynamixel!\n')
        else:
            self.dyn = USB2Dynamixel

        # ID exists on bus?
        self.servo_id = servo_id
        try:
            if self.read_servo_id() != servo_id:
                print 'The servo-id address is not what it should be - You should never hit this line'
        except:
            raise RuntimeError('lib_robotis: Error encountered.  Could not find ID (%d) on bus (%s), or USB2Dynamixel \
                3-way switch in wrong position.\n' % ( servo_id, self.dyn.dev_name ))

        # Set Return Delay time - Used to determine when next status can be requested
        data = self.read_address( 0x05, 1)
        self.return_delay = data[0] * 2e-6



    #Modified By Rajan - Reflex operates dynamixel in Multi-turn mode

    def is_moving(self):
        ''' returns True if servo is moving.
        '''
        data = self.read_address( 0x2e, 1 )
        return data[0] != 0



    def read_servo_id(self):
        '''read the servo-id from location address 0x03 one byte
        '''
        data = self.read_address(0x03,1)
        servo_id = data[0]
        return servo_id



    def read_current_position(self):
        ''' returns current position
        '''
        data = self.read_address(0x24,2)
        current_position = data[0] + data[1] * 256
        return current_position


    def set_goal_position(self, n):
        ''' move to n  (-28672 to 28672 for multi-turn corresponding to 7 turns (hi resolution)
        '''

        hi,lo = n / 256, n % 256
        val = self.write_address(0x1e, [lo,hi])
        return val

    def get_goal_position(self):
        ''' get goal position
        '''
        data = self.read_address(0x1e,2)
        current_goal_position = data[0] + data[1] * 256
        return current_goal_position


    def read_speed(self):
        data = self.read_address(0x20,2)
        speed = data[0] + data[1] * 256
        return speed

    def set_speed(self,n):
        ''' 0 - Max speed. n should be less than 1023
        '''

        hi,lo = n / 256, n % 256
        if (hi>3):
            hi = 3
        return self.write_address(0x20, [lo,hi])

    def read_present_speed(self):  # I think this is 0 unless servo is moving
        data = self.read_address(0x26,2)
        present_speed = data[0] + data[1] * 256
        return present_speed

    def read_multi_turn_offset(self):
        data = self.read_address(0x14,2)
        offset = data[0] + data[1] * 256
        return offset

    def read_resolution_divider(self):
        '''read tfrom location address 0x16 one byte
        '''
        data = self.read_address(0x16,1)
        resdiv = data[0]
        return resdiv

    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = ( ~chksum ) % 256
        return chksum

    def read_address(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [ 0x02, address, nBytes ]
        return self.send_instruction( msg, self.servo_id )

    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg = [ 0x03, address ] + data
        return self.send_instruction( msg, self.servo_id )

    def send_instruction(self, instruction, id):
        msg = [ id, len(instruction) + 1 ] + instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum( msg )
        msg = [ 0xff, 0xff ] + msg + [chksum]

        self.dyn.acq_mutex()
        try:
            self.send_serial( msg )
            data, err = self.receive_reply()
        except:
            self.dyn.rel_mutex()
            raise
        self.dyn.rel_mutex()

        if err != 0:
            self.process_err( err )

        return data

    def process_err( self, err ):
        raise RuntimeError('lib_robotis: An error occurred: %d\n' % err)

    def receive_reply(self):
        start = self.dyn.read_serial( 2 )
        if start != '\xff\xff':
            raise RuntimeError('lib_robotis: Failed to receive start bytes\n')
        servo_id = self.dyn.read_serial( 1 )
        if ord(servo_id) != self.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received: %d\n' % ord(servo_id))
        data_len = self.dyn.read_serial( 1 )
        err = self.dyn.read_serial( 1 )
        data = self.dyn.read_serial( ord(data_len) - 2 )
        checksum = self.dyn.read_serial( 1 ) # I'm not going to check...
        return [ord(v) for v in data], ord(err)


    def send_serial(self, msg):
        """ sends the command to the servo
        """
        out = ''
        for m in msg:
            out += chr(m)
        self.dyn.write_serial( out )