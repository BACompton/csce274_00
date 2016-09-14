#!/usr/bin/python

import threading

import robot_inf
import serial_inf

# FOR TESTING PURPOSES ONLY!
# DO NOT INCLUDE IN UPLOADED PROJECT FILE!

# This assumes two serial ports are connected. This can be used with virtual
# serial ports.


# Emulates the Robots response
class EmulateResponse(threading.Thread):
    button_val = 0
    _serial_conn = None
    _stop = None

    def __init__(self, port,  thread_id, name, counter):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.name = name
        self.counter = counter
        self._serial_conn = serial_inf.SerialConn(port, robot_inf._BAUD_RATE)
        self._stop = False
        self.button_val = 0

    def run(self):
        while not self._stop:
            # Try to read the opcode
            opcode = self._serial_conn.read_data(1)

            # Nothing was sent
            if len(opcode) == 0:
                continue

            # Sensor command sent
            if opcode == str(chr(142)):
                # Read the packet id
                packet = self._serial_conn.read_data(1)

                # Emulates Button Packet
                if packet == str(chr(18)):
                    self._serial_conn.send_command(str(self.button_val))
                    self.button_val = EmulateResponse.button_val
            # Drive command sent
            if opcode == str(chr(137)):
                self._serial_conn.read_data(4)

    def stop(self):
        self._stop = True

    # Emulates pressing a button on the robot
    def press_button(self, button):
        self.button_val = self.button_val | button


# Used to artificially press button and fake sensor data
class EmulateController(threading.Thread):
    _stop = None
    _emulator = None

    def __init__(self, thread_id, name, counter, emulator):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.name = name
        self.counter = counter
        self._stop = False
        self._emulator = emulator

    def run(self):
        while not self._stop:
            input = raw_input("Enter clean or exit:")

            # Exit Emulation
            if input == "exit":
                self._stop = True
                self._emulator.stop()
            # Press the clean button
            elif input == "clean":
                self._emulator.press_button(robot_inf.Button.CLEAN)
