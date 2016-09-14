#!/usr/bin/python

import threading

import serial_inf

# Running this on the robot will let you input raw commands to the robot
# and see the raw input from the robot. The main use of this is to test
# the serial interface.

stop = False

class Reader(threading.Thread):
    _robot = None

    def __init__(self, thread_id, name, counter, robot):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.name = name
        self.counter = counter
        self._robot = robot

    def run(self):
        while not stop:
            data = self._robot.read_data(1)
            if len(data) > 0:
                print "Read data: "+data


class Writer(threading.Thread):
    _robot = None

    def __init__(self, thread_id, name, counter, robot):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.name = name
        self.counter = counter
        self._robot = robot

    def run(self):
        global stop
        while not stop:
            usr_input = raw_input('Enter cmd or exit:')
            if usr_input == "exit":
                stop = True
            else:
                self._robot.send_command(usr_input)

def test():
    r = serial_inf.SerialConn("/dev/ttyUSB0", 115200, timeout=5)
    read = Reader(1, "Reader", 1, r)
    write = Writer(2, "Writer", 1, r)

    read.start()
    write.start()

if __name__ == '__main__':
    test()