#!/usr/bin/python

import threading
import time

import robot_inf

class _SensorUpdate(threading.Thread):
    _sensor = None
    _sensor_sema = None
    _stop = False

    def __init__(self, sensor, semaphor):
        threading.Thread.__init__(self)
        self._sensor = sensor
        self._sensor_sema = semaphor
        print self._sensor.get_robot().read_buttons()

    def run(self):
        robot = self._sensor.get_robot()

        while not self._stop:
            self._sensor_sema.acquire()

            self._sensor.btn_prev = self._sensor.btn_down.copy()
            self._sensor.btn_down.clear()

            btns = robot.read_buttons()


            print btns
            for btn, value in btns.iteritems():
                if value:
                    print btn
                    self._sensor.btn_down[btn] = value

            self._sensor_sema.release()

            time.sleep(1+robot_inf.SENSOR_UPDATE_WAIT)


    def stop(self):
        self._stop = True

class Sensor:
    _robot = None
    _sensor_sema = None
    _sensor_update = None

    btn_prev = {}
    btn_down = {}

    def __init__(self, robot):
        self._robot = robot
        self._sensor_sema = threading.Semaphore()
        self.start_update()

    def start_update(self):
        if self._sensor_update is None:
            self._sensor_update = _SensorUpdate(self, self._sensor_sema)
            self._sensor_update.setDaemon(True)
            self._sensor_update.start()

    def stop_update(self):
        if self._sensor_update is not None:
            self._sensor_update.stop()

    def get_robot(self):
        return self._robot

    def is_btn_pressed(self, btn):
        rtn = False

        self._sensor_sema.acquire()
        rtn = not self.btn_prev.has_key(btn) and self.btn_down.has_key(btn)
        self._sensor_sema.release()

        return rtn