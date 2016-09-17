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

    def run(self):
        robot = self._sensor.get_robot()

        while not self._stop:
            self._sensor_sema.acquire()

            self._sensor.request_sources.clear()
            self._sensor.btn_prev = self._sensor.btn_down.copy()
            self._sensor.btn_down.clear()

            btns = robot.read_buttons()

            for btn, value in btns.iteritems():
                if value:
                    self._sensor.btn_down[btn] = value

            self._sensor_sema.release()

            time.sleep(robot_inf.SENSOR_UPDATE_WAIT)


    def stop(self):
        self._stop = True

class Sensor:
    _robot = None
    _sensor_sema = None
    _sensor_update = None

    btn_prev = {}
    btn_down = {}
    request_sources = {}

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

    def is_btn_pressed(self, btn, src = "Default", override=False):
        enc_function = "BtnPress"+str(btn)
        rtn = False

        self._sensor_sema.acquire()
        self._add_function_key(enc_function)

        if not self.request_sources[enc_function].has_key(src):
            rtn = not self.btn_prev.has_key(btn) and self.btn_down.has_key(btn)
            self.request_sources[enc_function][src] = True
        self._sensor_sema.release()

        return rtn

    def is_btn_released(self, btn, src="Default", override=False):
        enc_function = "BtnRelease"+str(btn)
        rtn  = False

        self._sensor_sema.acquire()
        self._add_function_key(enc_function)

        if not self.request_sources[enc_function].has_key(src):
            rtn = self.btn_prev.has_key(btn) and not self.btn_down.has_key(btn)
            self.request_sources[enc_function][src] = True
        self._sensor_sema.release()

        return rtn

    def _add_function_key(self, enc_function):
        if not self.request_sources.has_key(enc_function):
            self.request_sources[enc_function] = {}