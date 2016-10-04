#!/usr/bin/python

import threading
import time

import robot_inf

class _SensorUpdate(threading.Thread):
    """
        Updates the internal sensor values stored within an instance of Sensor.
    """

    _sensor = None          # Instance of Sensor
    _sensor_sema = None     # The semaphor used to control read/write access
    _interval = None        # The interval between updates
    _stop = False           # The stopping condition

    def __init__(self, sensor, semaphor, interval=robot_inf.SENSOR_UPDATE_WAIT):
        """ Initializes an instance of SensorUpdate with a specified sensor,
            and semaphor.

        :type sensor Sensor:
        :param sensor:
            The instance of Sensor to update.
        :type semaphor Semaphor:
        :param semaphor:
            The semaphor that is used to control read and write access.
        :param interval:
            The interval between sensor updates
        """
        threading.Thread.__init__(self)
        self._sensor = sensor
        self._sensor_sema = semaphor
        self._interval = interval

    def run(self):
        robot = self._sensor.get_robot()

        while not self._stop:
            self._sensor_sema.acquire()         # Acquire Lock

            # Update Buttons
            self._sensor.request_sources.clear()
            self._sensor.btn_prev = self._sensor.btn_down.copy()
            self._sensor.btn_down.clear()

            btns = robot.read_buttons()

            for btn, value in btns.iteritems():
                if value:
                    self._sensor.btn_down[btn] = value

            self._sensor_sema.release()         # Release Lock

            time.sleep(self._interval)


    def stop(self):
        """
            Flags this instance of SensorUpdate to stop. The instance will not
            immediately exit. Instead it will try to exit safely.
        """
        self._stop = True

class Sensor:
    """
        Interface to help manage and interpret sensor data.

        This interface will automatically update the internal sensor value at
        the provided interval. It will also help determine when a button is
        initially pressed or released.

        This interface also provides helpers to safely access the public data
        members. Please uses there methods to access any sensor data member
        outside of its accompanying module.

        Attributes:
            btn_prev: A dictionary of the buttons down on the previous cycle.
            btn_down: A dictionary of the buttons down for the current cycle.
            request_sources:
    """
    _robot = None
    _sensor_sema = None
    _sensor_update = None
    _interval = None


    btn_prev = {}
    btn_down = {}
    request_sources = {}

    def __init__(self, robot, interval=robot_inf.SENSOR_UPDATE_WAIT):
        """ Creates an instance of the sensor interface. This will
            automatically spawn a thread to update the internal sensor data.

        :type robot Robot:
        :param robot:
            The robot to pull the sensor data from
        :param interval:
            The interval between sensor updates
        """
        self._robot = robot
        self._sensor_sema = threading.Semaphore()
        self.start_update()

    def start_update(self):
        """ Spawns a new thread to update the internal sensor data only if
            there is no thread is currently updating its sensor data.
        """
        if self._sensor_update is None:
            self._sensor_update = _SensorUpdate(self,
                                                self._sensor_sema,
                                                self._interval)
            self._sensor_update.setDaemon(True)
            self._sensor_update.start()

    def stop_update(self):
        """ Stops the thread currently updating the internal sensor data.
        """
        if self._sensor_update is not None:
            self._sensor_update.stop()
            self._sensor_update = None

    def get_robot(self):
        """ Gets the robot that provides the sensor data.
        :return:
            The robot providing the sensor data.
        """
        return self._robot

    # TODO: Add helpers to safely access data structures and add other sensor data
    def is_btn_pressed(self, btn, src = "Default", override=False):
        """ Determines if the specified button has been pressed.

            Without overriding this should only be called once per cycle.
        :param btn:
            The button to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if the button is pressed.
        """
        enc_function = "BtnPress"+str(btn)
        rtn = False

        self._sensor_sema.acquire()             # Acquire Lock
        self._add_function_key(enc_function)

        if override or not self.request_sources[enc_function].has_key(src):
            # rtn = not down on previous cycle and is down on current cycle
            rtn = not self.btn_prev.has_key(btn) and self.btn_down.has_key(btn)
            self.request_sources[enc_function][src] = True
        self._sensor_sema.release()             # Release Lock

        return rtn

    def is_btn_released(self, btn, src="Default", override=False):
        """ Determines if the specified button has been released.

            Without overriding this should only be called once per cycle.
        :param btn:
            The button to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if the button is released.
        """
        enc_function = "BtnRelease"+str(btn)
        rtn  = False

        self._sensor_sema.acquire()             # Acquire Lock
        self._add_function_key(enc_function)

        if override or not self.request_sources[enc_function].has_key(src):
            # rtn = down on previous cycle and is not down on current cycle
            rtn = self.btn_prev.has_key(btn) and not self.btn_down.has_key(btn)
            self.request_sources[enc_function][src] = True
        self._sensor_sema.release()             # Release Lock

        return rtn

    def _add_function_key(self, enc_function):
        """ Adds a key to the request_sources if it is not already there.
            The key's value will be initialized to an empty dictionary.

        :param enc_function:
            The string representing a particular function.
        """
        if not self.request_sources.has_key(enc_function):
            self.request_sources[enc_function] = {}