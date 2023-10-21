#!/usr/bin/env python3
# experiemnts and tests with occupancy grids


class Device_Mock(object):
    """
    Mock Device class to emulate a base EV3 device
    """
    def __init__(self):
        x = 10
        x = x + 10

class Sensor_Mock(Device_Mock):
    """
    Mock of the EV3 sensor class provides a uniform interface for using most of the
    sensors available for the EV3.
    """

    SYSTEM_CLASS_NAME = 'lego-sensor'
    SYSTEM_DEVICE_NAME_CONVENTION = 'sensor*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        self._address = "0xcddeeff"
        self._command = "Cmd1"
        self._commands = ["Cmd1","Cmd2","Cmd3"]
        self._decimals = 2
        self._driver_name = "Test device"
        self._mode = "Mock Mode"
        self._modes = ["Mode1","Mode2","Mode3"]
        self._num_values = 2
        self._units = None
        self._value = [1.0, 2.0, 3.0, 4.0, None, None, None, None]

        self._bin_data_format = None
        self._bin_data_size = None
        self._bin_data = None
        self._mode_scale = {}

    @property
    def address(self):
        """
        Returns the name of the sensor device/driver. See the list of [supported
        sensors] for a complete list of drivers.
        """
        value = self._address
        return value

    @property
    def commands(self):
        """
        Returns the name of the sensor device/driver. See the list of [supported
        sensors] for a complete list of drivers.
        """
        value = self._commands
        return value

    @property
    def decimals(self):
        """
        Returns the name of the sensor device/driver. See the list of [supported
        sensors] for a complete list of drivers.
        """
        value = self._decimals
        return value

    @property
    def driver_name(self):
        """
        Returns the name of the sensor device/driver. See the list of [supported
        sensors] for a complete list of drivers.
        """
        value = self._driver_name
        return value
    
    @property
    def mode(self):
        """
        Returns the current mode. Writing one of the values returned by ``modes``
        sets the sensor to that mode.
        """
        value = self._mode
        return value

    @property
    def modes(self):
        """
        Returns a list of the valid modes for the sensor.
        """
        value = self._modes
        return value

    @property
    def num_values(self):
        """
        Returns the number of ``value<N>`` attributes that will return a valid value
        for the current mode.
        """
        value = self._num_values
        return value

    @property
    def units(self):
        """
        Returns the units of the measured value for the current mode. May return
        empty string
        """
        value = self._units
        return value

    def value(self, n=0):
        """
        Returns the value or values measured by the sensor. Check num_values to
        see how many values there are. Values with N >= num_values will return
        an error. The values are fixed point numbers, so check decimals to see
        if you need to divide to get the actual value.
        """
        n = int(n)

        value = self._value[n]
        return value

class UltrasonicSensor_Mock(Sensor_Mock):
    """
    LEGO EV3 ultrasonic sensor.
    """

    def __init__(self, address=None, name_pattern=None, name_exact=False, **kwargs):
        super(UltrasonicSensor_Mock, self).__init__(address,
                                               name_pattern,
                                               name_exact,
                                               driver_name="Mock Ultrasonic sensor",
                                               **kwargs)
    @property
    def distance_centimeters(self):
        return 205.1

class MotorSet_Mock(object):
    def __init__(self, motor_specs, desc=None):
        """
        motor_specs is a dictionary such as
        {
            OUTPUT_A : LargeMotor,
            OUTPUT_C : LargeMotor,
        }
        """


class MoveTank_Mock(MotorSet_Mock):
    """
    Controls a pair of motors simultaneously, via individual speed setpoints for each motor.

    Example:

    .. code:: python

        tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
        # drive in a turn for 10 rotations of the outer motor
        tank_drive.on_for_rotations(50, 75, 10)
    """
    def __init__(self, left_motor_port, right_motor_port, desc=None, motor_class=None):
        self._left_speed = 0
        self._right_speed = 0

    def on(self, left_speed, right_speed):
        """
        Start rotating the motors according to ``left_speed`` and ``right_speed`` forever.
        Speeds can be percentages or any SpeedValue implementation.
        """
        self._left_speed = left_speed
        self._right_speed = right_speed

    def stop(self):
        self._left_speed = 0
        self._right_speed = 0

class Button_Mock():
    def __init__(self):
        self.buttons_pressed = False
        
    _buttons = {
        'up': {
            'name': "Up",
            'value': 103
        },
        'down': {
            'name': "Down",
            'value': 108
        },
        'left': {
            'name': "Left",
            'value': 105
        },
        'right': {
            'name': "Right",
            'value': 106
        },
        'enter': {
            'name': "Enter",
            'value': 28
        },
        'backspace': {
            'name': "Backspace",
            'value': 14
        }
        }
    
    def any(self):
        """
        Checks if any button is pressed.
        """
        return bool(self.buttons_pressed)