#!/usr/bin/env python3
# Thisprogram runs on host computer and connects to an RPyC server running on th EV3 device

#  py -3 -m venv .venv
#  .venv\Scripts\activate
#  python -m pip install --upgrade pip
#  pip install python-ev3dev2
# on the ev3 call
#  sudo apt install python3-paho-mqtt

test_mode_enabled = False
#import os
import sys
import time
import platform
import json
from enum import Enum
import paho.mqtt.client as mqtt

#from ev3dev2.sensor.lego import Sensor, ColorSensor, TouchSensor, LightSensor
from ev3dev2.sensor import INPUT_4  #INPUT_1, INPUT_2, INPUT_3,
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.button import Button
#from ev3dev2.motor import MediumMotor, OUTPUT_D
from ev3dev2.sensor.lego import Sensor
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C

if test_mode_enabled:
    import ev3devemulator
    Sensor = ev3devemulator.Sensor_Mock
    UltrasonicSensor = ev3devemulator.UltrasonicSensor_Mock
    MoveTank = ev3devemulator.MoveTank_Mock
    Button = ev3devemulator.Button_Mock

# TODO set up text stubs for testing without Robot HW
# import module

# def replacement_func():
#     return 4 # chosen by fair dice roll

# module.func = replacement_func

# # run unit tests here
# Now, whenever code in module calls func(), then it will actually call back out to your replacement_func().
# import module

# def replacement_func():
#     return 4 # chosen by fair dice roll

# module.func = replacement_func

# # run unit tests here
# Now, whenever code in module calls func(), then it will actually call back out to your replacement_func().


# state constants
ON = True
OFF = False

outscreen = sys.stderr

# states
class State(Enum):
    """Robot state"""
    EXIT = -1
    STARTUP = 0
    IDLE = 1
    ULTRA_TEST = 2
    ACCEL_TEST = 3
    EXPLORER = 4
    COMPASS_CAL = 5
    LOST = 6



class EV3Controller:
    """Class to control an EV3 mobile wheeled robot"""

    def __init__(self):
        self.state = State.STARTUP
        
        self.position = [0.0, 0.0]
        
        # compass device
        self.compass = Sensor(INPUT_4)
        self.compass_cal_start_time = None
        
        # Number of seconds to run the comapss calibration
        self.compass_cal_duration = 60.0
        
        self.report_sensor(self.compass)
        self.ultra = UltrasonicSensor('in3')
        self.report_sensor(self.ultra)

        self.mwr = MoveTank(OUTPUT_B, OUTPUT_C)

        self.btn = Button()

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        if test_mode_enabled:
            self.client.connect("localhost", 1883,60)
        else:
            self.client.connect("192.168.2.81", 1883,60)
        self.client.loop_start()
        

        
        self.control_loop()

       

    def on_connect(self, client, userdata, flags, rc):
        """Set up MQTT subscriptions upon connect event"""
        self.client.subscribe("ev3/control/stop",0)
        self.client.subscribe("ev3/control/ultra_test",0)
        self.client.subscribe("ev3/control/accel_test",0)
        self.client.subscribe("ev3/control/terminate",0)
        self.client.subscribe("ev3/control/cal_compass",0)

        self.client.message_callback_add("ev3/control/stop", self.on_stop_message)
        self.client.message_callback_add("ev3/control/ultra_test", self.on_ultra_test_message)
        self.client.message_callback_add("ev3/control/terminate", self.on_terminate_message)
        self.client.message_callback_add("ev3/control/cal_compass", self.on_compass_calibarte_message)


        self.state_change(State.IDLE)
        self.client.publish("ev3/status", "EV3 connected")

    def on_terminate_message(self, the_client, user_data, msg):
        """
        Message handler for a terminate messgae"""
        print(msg.topic, msg.payload.decode("utf-8"))

        if msg.topic == "ev3/control/terminate":
            decode_msg = msg.payload.decode("utf-8")
            u_msg = json.loads(decode_msg)
            # indicate update of data for GUI
            #draw_figure(window['-IMAGE-'], your_matplotlib_code(robot.get_map_grid()))
            self.exit()
    
    def on_stop_message(self, the_client, user_data, msg):
        """Message handler for a stop messgae"""
        print(msg.topic, msg.payload.decode("utf-8"))

        if msg.topic == "ev3/control/stop":
            decode_msg = msg.payload.decode("utf-8")
            u_msg = json.loads(decode_msg)
            # indicate update of data for GUI
            #draw_figure(window['-IMAGE-'], your_matplotlib_code(robot.get_map_grid()))
            self.stop_wheels()
            self.state_change(State.IDLE)
    
    def on_ultra_test_message(self, the_client, user_data, msg):
        """Message handler for a stop messgae"""
        print(msg.topic, msg.payload.decode("utf-8"))

        if msg.topic == "ev3/control/ultra_test":
            decode_msg = msg.payload.decode("utf-8")
            u_msg = json.loads(decode_msg)
            # indicate update of data for GUI
            #draw_figure(window['-IMAGE-'], your_matplotlib_code(robot.get_map_grid()))
        self.start_ultra_test()


    def on_accel_test_message(self, the_client, user_data, msg):
        """Message handler for a stop messgae"""
        print(msg.topic, msg.payload.decode("utf-8"))

        if msg.topic == "ev3/control/ultra_test":
            decode_msg = msg.payload.decode("utf-8")
            u_msg = json.loads(decode_msg)
            self.state_change(State.ACCEL_TEST)
            # indicate update of data for GUI
            #draw_figure(window['-IMAGE-'], your_matplotlib_code(robot.get_map_grid()))

    def on_compass_calibarte_message(self, the_client, user_data, msg):
        """Message handler to run a compass calibration"""
        print(msg.topic, msg.payload.decode("utf-8"))

        if msg.topic == "ev3/control/cal_compass":
            decode_msg = msg.payload.decode("utf-8")
            u_msg = json.loads(decode_msg)
            self.start_compass_calibrate()
            # indicate update of data for GUI
            #draw_figure(window['-IMAGE-'], your_matplotlib_code(robot.get_map_grid()))


    def state_change(self, new_state=State.IDLE):
        print("[%5.5f] State change from %s to %s" % (time.time(),
                                                            self.state,
                                                            new_state,
                                                            ),file=outscreen)
        self.client.publish("ev3/status/state_change", str([time.time(), 
                                                        self.state, 
                                                        new_state]) )
        self.state = new_state

    def control_loop(self):
        """Robot control handler"""
        self.cycle_count = 0
        # loop thru the tste machine until State.EXIT is encountered
        while self.state != State.EXIT:
            if self.btn.any():    # Stop program by pressing any button
                self.stop_all()
                self.state_change(State.EXIT)

            if self.state == State.STARTUP:
                # Do nothing - SHOULD 
                self.cycle_count += 1
            elif self.state == State.IDLE:
                self.cycle_count += 1
                self.heading_update()
            elif self.state == State.ULTRA_TEST:
                self.ultra_test()
                self.cycle_count += 1
            elif self.state == State.COMPASS_CAL:
                self.compass_calibrate()
            elif self.state == State.ACCEL_TEST:
                self.cycle_count += 1
            elif self.state == State.EXPLORER:
                self.cycle_count += 1
            elif self.state == State.LOST:
                self.cycle_count += 1
            else:
                print("WARNING state is UNKNOWN")

            time.sleep(0.5)
                
    
    def exit(self):
        """Terminate robot controller"""
        self.mwr.stop()
        self.client.publish("ev3/state","Terminating EV3..." )
        self.state_change(State.EXIT)
        self.client.disconnect()
        print('End program',file=outscreen)

    def report_sensor(self,sens):
        """Report sensor information and parameters"""
        print('=================================================' ,file=outscreen)
        print('Sensor name: ' + sens.driver_name ,file=outscreen)
        print('Sensor port: ' + sens.address ,file=outscreen)
        print('Sensor commands: ', sens.commands ,file=outscreen)
        print('Sensor modes: ', sens.modes ,file=outscreen)
        print('Sensor current mode: ', sens.mode ,file=outscreen)
        print('Sensor values: ', sens.num_values ,file=outscreen)
        print('Sensor decimals: ', sens.decimals ,file=outscreen)
        print('=================================================' ,file=outscreen)

    def read_accel(self,sens):
        """Ready raw accelerometer data and return as x, y, z coord values"""
        x_msb, y_msb, z_msb, x_lsb, y_lsb, z_lsb = sens.bin_data("<6b")

        x = (x_msb << 2) | (x_lsb >> 6) # no idea if this is right, see footnote in docs
        y = (y_msb << 2) | (y_lsb >> 6)
        z1 = (z_msb << 2) | (z_lsb >> 6)
        z2 = (z_msb << 2) | (z_lsb & 0x03)
        #MSB << 2 + LSB & 0x03
        return  [x,y,z1]
    
    def stop_all(self):
        self.stop_wheels()
        self.state = State.IDLE
    
    def stop_wheels(self):
        """Stop all wheel motors"""
        self.mwr.stop()


    def start_compass_calibrate(self):
        """Enter the ultra test test conduct a test to generate """

        self.stop_wheels()
        self.state_change(State.COMPASS_CAL)
        # start rotating on the spot
        self.compass_cal_start_time = time.time()
        self.compass.command = "BEGIN-CAL"
        self.client.publish("ev3/status/compass_calibration_start", str([time.time()]) )

    def compass_calibrate(self):
        elapsed_time = time.time() - self.compass_cal_start_time

        if elapsed_time > self.compass_cal_duration:
            self.compass.command = "END-CAL"
            self.client.publish("ev3/status/compass_calibration_complete", str([time.time()]) )
            self.state_change(State.IDLE)

    def start_ultra_test(self):
        """Enter the ultra test test conduct a test to generate """

        self.state_change(State.ULTRA_TEST)
        # start rotating on the spot
        self.mwr.on(-2,2)
            #client.publish("ev3/sensor/accel", str( [time.time(), compass.value(0), x, y, z] ))

    def ultra_test(self):
        """Exectute ultrasonic sensor test"""
        
        [_x,_y,_z] = [0,0,0] #read_accel(accel)
        print("Range [cm]: %5.5f %3.1f %3.1f %3.1f %3.1f %3.1f" % (time.time(),
                                                            self.compass.value(0),
                                                            self.ultra.distance_centimeters,
                                                            _x, _y, _z ),file=outscreen)
        self.client.publish("ev3/sensor/ultra", str([time.time(), 
                                                        self.compass.value(0), 
                                                        self.ultra.distance_centimeters]))
        #client.publish("ev3/sensor/hdg", str([time.time(), compass.value(0)]))
        # TODO impement filter for aceel readings

    def heading_update(self):
        """
        Send an update of the heading sensor
        """
        
        [_x,_y,_z] = [0,0,0] #read_accel(accel)
        print("Heading [deg]: %5.5f %3.0f" % (time.time(), self.compass.value(0), ),file=outscreen)
        self.client.publish("ev3/sensor/hdg", str([time.time(), self.compass.value(0)]))
        # TODO impement filter for aceel readings


def main():
    '''The main function of our program'''

    # set the console just how we want it
    #reset_console()
    #set_cursor(OFF)
    #set_font('Lat15-Terminus24x12')

    # print something to the screen of the device
    print('Hello World!', file=outscreen)

    print(platform.python_version())
    print(sys.version)
    # print something to the output panel in VS Code
    print('Hello VS Code!',file=outscreen) 

    robot = EV3Controller()

    #accel = Sensor(INPUT_2)
    #accel.mode = 'ALL'
    #report_sensor(accel)
    




    # wait a bit so you have time to look at the display before the program
    # exits
    time.sleep(2)

if __name__ == '__main__':
    main()