import signal
import time

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
from serial import Serial

from encoder import Encoder
from EthernetAPI.server import Server
from EthernetAPI.message_types import RC_ORDER

# imports for I2C communications
import sys
import board
import busio
import subprocess

# For debugging
import argparse
import logging

BYPASS = 100
# THROTTLE RANGE 7.5 - 8
# STEERING RANGE 6-9

parser = argparse.ArgumentParser(description='BeagleBone Black middle man '
                                 +'connected to the Jetson Nano via an '
                                 +'ethernet cable for ' 
                                 +'steering/throttle+braking values.')

# Add an argument for logging level
parser.add_argument('--log', default='WARNING', help='CRITICAL > ERROR > WARNING > INFO > DEBUG ')
# Parse the argument
args = parser.parse_args()
# Convert to upper case and get numeric level
numeric_level = getattr(logging, args.log.upper(), None)
# Check if valid
if not isinstance(numeric_level, int):
    raise ValueError('Invalid log level: %s' % args.log)
# Create a logger object
logger = logging.getLogger('BBB')
# Set its level
logger.setLevel(numeric_level)
# Create a handler object
handler = logging.StreamHandler()
# Set its level
handler.setLevel(numeric_level)
# Create a formatter object
formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
# Add formatter to handler
handler.setFormatter(formatter)
# Add handler to logger
logger.addHandler(handler)


class Drive:
    def __init__(self):
        self.STEERING_ADDR = 0x10
        self.THROTTLE_BRAKE_ADDR = 0x18
        
        self.THROTTLE_CONNECTED =True
        self.STEERING_CONNECTED = True
        self.throttle_old = -5
        self.steering_old = -5
        global BYPASS
        self.throttle_bypass = BYPASS
        self.steering_bypass = BYPASS

        self.i2c = busio.I2C(board.SCL, board.SDA)

        if not  self.THROTTLE_BRAKE_ADDR in self.i2c.scan():
            logger.warning("Didn't find Throttle And Brake Control Adruino.")
            self.THROTTLE_CONNECTED = False

        if not self.STEERING_ADDR in self.i2c.scan():
            logger.warning("Didn't find Steering Control Adruino.")
            self.STEERING_CONNECTED = False

        self.jumped = False
        self.sending = 0
        self.start_throttle = 0
        self.cur_throttle = 0

        self.estop = False    
        
        self.encoder = Encoder()

    def set_throttle_direct(self, value: int):
        if self.throttle_old ==  value and self.throttle_bypass > 0:
            self.throttle_bypass -= 1
            return
        
        if not self.THROTTLE_CONNECTED:
            logger.warning('THROTTLE DISCONNECTED. '+str(value))
            return
        
        # connected_devs = []
        # try:
        #     connected_devs  = self.i2c.scan()
        # except:
        #     logger.warning("NO DEVICES CONNECTED")
        #     return

        # if not self.THROTTLE_BRAKE_ADDR in connected_devs:
        #     logger.warning("THROTTLE ADRUINOS DISCONNECTED. WANTED VALUE: "+str(value))
        #     return
        
        global BYPASS
        self.throttle_bypass = BYPASS
        self.throttle_old = value
        
        if value > 255 or value < 0:
            logger.warning("Inappropriate throttle's value being sent = "+str(value))
            return
        
        readbuffer = bytearray(1)
        try:
            self.i2c.writeto_then_readfrom(self.THROTTLE_BRAKE_ADDR,
                bytes([value]), readbuffer)
            logger.info('Got '+str(int(readbuffer[0]))+' from throttle.')
        except Exception as e:
            logger.error(e)
            logger.error('Set throttle direct failed. Retrying....')
            self.throttle_bypass = -1
            self.set_throttle_direct(value)
            return
        self.cur_throttle = value
            
    def set_steering(self, value: int):
        if self.steering_old == value and self.steering_bypass > 0:
            self.steering_bypass -= 1
            return
        
        if not self.STEERING_CONNECTED:
            logger.warning("STEERING DISCONNECTED. "+str(value))
            return
        
        # connected_devs = []
        # try:
        #     connected_devs = self.i2c.scan()
        # except:
        #     logger.warning("NO DEVICES FOUND")
        #     return
    
        # if  self.STEERING_ADDR in connected_devs:
        #     logger.warning("STEERING ADRUINOS DISCONNECTED. WANTED VALUE: "+str(value))
        #     return
       
        global BYPASS 
        self.steering_bypass = BYPASS
        self.steering_old = value 
        
        if value >255 or value <0:
            logger.warning("Trying to send an invalid steering angle")
            return
        
        readbuffer = bytearray(1)
        
        try:
            self.i2c.writeto_then_readfrom(self.STEERING_ADDR,
                    bytes([value]), readbuffer)
            logger.info('Got '+str(int(readbuffer[0]))+' from steering')
        except Exception as e:
            logger.error(e)
            logger.error('set steering failed. Retrying...')
            self.steering_bypass = -1
            self.set_steering(value)

    
    def send_steering_throttle(self, steering, throttle, cur_throttle):
        self.set_steering(steering)
        
        self.set_throttle(throttle, cur_throttle)
        
        
    

    
    def close(self):
        self.send_steering_throttle(128, 0)
        self.encoder.close()
        
    def drive_loop(self):
        # Connect to Jetson Nano for orders
        # Kill the process
        
        try:
            logger.info("Try killing existing process with opened port 60006.")
            pid = subprocess.check_output(['lsof', '-ti', f':{60006}']).strip()
            subprocess.run(['kill','-9', pid])
            logger.info("Sucessfully close port 60006.")
        except:
            pass
                
        
        logger.info('Waiting to connect to the server.')
        server = Server()
        server.connect()
        logger.info('Sucessfully connected to the server.')
        
        # Encoder feedback for speed control
        # TODO: Make this connect to the Arduino in charge of the golf cart speed encoder
        UART.setup("UART1")
        ser_encoder = Serial("/dev/ttyO1", 9600, timeout=0.1)
        ser_encoder.close()
        ser_encoder.open()
        if ser_encoder.readline() != b'':
            logger.info("Opened encoder serial sucessfully!!!")
        else:
            logger.warning("Unable to read line from encoder")

        # CV feedback for entering the stop state
        UART.setup("UART4")
        ser_cv = Serial("/dev/ttyO4", 9600, timeout=0.1)
        ser_cv.close()
        ser_cv.open()
        if ser_cv.readline() != b'':
            logger.info("Opened cv serial sucessfully!!!")
        else:
            logger.warning("Unable to read line from cv")
        cv_modifier = 1
        
        # Setup GPIO for emergency stop 
        def estop_pressed(channel):
            logger.warning("Emergency Button Pressed! ")
            time.sleep(.2)
            if (GPIO.input(channel) == GPIO.HIGH):
                self.estop = True

        ESTOP_PIN = "P9_12"
        GPIO.setup(ESTOP_PIN, GPIO.IN)
        GPIO.add_event_detect(ESTOP_PIN, GPIO.RISING, callback=estop_pressed)
        
        # Setup GPIO for state control
        IDLE = 0
        PARKING = 1
        PICKUP = 2
        STOP = 3
        STATE0_PIN = "P9_23"
        STATE1_PIN = "P9_27"
        #GPIO.setup(STATE0_PIN, GPIO.IN)
        #GPIO.setup(STATE1_PIN, GPIO.IN)

        # Initialize variables for the main loop
        throttle_order = None
        steering_order = None

        # Handle ctrl+c more gracefully
        old_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})

        while True:
            if self.estop:
                self.send_steering_throttle(128, 0)
                logger.critical("SHUTTING DOWN SYSTEM DUE TO E-BUTTON!!!!")
                return

            # Read in fob state
 #           state = 2 * GPIO.input(STATE1_PIN) + GPIO.input(STATE0_PIN)
            state = PICKUP
            # Temporary stop, e.g. ultrasonic sensor see something
            if state == STOP:
                logger.info("STATE: STOP!!!!!")
                self.send_steering_throttle(128, 0)
            # Take orders from the Jetson Nano
            elif state == IDLE or state == PARKING or state == PICKUP:

                # Get list messages from Jetson Nano
                messages = server.read_messages(timeout=0.001)

                # List is considered False if empty
                if messages:
                    # Work through the messages in reverse order
                    for mtype, message in messages[::-1]:
                    # If the order is an RC_ORDER update speed and steering
                        if mtype == RC_ORDER:
                            throttle_str, steering_str = message.split("|")
                            throttle_order = int(round(float(throttle_str)))
                            steering_order = int(round(float(steering_str)))
                            # We only care about the latest RC_ORDER
                            break
                    # Add more if's to handle new message types (if needed). Will also need to change break behavior above.
                #elif not throttle_order or not steering_order:
                   # print("Skipping due to no orders")
                #    continue
                else:
                    continue

            #    cv_modifier_enc = ser_cv.readline()
            #    try:
            #        cv_modifier_str = cv_modifier_enc.decode()
            #    except UnicodeDecodeError:
            #        continue
            #    if cv_modifier_str[0] == "S":
            #        cv_modifier = float(cv_modifier_str[1:])
            #        if cv_modifier < 0.3:
            #            print("CV stop")
            #            cv_modifier = 0
            #        elif cv_modifier < 0.7:
            #            cv_modifier = 0.5
            #        else:
            #            cv_modifier = 1
                cv_modifier = 1
                if throttle_order > 128:
                    scaled_throttle_order = int((throttle_order - 128) * cv_modifier + 128)
                else:
                    scaled_throttle_order = throttle_order
                logger.info(str(steering_order)+'|'+str(scaled_throttle_order))
                
                
                cur_throttle_enc = ser_encoder.readline()
                try:
                   cur_throttle_str = cur_throttle_enc.decode()
                except UnicodeDecodeError:
                    continue
               
                if cur_throttle_str[0] == "S":
                    cur_throttle = int(cur_throttle_str[1:])
                       
                self.send_steering_throttle(steering_order, scaled_throttle_order, cur_throttle)

                if signal.SIGINT in signal.sigpending():
                    logger.info("Cleaning up...")
                    self.close()
                    server.close()
                    ser_encoder.close()
                    ser_cv.close()
                    break

        signal.pthread_sigmask(signal.SIG_SETMASK, old_mask)

    # current: in the encoder space
    # target: in 0-255 range with 128-255 being throttling
    def set_throttle(self, target, current):
        if target <= 128:
            self.jumped = False
            logger.info("Throttle zeroed")
            self.set_throttle_direct(target)
            return
        
        # Remapping the current speed read from the sensor to within 128 to 255.
        max_speed = 500
        current = 128 + (current/(max_speed))*128 
        
        if target > 128 and current == 0 and not self.jumped:
            self.jumped = True
            jump = 10 # TODO: Tune this value
        
        diff = abs(target - current)
        #print(target, current, diff)
        if target > 128:
            if diff < 1:
                pass
            elif current < target:
                # print("Throttle increase")
                self.set_throttle_direct(self.cur_throttle + (jump + self.diff_to_delta(diff)))

            elif current > target:
                self.set_throttle_direct(
                    self.cur_throttle - (jump + self.diff_to_delta(diff))
                )
        # elif target < 0:
        #     target = abs(target)
        #     if diff < 1:
        #         pass
        #     elif current > target:
        #         self.set_throttle_direct(
        #             self.cur_throttle + (jump + self.diff_to_delta(diff))
        #         )
        #     elif current < target:
        #         # print("Throttle decrease")
        #         # self.set_throttle_direct(self.cur_throttle - delta)
        #         self.set_throttle_direct(self.cur_throttle - (jump + self.diff_to_delta(diff)))
        #         #self.set_throttle_direct(target)
        #     # print(self.cur_throttle)

    def get_speed(self):
        return self.encoder.get_ticks()
    
    def diff_to_delta(self, throttle_diff):
        if throttle_diff < 3:
            return -0.00005
        elif throttle_diff < 5:
            return 0.00005
        elif throttle_diff < 6:
            return 0.0001
        elif throttle_diff < 8:
            return 0.0003
        else:
            return 0.0004

    
if __name__ == "__main__":
    drive = Drive()
    drive.drive_loop()
    exit()
