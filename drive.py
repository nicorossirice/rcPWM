import signal
import time

import Adafruit_BBIO.GPIO as GPIO
# import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.UART as UART
from serial import Serial

from encoder import Encoder
from EthernetAPI.server import Server
from EthernetAPI.message_types import RC_ORDER

# imports for I2C communications
import sys
import board
import busio


# THROTTLE RANGE 7.5 - 8
# STEERING RANGE 6-9

class Drive:
    def __init__(self):
        self.STEERING_ADR = 0x10
        self.THROTTLE_BRAKE_ADDR = 0x18
        self.THROTTLE_CONNECTED =True
        self.STEERING_CONNECTED = True

        self.i2c_connection = busio.I2C(board.SCL, board.SDA)

        if not  self.THROTTLE_BRAKE_ADDR in self.i2c_connection.scan():
            print("Didn't find Throttle And Brake Control Adruino.")
            self.THROTTLE_CONNECTED = False

        if not self.STEERING_ADR in self.i2c_connection.scan():
            print("Didn't find Steering Control Adruino.")
            self.STEERING_CONNECTED = False

        self.jumped = False

        self.start_throttle = 0
        self.cur_throttle = 0

        self.set_throttle_direct(self.start_throttle)

        self.set_steering(0)

        self.encoder = Encoder()

        self.estop = False    
    def get_speed(self):
        return self.encoder.get_ticks()

    def set_throttle_direct(self, value):
        if not self.THROTTLE_CONNECTED:
            print("ERROR: THROTTLE ADRUINOS DISCONNECTED. WANTED VALUE: "+str(value))
            return

        if value <= 255 and value >= 0:
            self.i2c_connection.writeto(self.THROTTLE_BRAKE_ADDR,
                    bytes([int(value)]))
            self.cur_throttle = value
        else:
            print("ERROR: inappropriate throttle's value being sent = "+str(value))


    def set_throttle(self, target, current, delta=0.0001):
        self.set_throttle_direct(target)
        return

        if target == 0:
            self.jumped = False
            print("Throttle zeroed")
            #self.set_throttle_direct(self.start_throttle)
            self.set_throttle_direct(0)
            return

        diff = abs(target - current)
        #print(target, current, diff)
        if target > 0:
            if diff < 1:
                pass
            elif current < target:
                # print("Throttle increase")
                # self.set_throttle_direct(self.cur_throttle + delta)
                self.set_throttle_direct(self.cur_throttle + (jump + self.diff_to_delta(diff)))
                #self.set_throttle_direct(target)
            elif current > target:
                self.set_throttle_direct(
                    self.cur_throttle - (jump + self.diff_to_delta(diff))
                )
        elif target < 0:
            target = abs(target)
            if diff < 1:
                pass
            elif current > target:
                self.set_throttle_direct(
                    self.cur_throttle + (jump + self.diff_to_delta(diff))
                )
            elif current < target:
                # print("Throttle decrease")
                # self.set_throttle_direct(self.cur_throttle - delta)
                self.set_throttle_direct(self.cur_throttle - (jump + self.diff_to_delta(diff)))
                #self.set_throttle_direct(target)
            print(self.cur_throttle)

    def set_steering(self, value):
        if not self.STEERING_CONNECTED:
            print("ERROR: STEERING ADRUINOS DISCONNECTED. WANTED VALUE: "+str(value))
            return
        self.i2c_connection.writeto(self.STEERING_ADR,
                    bytes([int(value)]))

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

    def close(self):
        self.set_steering(128)
        self.set_throttle_direct(0)
        self.encoder.close()
        
    
          
    def drive_loop(self):
        # Connect to Jetson Nano
        server = Server()
        server.connect()

        # TODO: Make this connect to the Arduino in charge of the golf cart speed encoder
        UART.setup("UART1")
        ser = Serial("/dev/ttyO1", 9600)
        ser.close()
        ser.open()
        print("opened")
        estop = False
        def interrupt_handler(channel):
          print("Emergency detected. Braking all the way. And Setting steering to 0.")
          self.set_throttle_direct(0)
          self.set_steering(0)
          self.estop = True
        # Setup GPIO for emergency stop 
        ESTOP_PIN = "P9_12"
        GPIO.setup(ESTOP_PIN, GPIO.IN)
        GPIO.add_event_detect(ESTOP_PIN, GPIO.RISING, callback=interrupt_handler)
        
        

        # Initialize variables for the main loop
        throttle_order = None
        steering_order = None
        throttle_actual = None
        steering_actual = None

        # Handle ctrl+c more gracefully
        old_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})

        self.set_throttle_direct(7.85)
        while True:
            # TODO: Check  from Eunice's work? Depending on state, do different things
            state = "drive" # TODO: Replace this with actual states

            if self.estop:
                return

            if state == "drive":

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
                          #  if throttle_order != 0:
                          #      throttle_order 
                          #  else:
                          #      throttle_order =0;
                            steering_order = int(round(float(steering_str)))
                            # We only care about the latest RC_ORDER
                            break
                    # Add more if's to handle new message types (if needed). Will also need to change break behavior above.
                #elif not throttle_order or not steering_order:
                   # print("Skipping due to no orders")
                #    continue
                #else:
                   # print("No order")

                self.set_steering(steering_order)
                # self.set_throttle(throttle_order, int(throttle_actual))
                self.set_throttle(throttle_order, 0)

                if signal.SIGINT in signal.sigpending():
                    self.close()
                    server.close()
                    ser.close()
                    break

        signal.pthread_sigmask(signal.SIG_SETMASK, old_mask)


if __name__ == "__main__":
    drive = Drive()
    drive.drive_loop()
    exit()
