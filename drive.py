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

def shutdown_pwm(signum, stackframe):
    PWM.cleanup()

# THROTTLE RANGE 7.5 - 8
# STEERING RANGE 6-9

class Drive:

    def __init__(self):
        self.STEERING_ADR = 0x10
        self.THROTTLE_BRAKE_ADDR = 0x18
        
        self.i2c_connection = busio.I2C(board.SCL, board.SDA)
        
        if not  self.THROTTLE_BRAKE_ADDR in self.i2c_connection.scan():
            print("Didn't find Throttle And Brake Control Adruino.")

        if not self.STEERING_ADR in self.i2c_connection.scan():
            print("Didn't find Steering Control Adruino.")

        self.jumped = False

        self.start_throttle = 0
        self.cur_throttle = 0

        self.i2c_connection.writeto(self.THROTTLE_BRAKE_ADDR, 
                    bytes([self.start_throttle]))
        
        self.i2c_connection.writeto(self.STEERING_ADR, 
                    bytes([128]))

        self.encoder = Encoder()

    def get_speed(self):
        return self.encoder.get_ticks()

    def set_throttle_direct(self, value):
        if value < 255 and value >= 0:
            self.i2c_connection.writeto(self.THROTTLE_BRAKE_ADDR, 
                    bytes([value]))
            self.cur_throttle = value
        else:
            print("ERROR: inappropriate throttle's value being sent = "+str(value))
        

    def set_throttle(self, target, current, delta=0.0001):
        self.set_throttle_direct(targe)
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
                # print("Throttle decrease")
                # self.set_throttle_direct(self.cur_throttle - delta)
                self.set_throttle_direct(self.cur_throttle - (jump + self.diff_to_delta(diff)))
                #self.set_throttle_direct(target)
        elif target < 0:
            target = abs(target)
            if diff < 1:
                pass
            elif current > target:
                # print("Throttle increase")
                # self.set_throttle_direct(self.cur_throttle + delta)
                self.set_throttle_direct(self.cur_throttle + (jump + self.diff_to_delta(diff)))
                #self.set_throttle_direct(target)
            elif current < target:
                # print("Throttle decrease")
                # self.set_throttle_direct(self.cur_throttle - delta)
                self.set_throttle_direct(self.cur_throttle - (jump + self.diff_to_delta(diff)))
                #self.set_throttle_direct(target)
            print(self.cur_throttle)

    def set_steering(self, value):
        self.i2c_connection.writeto(self.STEERING_ADR, 
                    bytes([value]))

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
        server = Server()
        server.connect()

        UART.setup("UART1")
        ser = Serial("/dev/ttyO1", 9600)
        ser.close()
        ser.open()

        throttle_order = None
        steering_order = None
        throttle_actual = None
        steering_actual = None

        old_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
        self.set_throttle_direct(7.85)
        while True:
            line = ser.readline()
            try:
                dec_line = line.decode()
            except UnicodeDecodeError:
                continue
            if dec_line[0] != "S":
                continue
            throttle_actual, steering_actual  = dec_line[1:].strip().split("|")

            messages = server.read_messages(timeout=0.001)
            if messages:
                for mtype, message in messages[::-1]:
                    if mtype == RC_ORDER:
                        throttle_str, steering_str = message.split("|")
                        throttle_order = int(float(throttle_str))
                        if throttle_order != 0:
                            throttle_order = 6
                        else:
                            throttle_order =0;
                        steering_order = float(steering_str)
                        break
            elif not throttle_order or not steering_order:
                print("Skipping due to no orders")
                continue
            else:
                print("No order")

            self.set_steering(steering_order)
            self.set_throttle(throttle_order, int(throttle_actual))

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
    drive.set_steering(8)

    time.sleep(5)
    drive.set_throttle_direct(7.95)

    #throttle_vals = [3, 8, 6, 10, 4, 0]
    throttle_vals = [3, 4, 5, 4, 3, 3, 4, 5]


    UART.setup("UART1")
    ser = Serial("/dev/ttyO1", 9600)
    ser.close()
    ser.open()

    start = time.time()
    idx = 0
    while True:
        line = ser.readline()
        try:
            dec_line = line.decode()
        except UnicodeDecodeError:
            continue
        if dec_line[0] != "S":
            continue
        throttle, steering  = dec_line[1:].strip().split("|")
        print(throttle, throttle_vals[idx])
        drive.set_throttle(throttle_vals[idx], int(throttle))
        if time.time() - start > 4:
            start = time.time()
            idx += 1
            if not idx < len(throttle_vals):
                break

    #print(drive.set_throttle_v2(3))
    #time.sleep(2)
    #print(drive.set_throttle(250))
    #time.sleep(2)
    #print(drive.set_throttle(100))
    time.sleep(2)
    drive.close()
