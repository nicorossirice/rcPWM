import time

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.UART as UART
from serial import Serial

from encoder import Encoder

# THROTTLE RANGE 7.5 - 8
# STEERING RANGE 6-9

class Drive:

    def __init__(self):
        self.throttle_pin = "P9_14"
        self.steering_pin = "P8_13"

        self.cur_throttle = 7.95

        PWM.start(self.throttle_pin, self.cur_throttle, 50, 0)
        PWM.set_duty_cycle(self.throttle_pin, self.cur_throttle)
        PWM.set_frequency(self.throttle_pin, 50)

        PWM.start(self.steering_pin, 7.5, 50, 0)
        PWM.set_duty_cycle(self.steering_pin, 7.5)
        PWM.set_frequency(self.steering_pin, 50)

        self.encoder = Encoder()

    def get_speed(self):
        return self.encoder.get_ticks()


    def set_throttle_direct(self, duty_cycle):
        PWM.set_duty_cycle(self.throttle_pin, duty_cycle)
        self.cur_throttle = duty_cycle

    def set_throttle(self, ticks):
        valid = 0
        necessary_valid = 10
        while True:
            cur_ticks = self.encoder.get_ticks()
            if abs(cur_ticks - ticks) < 50:
                valid += 1
                if valid > necessary_valid:
                    break
            elif cur_ticks < ticks:
                valid = 0
                self.set_throttle_direct(self.cur_throttle + 0.01)
            elif cur_ticks > ticks:
                valid = 0
                self.set_throttle_direct(self.cur_throttle - 0.01)
            time.sleep(0.1)
        return cur_ticks

    def set_throttle_v2(self, target, current):
        delta = 0.0005
        if abs(target - current) < 1:
            pass
        elif current < target:
            self.set_throttle_direct(self.cur_throttle + delta)
        elif current > target:
            self.set_throttle_direct(self.cur_throttle - delta)

    def set_steering(self, duty_cycle):
        PWM.set_duty_cycle(self.steering_pin, duty_cycle)

    def close(self):
        self.set_steering(7.5)
        self.set_throttle_direct(7.5)
        PWM.stop(self.throttle_pin)
        PWM.stop(self.steering_pin)
        PWM.cleanup()
        self.encoder.close()

if __name__ == "__main__":
    drive = Drive()
    drive.set_steering(8)

    throttle_vals = [3, 8, 6, 10, 4, 0]

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
        drive.set_throttle_v2(throttle_vals[idx], int(throttle))
        if time.time() - start > 3:
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
