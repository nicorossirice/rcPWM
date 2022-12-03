import signal
import time

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.UART as UART
from serial import Serial

from encoder import Encoder
from EthernetAPI.server import Server
from EthernetAPI.message_types import RC_ORDER

def shutdown_pwm(signum, stackframe):
    PWM.cleanup()

# THROTTLE RANGE 7.5 - 8
# STEERING RANGE 6-9

class Drive:

    def __init__(self):
        self.throttle_pin = "P8_19"
        self.steering_pin = "P8_13"

        self.jumped = False

        self.start_throttle = 7.5
        self.cur_throttle = 7.5

        PWM.start(self.throttle_pin, self.start_throttle, 50, 0)
        PWM.set_duty_cycle(self.throttle_pin, self.start_throttle)
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

    # Old set throttle method
    # def set_throttle(self, ticks):
    #     valid = 0
    #     necessary_valid = 10
    #     while True:
    #         cur_ticks = self.encoder.get_ticks()
    #         if abs(cur_ticks - ticks) < 50:
    #             valid += 1
    #             if valid > necessary_valid:
    #                 break
    #         elif cur_ticks < ticks:
    #             valid = 0
    #             self.set_throttle_direct(self.cur_throttle + 0.01)
    #         elif cur_ticks > ticks:
    #             valid = 0
    #             self.set_throttle_direct(self.cur_throttle - 0.01)
    #         time.sleep(0.1)
    #     return cur_ticks

    def set_throttle(self, target, current, delta=0.0001):
        if target == 0:
            self.jumped = False
            #print("Throttle zeroed")
            #self.set_throttle_direct(self.start_throttle)
            self.set_throttle_direct(7.85)
            return

        jump = 0
        if not self.jumped and current == 0:
            jump = 0.15
            self.jumped = True

        diff = abs(target - current)
        print(target, current, diff)
        if diff < 1:
            pass
        elif current < target:
            #print("Throttle increase")
            # self.set_throttle_direct(self.cur_throttle + delta)
            self.set_throttle_direct(self.cur_throttle + (jump + self.diff_to_delta(diff)))
            #self.set_throttle_direct(target)    
        elif current > target:
            #print("Throttle decrease")
            # self.set_throttle_direct(self.cur_throttle - delta)
            self.set_throttle_direct(self.cur_throttle - (jump + self.diff_to_delta(diff)))
            #self.set_throttle_direct(target)

    def set_steering(self, duty_cycle):
        PWM.set_duty_cycle(self.steering_pin, duty_cycle)

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
        self.set_steering(7.5)
        self.set_throttle_direct(7.5)
        PWM.stop(self.throttle_pin)
        PWM.stop(self.steering_pin)
        PWM.cleanup()
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
                            throttle_order = 3
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
