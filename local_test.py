from time import sleep, time

import Adafruit_BBIO.UART as UART
from serial import Serial

from drive import Drive

if __name__ == "__main__":
    d = Drive()
    UART.setup("UART1")
    ser = Serial("/dev/ttyO1", 9600)
    ser.close()
    ser.open()
    sleep(5)
    throttle_vals = [0, 4, 6, 8, 6, 5, 6]
    cur_time = time()
    idx = 0
    while True:
        line = ser.readline()
        try:
            dec_line = line.decode()
        except UnicodeDecodeError:
            continue
        if dec_line[0] != "S":
            continue
        throttle, steering = dec_line[1:].strip().split("|")
        print(throttle, throttle_vals[idx])
        d.set_throttle(throttle_vals[idx], int(throttle), delta=0.0003)
        if time() - cur_time > 4:
            idx += 1
            cur_time = time()
            if not idx < len(throttle_vals):
                break
    d.close()
