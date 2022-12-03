import select
import signal
import sys
from time import sleep, time

import Adafruit_BBIO.UART as UART
from serial import Serial

from drive import Drive

if __name__ == "__main__":
    d = Drive()

    def cleanup(signum, stackframe):
        print("Cleaning up...")
        d.close()
        exit()
    signal.signal(signal.SIGINT, cleanup)
    UART.setup("UART1")
    ser = Serial("/dev/ttyO1", 9600)
    ser.close()
    ser.open()
    sleep(2)
    d.set_throttle_direct(7.85)
    #throttle_vals = [0, 4, 6, 8, 6, 5, 6]
    throttle_vals = [0, 4, 4, 4]
    cur_time = time()
    idx = 0
    order = 0
    while True:
        line = ser.readline()
        try:
            dec_line = line.decode()
        except UnicodeDecodeError:
            continue
        if dec_line[0] != "S":
            continue
        throttle, steering = dec_line[1:].strip().split("|")
        i, o, e = select.select( [sys.stdin], [], [], 0 )
        if (i):
            order = int(sys.stdin.readline().strip())
        #print(throttle, order)
        d.set_throttle(order, int(throttle), delta=0.0003)
        sleep(0.01)
        continue
        if time() - cur_time > 4:
            idx += 1
            cur_time = time()
            if not idx < len(throttle_vals):
                break
    d.close()
