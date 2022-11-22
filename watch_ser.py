import signal

import Adafruit_BBIO.UART as UART
from serial import Serial

UART.setup("UART1")

ser = Serial(port="/dev/ttyO1", baudrate=115200)
ser.close()
ser.open()
if ser.isOpen():
    print("Serial is open")

def cleanup(signum, stackframe):
    print("cleaning...")
    UART.cleanup()
    exit()

signal.signal(signal.SIGINT, cleanup)

while True:
    print(ser.readline())
