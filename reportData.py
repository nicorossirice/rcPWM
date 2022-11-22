import signal

import Adafruit_BBIO.UART as UART
from serial import Serial

from EthernetAPI.client import Client
from EthernetAPI.message_types import RC_DATA

def cleanup(signum, stackframe):
    print("\nCleaning up...\n")
    #client.disconnect()
    exit()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup)
    UART.setup("UART1")
    ser = Serial("/dev/ttyO1", baudrate=9600)
    ser.close()
    ser.open()

    #client = Client()
    #client.connect()

    while True:
        line = ser.readline()
        try:
            dec_line = line.decode()
        except UnicodeDecodeError:
            continue
        if dec_line[0] != "S":
            continue
        throttle, steering = dec_line[1:].strip().split("|")
        print(f"{throttle}|{float(steering) / 200}")
        #client.send_message(RC_DATA, f"{throttle}|{float(steering) / 0.02}")
