import time

import Adafruit_BBIO.GPIO as GPIO


class Encoder:

    def __init__(self):
        self.encoder_pin = "P8_11"

        GPIO.setup(self.encoder_pin, GPIO.IN)
        GPIO.add_event_detect(self.encoder_pin, GPIO.RISING)

    def get_ticks(self, duration = 0.1, interval = 0.001):
        count = 0
        start = time.time()
        while time.time() - start < duration:
            if GPIO.event_detected(self.encoder_pin):
                count += 1
        print(count)
        return count / duration

    def close(self):
        GPIO.cleanup()
