import signal
import time

import Adafruit_BBIO.PWM as PWM

led_pin = "P9_14"
#led_pin2 = "P8_13"
PWM.start(led_pin, 0)
PWM.set_frequency(led_pin, 50)

PWM.set_duty_cycle(led_pin, 7.5)

signal.sigwait({signal.SIGINT})

#PWM.start(led_pin2, 0)
#PWM.set_frequency(led_pin2, 50)

#for i in range(0, 101, 10):

#    PWM.set_duty_cycle(led_pin, i)
#    PWM.set_duty_cycle(led_pin2, 100-i)

#    time.sleep(1)
print("beginning cleanup...")
PWM.stop(led_pin)

#PWM.stop(led_pin2)

PWM.cleanup()
