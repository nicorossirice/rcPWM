from time import sleep

from drive import Drive

if __name__ == "__main__":
    d = Drive()
    sleep(5)
    d.set_steering(9)
    d.set_throttle_direct(7.95)
    sleep(3)
    d.set_throttle_direct(7)
    sleep(5)
    d.close()
