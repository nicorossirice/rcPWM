import time
import sys
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
print("I2C devices found: ", [hex(i) for i in i2c.scan()])
tAndb_connected = True
s_connected = True


while True:
    if 0x18 in i2c.scan():
        t_val = int(input("Enter the desired throttle value for (0-127): "))
        b_val = int(input("Enter the desired brake value (0-100): "))
        
        if t_val<128 and t_val>=0:
            i2c.writeto(0x18, bytes([t_val+128]))
            print("Sending tuu_val of "+str(t_val))
        else:
            print("Invalid t_val.")
        if b_val<101 and b_val>=0:
            i2c.writeto(0x18, bytes([b_val]))
            print("Sending b_val of "+str(b_val))
        else:
            print("Invalid b_val.")
    
    if 0x10 in i2c.scan():
        print("Steering Instruction:")
        print("0 = leftmost | 64 = 1 rev | 128 = middle | 192 = 1 rev | 255 = rightmost")
        s_val = int(input("Enter the desired steering value (0-255): "))
        if s_val<256 and s_val>=0:
            i2c.writeto(0x10, bytes([s_val]))
            print("Sending s_val of "+str(s_val))
        else:
            print("Invalid s_val.")