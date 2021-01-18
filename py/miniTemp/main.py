#
import board
import busio
import gc
import storage
import time

import adafruit_rfm9x

from digitalio import DigitalInOut, Direction

gc.collect()

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Built in red LED
red_led = DigitalInOut(board.D13)
red_led.direction = Direction.OUTPUT
red_on = False

# Radio stuff TODO
cs = DigitalInOut(board.D10)
reset = DigitalInOut(board.D11)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)

def red_blink():
    red_led.value = True
    time.sleep(0.1)
    red_led.value = False

while True:
    red_blink()
    time.sleep(2.0)