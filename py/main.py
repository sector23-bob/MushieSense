# CircuitPython implementation of MushieSense, with LoRa!
import gc

from digitalio import DigitalInOut

gc.collect()

# Built in red LED
red_led = DigitalInOut(board.D13)
red_led.direction = Direction.OUTPUT