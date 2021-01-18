#
import board
import busio
import gc
import storage
import time

import adafruit_ahtx0
import adafruit_rfm9x

from digitalio import DigitalInOut, Direction

gc.collect()

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Control variables
LOOP_CNT = 20   # Number of loops to average over
loopCnt = 0     # Loop counter

# Built in red LED
red_led = DigitalInOut(board.D13)
red_led.direction = Direction.OUTPUT
red_on = False

# Radio stuff
cs = DigitalInOut(board.D10)
reset = DigitalInOut(board.D11)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)

# Temp/hum sensor
ahtx0 = adafruit_ahtx0.AHTx0(board.I2C())

# Sensor readings to average over
tmpVals = [0]*LOOP_CNT
humVals = [0]*LOOP_CNT

def red_blink():
    red_led.value = True
    time.sleep(0.1)
    red_led.value = False

while True:
    red_blink()

    if (loopCnt % LOOP_CNT == 0):
        tmpSum = sum(tmpVals)/LOOP_CNT
        humSum = sum(humVals)/LOOP_CNT
        rfm9x.send('TMP:%0.1f;HUM:%0.1f;ID:miniTemp\0' % (tmpSum, humSum))
        loopCnt = 0

    if (ahtx0.temperature):
        tmpVals[loopCnt%LOOP_CNT] = ahtx0.temperature*1.8 + 32.0
    if (ahtx0.relative_humidity):
        humVals[loopCnt%LOOP_CNT] = ahtx0.relative_humidity


    loopCnt += 1

    time.sleep(0.250)