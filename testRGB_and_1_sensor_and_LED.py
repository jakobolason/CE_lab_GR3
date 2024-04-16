import smbus
import time
import RPi.GPIO as GPIO

# Get I2C bus
bus = smbus.SMBus(1)

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO_range = 18
GPIO_TRIGECHO_LED = 15

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO_range, GPIO.OUT)  # Initial state as output
GPIO.setup(GPIO_TRIGECHO_LED, GPIO.OUT)  # Initial state as output

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO_range, False)
GPIO.output(GPIO_TRIGECHO_LED, False)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

def getAndUpdateColour():
    while True:
	# Read the data from the sensor
        # Insert code here
        data = bus.read_i2c_block_data(0x44, 0x09, 6)

        # Red
        block_low_red = data[2] / 256
        block_high_red = data[3]
        red = (block_low_red + block_high_red)

        # Green
        block_low_green = data[0] / 256
        block_high_green = data[1]
        green = (block_low_green + block_high_green) # Convert value to between 0 and 256.

        # Blue
        block_low_blue = data[4] / 256
        # Blue is inacurate, we found that multiplying it with 1.9 works
        block_high_blue = int(data[5] * 1.9)
        blue = (block_low_blue + block_high_blue)

        #print("Color: (" + str(red) + ", " + str(green) + ", " + str(blue) + ")")

        # Convert the data to green, red and blue int values
        # Insert code here
        
        
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        # print("RGB(%d %d %d)" % (red, green, blue))
        
        time.sleep(2) 

def blink_led(state, sleep):
  # State dictates if the LED is on or off: 0 = off, 1 = blinking and 2 = on
  if state == 0:
    # Turn off the LED
    GPIO.output(GPIO_TRIGECHO_LED, 0)
    time.sleep(sleep)
  elif state == 1:
    # Blink the led
    GPIO.output(GPIO_TRIGECHO_LED, 1)
    time.sleep(sleep)
    GPIO.output(GPIO_TRIGECHO_LED, 0)
    time.sleep(sleep)
  elif (state == 2):
    # Constantly blink the LED
    GPIO.output(GPIO_TRIGECHO_LED, 1)
    time.sleep(sleep)

def measure():
  # This function measures a distance
  # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO_range, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO_range, False)
  #ensure start time is set in case of very quick return
    start = time.time()

  # set line to input to check for start of echo response
    GPIO.setup(GPIO_TRIGECHO_range, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO_range)==0:
        start = time.time()

  # Wait for end of echo response
    while GPIO.input(GPIO_TRIGECHO_range)==1:
        stop = time.time()
  
    GPIO.setup(GPIO_TRIGECHO_range, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO_range, False)

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    time.sleep(0.1)
    return distance

try:
    while True:
        # Calibrate with 1.0752688172
        distance = measure() * 1.0752688172
        print ("Distance : %.1f cm" % distance)
        if 28 <= distance < 33:
          # Blink LED every 2 seconds
          blink_led(1, 1)
        elif 21 <= distance < 28:
          # Blink LED every second
          blink_led(1, 0.5)
        elif distance < 21:
          # Blink LED constantly
          blink_led(2, 1)
        else:
          # Turn off lED
          blink_led(0, 1)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()      

getAndUpdateColour()
measure()