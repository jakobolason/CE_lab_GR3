import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

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
        # Blue is inacurate, we found that multiplying it with 1.75 works
        block_high_blue = int(data[5] * 1.9)
        blue = (block_low_blue + block_high_blue)

        #print("Color: (" + str(red) + ", " + str(green) + ", " + str(blue) + ")")

        # Convert the data to green, red and blue int values
        # Insert code here
        
        
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        # print("RGB(%d %d %d)" % (red, green, blue))
        
        time.sleep(2) 


getAndUpdateColour()