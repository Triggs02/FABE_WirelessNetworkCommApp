import sys
import subprocess
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

# Verify command-line argument
if len(sys.argv) < 2:
    print("Usage: python stats.py <state_value>")
    sys.exit(1)

state_value = int(sys.argv[1])
if not (0 <= state_value <= 9):
    print("State value must be an integer from 0 to 9.")
    sys.exit(1)

# Define the status based on state_value
if state_value == 0:
    status = "Off"
elif 1 <= state_value <= 3:
    status = "On"
elif 4 <= state_value <= 7:
    status = "Data Collect"
elif state_value == 8:
    status = "Data Transmit"
elif state_value == 9:
    status = "Data Store"
else:
    status = "Unknown"

# Display setup
oled_reset = digitalio.DigitalInOut(board.D4)
WIDTH = 128
HEIGHT = 64
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)

# Create a blank image for drawing
image = Image.new("1", (oled.width, oled.height))
draw = ImageDraw.Draw(image)
font = ImageFont.truetype('PixelOperator.ttf', 16)

# Fetch system statistics
cmd_ip = "hostname -I | cut -d' ' -f1"
IP = subprocess.check_output(cmd_ip, shell=True).decode().strip()
cmd_signal = "iwconfig wlan0 | grep 'Link Quality' | awk '{print $2}' | cut -d'=' -f2"
signal = subprocess.check_output(cmd_signal, shell=True).decode().strip()

# Determine network strength
if not signal:
    network_strength = "Disconnected"
else:
    signal_quality = int(signal.split('/')[0])
    if signal_quality < 20:
        network_strength = "Weak"
    elif 20 <= signal_quality < 50:
        network_strength = "Average"
    elif signal_quality >= 50:
        network_strength = "Strong"

# Fetch temperature
cmd_temp = "vcgencmd measure_temp | cut -f2 -d'='"
Temp = subprocess.check_output(cmd_temp, shell=True).decode().strip()

# Clear the previous display
draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

# Positioning the text
draw.text((0, 0), "IP: " + IP, font=font, fill=255)
draw.text((0, 16), "Net: " + network_strength, font=font, fill=255)
draw.text((0, 32), "Temp: " + Temp, font=font, fill=255)
draw.text((0, 48), "State: " + status, font=font, fill=255)

# Update the display
oled.image(image)
oled.show()
