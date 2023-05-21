import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

i2c = busio.I2C(board.SCL, board.SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

# Set the PWM frequency to 50Hz.
pca.frequency = 100
channel_number = 15  # change to the channel number connected to your servo

servo_motor = servo.Servo(pca.channels[channel_number], actuation_range=180, min_pulse=450, max_pulse=2400)

# reset servo to initial angle = 0
servo_motor.angle = 0   
time.sleep(1)

delay = 0.02
for i in range(1,179,1):
		servo_motor.angle += 1
		time.sleep(delay)
time.sleep(1)


# Make the servo motor move between extremes.
servo_motor.angle = 0     # 0 degrees, right
time.sleep(1)
servo_motor.angle = 90   # 180 degrees, left
time.sleep(1)
servo_motor.angle = 180   # 180 degrees, left
time.sleep(1)
servo_motor.angle = 90   # 180 degrees, left
time.sleep(1)

# Turn off the PCA9685's outputs (turn off the servo motor).
pca.deinit()

