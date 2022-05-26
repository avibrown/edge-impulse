import time
import board
import serial
import adafruit_mpu6050

# Init accelerometer
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

# Config serial
ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)

# Try to send accX value
while True:
        ser.write(b"%.2f\n" % mpu.acceleration[0])
        time.sleep(1 / 1000)
