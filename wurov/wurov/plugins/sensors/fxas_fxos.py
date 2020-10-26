import numpy as np
import sys
import logging

from math import sqrt

import rclpy
from rclpy.node import Node

from wurov_messages.msg import Ninedof

try:
    import board
    import busio

    import adafruit_fxas21002c
    import adafruit_fxos8700

    enableSensor = True
except NotImplementedError as e:
    print(e, file=sys.stderr)
    enableSensor = False
except Exception as e:
    print(e, file=sys.stderr)
    enableSensor = False

logging.basicConfig(format='[Send][%(levelname)s]: %(message)s', level=logging.DEBUG)


class Sensor:
    # Offsets calculated by collecting data with the sensor stationary, averaging the values, and flipping the sign
    gyro_roll_offset = 0.092768
    gyro_pitch_offset = -0.290789
    gyro_yaw_offset = -0.387151
    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0

    # Magnetometer calibration is difficult due to all the factors that coud affect it.
    # This will remain uncalibrated since it is not currently being used.
    mag_x_offset = 0
    mag_y_offset = 0
    mag_z_offset = 0

    def __init__(self, i2c):
        # TODO: Set these values according to a user-specified transform that takes rotation of the sensor into account
        self._Roll = 0
        self._Pitch = 1
        self._Yaw = 2
        self._X = 0
        self._Y = 1
        self._Z = 2

        if enableSensor:
            self._i2c = i2c
            self._sensor_gyro = adafruit_fxas21002c.FXAS21002C(self._i2c)
            self._sensor = adafruit_fxos8700.FXOS8700(self._i2c)
        else:
            self._i2c = None
            self._sensor_gyro = None
            self._sensor = None

    def get_message(self) -> Ninedof:
        pass  # TODO: return a Ninedof object

    @property
    def gyro_roll(self):
        return self._sensor_gyro.gyroscope[self._Roll] + self.gyro_roll_offset if enableSensor else np.random.normal()

    @property
    def gyro_pitch(self):
        return self._sensor_gyro.gyroscope[self._Pitch] + self.gyro_pitch_offset if enableSensor else np.random.normal()

    @property
    def gyro_yaw(self):
        return self._sensor_gyro.gyroscope[self._Yaw] + self.gyro_yaw_offset if enableSensor else np.random.normal()

    @property
    def gyro_magnitude(self):
        return calc_magnitude(self.gyro_roll, self.gyro_pitch, self.gyro_yaw)

    # getAcc reads sensor data from accelerometer
    @property
    def accel_x(self):
        return self._sensor.accelerometer[self._X] + self.accel_x_offset if enableSensor else np.random.normal()

    @property
    def accel_y(self):
        return self._sensor.accelerometer[self._Y] + self.accel_y_offset if enableSensor else np.random.normal()

    @property
    def accel_z(self):
        return self._sensor.accelerometer[self._Z] + self.accel_z_offset if enableSensor else np.random.normal()

    @property
    def accel_magnitude(self):
        return calc_magnitude(self.accel_x, self.accel_y, self.accel_z)

    # getMag reads magnetometer values
    @property
    def mag_x(self):
        return self._sensor.magnetometer[self._X] + self.mag_x_offset if enableSensor else np.random.normal()

    @property
    def mag_y(self):
        return self._sensor.magnetometer[self._Y] + self.mag_y_offset if enableSensor else np.random.normal()

    @property
    def mag_z(self):
        return self._sensor.magnetometer[self._Z] + self.mag_z_offset if enableSensor else np.random.normal()

    @property
    def mag_magnitude(self):
        return calc_magnitude(self.mag_x, self.mag_y, self.mag_z)


class FXAS_FXOS(Node):
    """
    TODO: docstring
    """
    def __init__(self):
        super().__init__('fxas_fxos')
        self._publisher_ = self.create_publisher(Ninedof, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.read_sensor)

        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._sensor = Sensor(self.i2c)

    def read_sensor(self):
        msg = self._sensor.get_message()
        self._publisher.publish(msg)


def calc_magnitude(x, y, z):
    return sqrt((x ** 2) + (y ** 2) + (z ** 2))


def main(args=None):
    rclpy.init(args=args)

    fxas_fxos = FXAS_FXOS()

    rclpy.spin(fxas_fxos)

    fxas_fxos.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()