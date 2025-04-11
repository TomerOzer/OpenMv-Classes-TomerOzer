from machine import I2C
import time

class PCA9685:
    def __init__(self, i2c_bus=2, address=0x40):
        self.i2c = I2C(i2c_bus)
        self.address = address
        self.reset()
        self.set_pwm_freq(50)

    def reset(self):
        self.i2c.writeto_mem(self.address, 0x00, bytearray([0x00]))

    def set_pwm_freq(self, freq_hz):
        prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)
        old_mode = self.i2c.readfrom_mem(self.address, 0x00, 1)[0]
        new_mode = (old_mode & 0x7F) | 0x10  # sleep
        self.i2c.writeto_mem(self.address, 0x00, bytearray([new_mode]))
        self.i2c.writeto_mem(self.address, 0xFE, bytearray([prescale_val]))
        self.i2c.writeto_mem(self.address, 0x00, bytearray([old_mode]))
        time.sleep_ms(5)
        self.i2c.writeto_mem(self.address, 0x00, bytearray([old_mode | 0x80]))

    def set_pwm(self, channel, on, off):
        reg = 0x06 + 4 * channel
        data = bytearray([
            on & 0xFF,
            (on >> 8) & 0xFF,
            off & 0xFF,
            (off >> 8) & 0xFF
        ])
        self.i2c.writeto_mem(self.address, reg, data)


class Motor:
    def __init__(self, pca9685, pin1, pin2):
        self.pca = pca9685
        self.pin1 = pin1
        self.pin2 = pin2

    def forward(self, speed):
        speed = max(-100, min(100, speed))
        pwm_value = int(abs(speed) * 65535 / 100)

        if speed > 0:
            self.pca.set_pwm(self.pin1, 0, pwm_value)
            self.pca.set_pwm(self.pin2, 0, 0)
        elif speed < 0:
            self.pca.set_pwm(self.pin1, 0, 0)
            self.pca.set_pwm(self.pin2, 0, pwm_value)
        else:
            self.pca.set_pwm(self.pin1, 0, 0)
            self.pca.set_pwm(self.pin2, 0, 0)
