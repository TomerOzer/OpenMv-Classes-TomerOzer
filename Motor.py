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