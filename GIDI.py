import sensor, pyb
from pyb import LED
from Motor import Motor
from PCA9685 import PCA9685

class GIDI:
    def __init__(self):
        self.uart = pyb.UART(1, 115200)
        self.ledR = LED(1)
        self.ledG = LED(2)
        self.ledB = LED(3)
        self.ledB.on()  # Turn on BLUE LED to show startup
        self.deltaX = 0
        self.last_time = pyb.millis()
        self.prev_error = 0
        self.integral = 0

        try:
            print("Resetting sensor...")
            sensor.reset()
            print("Setting pixel format...")
            sensor.set_pixformat(sensor.RGB565)
            print("Setting frame size...")
            sensor.set_framesize(sensor.QVGA)
            print("Skipping frames...")
            sensor.skip_frames(time=2000)
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
            print("Sensor initialized.")
        except Exception as e:
            print("Sensor error:", e)
            self.ledR.on()

        self.pca9685 = PCA9685(i2c_bus=2, address=0x40)
        self.motorFL = Motor(self.pca9685, 0, 1)
        self.motorFR = Motor(self.pca9685, 2, 3)
        self.motorBL = Motor(self.pca9685, 4, 5)
        self.motorBR = Motor(self.pca9685, 6, 7)

    def pid(self, error, Kp, Ki, Kd):
        current_time = pyb.millis()
        dt = (current_time - self.last_time) / 1000.0
        self.last_time = current_time
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = Kp * error + Ki * self.integral + Kd * derivative
        self.prev_error = error
        return output

    def detect_black_line(self, max_intensity=50):
        try:
            img = sensor.snapshot()
            lines = img.find_lines(threshold=1000, theta_margin=25, rho_margin=25)
            if lines:
                self.ledR.on()
                self.ledB.on()
                self.ledG.off()
                for line in lines:
                    x1, y1, x2, y2 = line.x1(), line.y1(), line.x2(), line.y2()
                    r1, g1, b1 = img.get_pixel(x1, y1)
                    r2, g2, b2 = img.get_pixel(x2, y2)
                    avg = (r1 + g1 + b1 + r2 + g2 + b2) // 6
                    if avg <= max_intensity:
                        img.draw_line(line.line(), color=(0, 10, 240))
                        self.deltaX = x2 - x1
                        return self.deltaX
            return None
        except Exception as e:
            print("Line detection error:", e)
            return None

    def follow_line(self, SP, Kp, Ki, Kd, speed=40):
        line_data = self.detect_black_line()
        if line_data is not None:
            out = self.pid(line_data - SP, Kp, Ki, Kd)
            left_speed = max(-100, min(100, speed + out))
            right_speed = max(-100, min(100, speed - out))
            self.move(left_speed, right_speed, left_speed, right_speed)

    def move(self, speedFL, speedFR, speedBL, speedBR):
        self.motorFL.forward(speedFL)
        self.motorFR.forward(speedFR)
        self.motorBL.forward(speedBL)
        self.motorBR.forward(speedBR)

    def stop(self):
        self.motorFL.forward(0)
        self.motorFR.forward(0)
        self.motorBL.forward(0)
        self.motorBR.forward(0)


