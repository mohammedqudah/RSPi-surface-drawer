import RPi.GPIO as GPIO
import time
import math
import matplotlib.pyplot as plot


class step_motor:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.control_pins = [26, 19, 13, 6]
        for pin in self.control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        self.halfstep_seq = [
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
            [1, 0, 0, 1]
        ]

    def clockwise(self):
        for halfstep in range(8):
            for pin in range(4):
                GPIO.output(self.control_pins[pin], self.halfstep_seq[halfstep][pin])
            time.sleep(0.001)

    def counter_clockwise(self):
        for halfstep in range(7, -1, -1):
            for pin in range(4):
                GPIO.output(self.control_pins[pin], self.halfstep_seq[halfstep][pin])
            time.sleep(0.001)

    def move(self, degree):
        if degree > 0:
            rotations = round(degree / 0.7)
            for i in range(rotations):
                self.clockwise()
        else:
            rotations = round(degree / 0.7 * -1)
            for i in range(rotations):
                self.counter_clockwise()


class distance:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.GPIO_TRIGGER = 23
        self.GPIO_ECHO = 24
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def distance(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()
        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            StartTime = time.time()
        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            StopTime = time.time()
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        return distance

class draw:
    def coordinations(self, distance, angle):
        radian = angle * 0.0174533,2
        x = round(distance * math.cos(radian),1)
        y = round(distance*math.sin(radian),1)
        return [x,y]
    def plot(self,coordinations):
        plot.scatter(coordinations[0],coordinations[1])
    def savefig(self,file):
        plot.savefig(file)


if __name__ == "__main__":
    motor = step_motor()
    dstnc = distance()
    surround = draw()

    angle = 0
    moving_angle = 10
    for i in range(18):
        surround.coordinations(dstnc.distance(),angle)
        motor.move(moving_angle)
        angle =+ moving_angle
    surround.savefig('/tmp/fig1.png')

    GPIO.cleanup()
