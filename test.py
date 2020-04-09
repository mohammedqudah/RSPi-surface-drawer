import RPi.GPIO as GPIO
import time
import math
import matplotlib as mpl
mpl.use('Agg')
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

    def counter_clockwise(self):
        for halfstep in range(8):
            for pin in range(4):
                GPIO.output(self.control_pins[pin], self.halfstep_seq[halfstep][pin])
            time.sleep(0.001)

    def clockwise(self):
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
        delta = 0
        while GPIO.input(self.GPIO_ECHO) == 1 and delta < 1:
            StopTime = time.time()
            delta = StopTime - StartTime
            print(f'time delat is {0}'.str(delta))
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        return distance

class draw:
    def coordinations(self, distance, angle):
        radian = angle * 0.0174533
        x = round(distance * math.cos(radian),1)
        y = round(distance*math.sin(radian),1)
        return [x,y]
    def plot(self,x,y,file):
        plot.scatter(x,y)
        plot.plot(x,y)
        plot.savefig(file)


if __name__ == "__main__":
    motor = step_motor()
    dstnc = distance()
    surround = draw()

    angle = 0
    moving_angle = 1
    x=[]
    y=[]
    for i in range(180):
        #print(dstnc.distance())
        x_y=surround.coordinations(dstnc.distance(),angle)
        x.append(x_y[0])
        y.append(x_y[1])
        motor.move(moving_angle)
        angle = angle + moving_angle
        time.sleep(0.1)
    surround.plot(x,y,'/tmp/fig1.png')

    GPIO.cleanup()
