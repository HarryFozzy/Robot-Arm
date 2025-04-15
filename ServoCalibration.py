from gpiozero import Servo
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

ServoPin1 = 27
Servo1 = Servo(ServoPin1, min_pulse_width=0/1000, max_pulse_width=4/1000, pin_factory=factory)

Servo1.value = 0

while True:
    PWM_Signal = int(input("Enter a number between -1 and 1: "))
    print(2*PWM_Signal+2)
    Servo1.value = PWM_Signal
    