from gpiozero import Servo
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

# --------------------
# Servo Setup
# --------------------

ServoPin1 = 27
ServoPin2 = 22
ShoulderServo = Servo(ServoPin1, min_pulse_width=0.5/1000, max_pulse_width=2/1000, pin_factory=factory)
ElbowServo = Servo(ServoPin2, min_pulse_width=0.5/1000, max_pulse_width=2/1000, pin_factory=factory)

# ------------------------------
# Servo Values to set
# ------------------------------

# Initial Servo Values
ShoulderStart = 0
ElbowStart = 75

# Servo Max Movement Angles
maxShoulderAngle = 135
maxElbowAngle = 90

# Set servos to mid point
ShoulderServo.mid()
# ElbowServo.mid()
ElbowServo.value = 0



CurrentShoulderAngle = ShoulderStart
CurrentElbowAngle = ElbowStart

# -----------------------
# Movement Functions
# -----------------------
        
def ChangeShoulderAngle(InputAngle):
    # Calculate target angles for both servos
    TargetElbowAngle = ElbowStart + InputAngle
    TargetShoulderAngle = ShoulderStart - InputAngle
    
    global CurrentShoulderAngle
    global CurrentElbowAngle
    
    IncrementAngle = 1

    while CurrentElbowAngle != TargetElbowAngle or CurrentShoulderAngle != TargetShoulderAngle:
        if CurrentElbowAngle != TargetElbowAngle:
            if CurrentElbowAngle < TargetElbowAngle:
                CurrentElbowAngle += IncrementAngle
            elif CurrentElbowAngle > TargetElbowAngle:
                CurrentElbowAngle -= IncrementAngle
         
            ElbowServo.value = CurrentElbowAngle/maxElbowAngle
            sleep(0.1)  # Delay for smooth movement

        if CurrentShoulderAngle != TargetShoulderAngle:
            if CurrentShoulderAngle < TargetShoulderAngle:
                CurrentShoulderAngle += IncrementAngle
            elif CurrentShoulderAngle > TargetShoulderAngle:
                CurrentShoulderAngle -= IncrementAngle
        
            ShoulderServo.value = CurrentShoulderAngle/maxShoulderAngle
            sleep(0.1)  # Delay for smooth movement

    print(f"Final Elbow Angle: {CurrentElbowAngle}")
    print(f"Final Shoulder Angle: {CurrentShoulderAngle}")

    
try:
    ChangeShoulderAngle(0)
    
    while True:
        
        InputAngle = int(input("Enter Angle: "))
        
        ChangeShoulderAngle(InputAngle)

        
except KeyboardInterrupt:
    print("Exiting program.")


