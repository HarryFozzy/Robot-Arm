from pynput import keyboard
import gpiozero
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import threading

factory = PiGPIOFactory()

#-----------------------------------------------------------
# DEFINE PINS
#-----------------------------------------------------------

# Linear Actuator
LinearActuator_Enable_PIN = 18  # Enable Pin # 1
LinearActuator_Input1_PIN = 17  # Input Pin 1 # 2
LinearActuator_Input2_PIN = 12  # Input Pin 2 # 7

# Servos
ShoulderServo_Pin = 27
ElbowServo_Pin = 22

# Electromagnets
EM_R_PIN = 23  # Energise to Release Pin
EM_H_PINS = [24, 25, 26]  # Energise to Hold Pins

#-----------------------------------------------------------
# PIN SETUP
#-----------------------------------------------------------
# Linear Actuator
LinearActuator_Enable = gpiozero.OutputDevice(LinearActuator_Enable_PIN, active_high=True, initial_value=False)
LinearActuator_Input1 = gpiozero.OutputDevice(LinearActuator_Input1_PIN, active_high=True, initial_value=False)
LinearActuator_Input2 = gpiozero.OutputDevice(LinearActuator_Input2_PIN, active_high=True, initial_value=False)

# Servos
ShoulderServo = Servo(ShoulderServo_Pin, min_pulse_width=0.5/1000, max_pulse_width=2/1000, pin_factory=factory)
ElbowServo = Servo(ElbowServo_Pin, min_pulse_width=0.5/1000, max_pulse_width=2/1000, pin_factory=factory)

# Electromagnets
EM_R = gpiozero.OutputDevice(EM_R_PIN, active_high=True, initial_value=True)
EM_H_1 = gpiozero.OutputDevice(EM_H_PINS[0], active_high=True, initial_value=False)
EM_H_2 = gpiozero.OutputDevice(EM_H_PINS[1], active_high=True, initial_value=False)
EM_H_3 = gpiozero.OutputDevice(EM_H_PINS[2], active_high=True, initial_value=False)

#-----------------------------------------------------------
# LINEAR ACTUATOR FUNCTIONS
#-----------------------------------------------------------
LinearActuator_Speed = 6  # mm/s - Movement speed with 12V supply

def extend_actuator():
    LinearActuator_Enable.on()
    LinearActuator_Input1.off()
    LinearActuator_Input2.on()

def retract_actuator():
    LinearActuator_Enable.on()
    LinearActuator_Input1.on()
    LinearActuator_Input2.off()

def stop_actuator():
    LinearActuator_Enable.on()
    LinearActuator_Input1.off()
    LinearActuator_Input2.off()

#-----------------------------------------------------------
# SERVO FUNCTIONS WITH THREADING
#-----------------------------------------------------------
maxAngle = 135  # Max Servo Angle either way

shoulder_running = False
elbow_running = False

def shoulder_dance():
    global shoulder_running
    while shoulder_running:
        ShoulderServo.value = 30 / maxAngle
        sleep(1)
        ShoulderServo.value = -30 / maxAngle
        sleep(1)

def elbow_dance():
    global elbow_running
    while elbow_running:
        ElbowServo.value = 30 / maxAngle
        sleep(1)
        ElbowServo.value = -30 / maxAngle
        sleep(1)

def start_shoulder_dance():
    global shoulder_running
    if not shoulder_running:
        shoulder_running = True
        threading.Thread(target=shoulder_dance, daemon=True).start()

def stop_shoulder_dance():
    global shoulder_running
    shoulder_running = False

def start_elbow_dance():
    global elbow_running
    if not elbow_running:
        elbow_running = True
        threading.Thread(target=elbow_dance, daemon=True).start()

def stop_elbow_dance():
    global elbow_running
    elbow_running = False

#-----------------------------------------------------------
# ELECTROMAGNET FUNCTIONS
#-----------------------------------------------------------
def ETH_ON():
    EM_H_1.on()
    EM_H_2.on()
    EM_H_3.on()

def ETH_OFF():
    EM_H_1.off()
    EM_H_2.off()
    EM_H_3.off()

def ETR_ON():
    EM_R.on()

def ETR_OFF():
    EM_R.off()

#-----------------------------------------------------------
# KEYBOARD LISTENER
#-----------------------------------------------------------
def on_key_event(event):
    try:
        key = event.char  # Get key pressed
    except AttributeError:
        return  # Ignore special keys

    if key == "a":
        print("Extending Actuator")
        extend_actuator()
    elif key == "b":
        print("Retracting Actuator")
        retract_actuator()
    elif key == "c":
        print("Stopping Actuator")
        stop_actuator()
    elif key == "d":
        print("Starting Shoulder Servo Dance")
        start_shoulder_dance()
    elif key == "k":
        print("Stopping Shoulder Servo Dance")
        stop_shoulder_dance()
    elif key == "e":
        print("Starting Elbow Servo Dance")
        start_elbow_dance()
    elif key == "l":
        print("Stopping Elbow Servo Dance")
        stop_elbow_dance()
    elif key == "f":
        print("Electromagnet (ETH) ON")
        ETH_ON()
    elif key == "g":
        print("Electromagnet (ETH) OFF")
        ETH_OFF()
    elif key == "i":
        print("Electromagnet (ETR) ON")
        ETR_ON()
    elif key == "j":
        print("Electromagnet (ETR) OFF")
        ETR_OFF()
    elif key == "h":
        print("Help:...\na - Extend Actuator\nb - Retract Actuator\nc - Stop Actuator\nd - Shoulder Servo Start\nk - Shoulder Servo Stop\ne - Elbow Servo Start\nl - Elbow Servo Stop\nf - ETH ON\ng - ETH OFF\ni - ETR ON\nj - ETR OFF")
    elif key == "esc":
        print("Exiting...")
        return False  # Stop listener

# Start listening for key presses
print("Listening for key presses... Press ESC to exit.")
with keyboard.Listener(on_press=on_key_event) as listener:
    listener.join()
