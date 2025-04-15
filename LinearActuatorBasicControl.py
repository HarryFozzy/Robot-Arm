import gpiozero
from time import sleep

LinearActuator_Enable_PIN = 18 # Change this
LinearActuator_Input1_PIN = 17 # Change this
LinearActuator_Input2_PIN = 12 # Change this

LinearActuator_Enable = gpiozero.OutputDevice(LinearActuator_Enable_PIN, active_high=True, initial_value=False)
LinearActuator_Input1 = gpiozero.OutputDevice(LinearActuator_Input1_PIN, active_high=True, initial_value=False)
LinearActuator_Input2 = gpiozero.OutputDevice(LinearActuator_Input2_PIN, active_high=True, initial_value=False)

LinearActuator_Speed = 6 # mm/s

def extend_actuator(Distance):
    LinearActuator_Enable.on()
    LinearActuator_Input1.off()
    LinearActuator_Input2.on()
    sleep(Distance/LinearActuator_Speed)
    stop_actuator()
    
def retract_actuator(Distance):
    LinearActuator_Enable.on()
    LinearActuator_Input1.on()
    LinearActuator_Input2.off() 
    sleep(Distance/LinearActuator_Speed)
    stop_actuator()

def stop_actuator():
    LinearActuator_Enable.on()
    LinearActuator_Input1.off()
    LinearActuator_Input2.off()


try:
    while True:
        Extend = input("Extend? ")
        Distance = int(input("Distance 0-50mm: "))
        if Extend == "True":
            extend_actuator(Distance)
        else:        
            retract_actuator(Distance)

except KeyboardInterrupt:
    print("Exiting.")