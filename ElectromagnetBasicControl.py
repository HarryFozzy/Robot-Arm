import gpiozero
from time import sleep

EM_H_PIN = 23

# Check these pins
EM_R_1_PIN = 24
EM_R_2_PIN = 25
EM_R_3_PIN = 26

# Triggered by the output pin going high: active_high=True
# Initially off: initial_value=False

EM_H = gpiozero.OutputDevice(EM_H_PIN, active_high=True, initial_value=False)
EM_R_1 = gpiozero.OutputDevice(EM_R_1_PIN, active_high=True, initial_value=False)
EM_R_2 = gpiozero.OutputDevice(EM_R_2_PIN, active_high=True, initial_value=False)
EM_R_3 = gpiozero.OutputDevice(EM_R_3_PIN, active_high=True, initial_value=False)

Check_Position = bool(input("True or False: "))
while Check_Position:
    
    EM_R_1.on()
    EM_R_2.on()
    EM_R_3.on()
    
    sleep(5)
    
    EM_H.on() # switch on
    
    Transfer_Complete = bool(input("Transfer Complete?: "))
    if Transfer_Complete:
        EM_R_1.off()
        EM_R_2.off()
        EM_R_3.off()
        EM_H.off()
        break
    