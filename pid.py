from time import sleep

leftVel = 0
rightVel = 0
minVel = 190
maxVel = 255
Kp = 40
Ki = 0
Kd = 35
P = 0
I = 0
D = 0
PID = 0
error = 0
lastError = 0

errorP4 = [1, 1, 1, 1, 0]
errorP3 = [1, 1, 1, 0, 0]
errorP2 = [1, 1, 1, 0, 1]
errorP1 = [1, 1, 0, 0, 1]
errorN0 = [1, 1, 0, 1, 1]
errorN1 = [1, 0, 0, 1, 1]
errorN2 = [1, 0, 1, 1, 1]
errorN3 = [0, 0, 1, 1, 1]
errorN4 = [0, 1, 1, 1, 1]

actual  = [1, 1, 1, 1, 0]

while True:
    if   actual == errorP4: error = 2
    elif actual == errorP3: error = 1.75
    elif actual == errorP2: error = 1.5
    elif actual == errorP1: error = 1
    elif actual == errorN0: error = 0
    elif actual == errorN1: error = -1
    elif actual == errorN2: error = -1.5
    elif actual == errorN3: error = -1.75
    elif actual == errorN4: error = -2

    if error == 0:
        I = 0
    P = error
    I = I + error
    if error > 255:
        I = 255
    elif error < -255:
        I = -255
    D = error - lastError
    PID = (Kp * P) + (Ki + I) + (Kd *D)
    lastError = error

    if PID >= 0:
        leftVel = minVel
        rightVel = maxVel - PID
    else:
        leftVel = minVel + PID
        rightVel = maxVel

    if (leftVel < minVel):
        leftVel = minVel

    if (rightVel < minVel):
        rightVel = minVel

    print(f"leftVel: {leftVel} | rightVel {rightVel}")
    sleep(0.1)