import socket
import os
import argparse
import serial
import numpy as np
import time
import struct
import csv
import config

def convertForArduino(inputVal, previousAngle):

    inputVal = inputVal*(256/800)

    spr = 800
    conv1 = spr*(1000/256)
    newAngle1 = (inputVal*conv1)/1000
    highLimit = 450
    lowLimit = 350
    midPoint = 400
    mult = 1000
    offset = mult*spr

    if (newAngle1 < highLimit) and (newAngle1 >lowLimit):
        if(newAngle1 > midPoint):
            newAngle1 = highLimit
        else:
            newAngle1 = lowLimit
    newAngle1 = newAngle1 + offset

    if np.abs(newAngle1-previousAngle) > 400:
        if(newAngle1 > previousAngle):
            mult = mult-1
            offset = mult*spr
            newAngle1 = newAngle1 - spr

        else:
            mult = mult + 1
            offset = mult*spr
            newAngle1 = newAngle1 + spr

    previousAngle = newAngle1

    return newAngle1

def add_row(fw, avgd=da,posy=None, odorbyte=None, valvestate=None):
    now = datetime.now()
    date_time = now.strftime("%m-%d-%Y_%H:%M:%S")
    fw.writerow([date_time,da,posy,odorbyte,valvestate])
    return None

def pollSocket(sock, last=None):
    data = sock.recv(1024)
    #if not data:
     #   break
    line = data.decode('UTF-8')
    toks = line.split(',')

    if ((len(toks) < 24) | (toks[0] != "FT")):
        print('Bad read')
        #continue
    try:
        posy = -float(toks[16])
        heading = float(toks[17])
    except ValueError:
        posy = last[0]
        heading = last[1]
    return posy, heading


def run(motorSER, odorSER, ftCONFIG, fw):
    if ftCONFIG is not None:
        host = ftCONFIG[0]
        port = ftCONFIG[1]

        # WRITE FLOW BASELINE VALUES TO ARDUINO
        if config.conditionalOdor:
            odor_val = str(int(255*config.percent_odor))+'\n'
            odorSER.write(str.encode(odor_val))

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            time.sleep(0.02)
            sock.connect((host, port))

            motorLastTime = time.time()
            odorLastTime = time.time()
            startTime = time.time()
            lockout=True

            slidingWindow = []
            counter = 0

            previousAngle = 800000

            while True:

                # Retrieve data from FicTrac
                posy, heading = pollSocket(sock)

                # End or preserve lockout
                if lockout==False:
                    pass
                else:
                    if (time.time()-startTime) > config.delay_time:
                        lockout==False

                # Scale and adjust
                propHead = heading/6.28
                target = int(propHead*800)
                posy = posy*3

                if config.conditionalOdor:
                    slidingWindow.append(posy)

                if (motorSER is not None) and (time.time() - motorLastTime > (1/30)):
                    correctedTarget = convertForArduino(target, previousAngle)
                    previousAngle = correctedTarget

                    if lockout==False:
                        motorSendStr = '<'+str(correctedTarget)+'>\n'
                        motorSER.write(str.encode(motorSendStr))
                motorLastTime = time.time()


                if (config.proportionalOdor):
                    if posy >= srcDistance:
                        odorSendVal = int(255)
                        odorSendVal = max(0, odorSendVal)
                        odorSendStr = '<'+str(odorSendVal)+'>\n'
                        if lockout==False:
                            odorSER.write(str.encode(odorSendStr))
                            add_row(fw, avgd=da,posy=posy, odorbyte=odorSendVal, valvestate=None)
                            print('ODOR SETPOINT : {} POSITION : {}'.format(odorSendStr, posy))

                    else:
                        odorSendVal =  int((config.maxAirFlow / 1000)*(posy/srcDistance)*255)
                        odorSendVal = max(0, odorSendVal)
                        odorSendStr = '<'+str(odorSendVal)+'>\n'
                        if lockout==False:
                            odorSER.write(str.encode(odorSendStr))
                            add_row(fw, avgd=da,posy=posy, odorbyte=odorSendVal, valvestate=None)
                            print('ODOR SETPOINT : {} POSITION : {}'.format(odorSendStr, posy))

                elif config.conditionalOdor:
                    if counter > 2:
                        slidingWindow.pop(0)
                        d = np.diff(slidingWindow)
                        da = np.mean(d)

                        # 1 = inactivated
                        # 2 = activated
                        if da < config.upwindCondition:
                            state = True
                            signal ='2'+'\n'
                        else:
                            state = False
                            signal = '1'+'\n'

                        print('AVERAGE DISPLACEMENT VALUE: {}, VALVE STATE: {}'.format(da, state))
                    else:
                        signal = '0'+'\n'
                    if lockout==False:
                        odorSER.write(str.encode(signal))
                        add_row(fw, avgd=da,posy=posy, odorbyte=None, valvestate=signal)

                else:
                    pass
                counter+=1

    else:
        motorLastTime = None
        odorLastTime = None

        if motorSER is not None:
            motorTestPts = config.motorTestPts
            motorLastTime=time.time()
        if odorSER is not None:
            odorTestPts = config.odorTestPts
            odorLastTime=time.time()

        for j in range(len(motorTestPts)):

            previousAngle = 800000

            if (motorSER is not None) and (time.time() - motorLastTime > (1)):
                target = motorTestPts[j]
                correctedTarget = convertForArduino(target, previousAngle)
                previousAngle = correctedTarget

                motorSendStr = '<'+str(correctedTarget)+'>\n'
                motorSER.write(str.encode(motorSendStr))
                motorLastTime = time.time()

            if (odorSER is not None) and (time.time() - odorLastTime > (1)):
                posy = int(1.2*srcDistance*odorTestPts[j])
                if posy >= srcDistance:
                    odorSendVal = int(255)
                else:
                    odorSendVal =  int((posy/srcDistance)*255)
                odorSendVal = max(0, odorSendVal)
                odorSendStr = '<'+str(odorSendVal)+'>\n'
                odorSER.write(str.encode(odorSendStr))
                odorLastTime = time.time()
            time.sleep(1)

if config.proportionalOdor:
    odorCOMM = config.odorCOMM
    odorBAUD = config.odorBAUD
    srcDistance = config.srcDistance
    maxAirFlow = config.maxAirFlow
    odorSER = serial.Serial(odorCOMM, odorBAUD)

elif config.conditionalOdor:
    odorCOMM = config.odorCOMM
    odorBAUD = config.odorBAUD
    odorSER = serial.Serial(odorCOMM, odorBAUD)

else:
    odorCOMM = None
    odorBAUD = None
    srcDistance = None
    maxAirFlow = None
    odorSER = None

if config.runMotor:
    motorCOMM = config.motorCOMM
    motorBAUD = config.motorBAUD
    motorSER = serial.Serial(motorCOMM, motorBAUD)
else:
    motorCOMM = None
    motorBAUD = None
    motorSER = None

if config.runFicTrac:
    ftHOST = config.ftHOST
    ftPORT = config.ftPORT

else:
    ftHOST = None
    ftPORT = None

if motorSER is not None or odorSER is not None:
    time.sleep(4)

from datetime import datetime
now = datetime.now() # current date and time
date_time = now.strftime("%m-%d-%Y_%H%M")

f = open(os.path.join(config.logDirectory, date_time+'.csv'), mode='wb', newline='')
fw = csv.writer(f, delimiter=',')
fw.writerow(['time','avg_vel','ypos','odor_signal','valve_state'])

run(motorSER, odorSER, [ftHOST, ftPORT], fw)
