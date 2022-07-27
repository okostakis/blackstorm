#!/usr/bin/env pybricks-micropython

# getLeftDistance getRightDistance
# calibrate gyro

# 12 περιστροφές

from operator import truediv
import myLib250 as myLib

import time

# test drive at specific distance from wall

c=myLib.myCar(loggingLevel=3)
# Το μοτέρ για το τιμόνι
c.motorA=myLib.myMotor(port='A',type='medium',inverted=False,loggingLevel=3,stallDetectionTime=0.5)
# Το μοτέρ για την κίνηση
c.motorB=myLib.myMotor(port='B',type='large',inverted=True,loggingLevel=3,stallDetectionTime=2)
# το μοτέρ για το sonar
c.motorC=myLib.myMotor(port='C',type='medium',inverted=False,loggingLevel=3,stallDetectionTime=0.5)

# Tα γυροσκόπιο
# c.gyro1=myLib.myGyro(port=1)
c.gyro=myLib.myGyro(port=4)
# Το μπροστινό sonar 
c.ultraSonic2 = myLib.myUltraSonic(port=2,loggingLevel=3)

# Ο αισθητήρας απόστασης για τον τοίχο
c.ultraSonic1 = myLib.myUltraSonic(port=1,loggingLevel=3)
c.ultraSonic3 = myLib.myUltraSonic(port=3,loggingLevel=3)

markDistance=0
c.Start(waitForButton=False)

c.UpdateSendorData()
c.Print('leftDistance=',c.leftDistance)

# 1st calibrate steering Lane
flag=False
while True:
    c.UpdateSendorData()    
    c.MoveToAngle(speed=50,targetAngle=0,findSteeringCorrection=True)
    if c.fullDistance>=40:
            c.SetSteeringCorrection()
            break
    

if c.leftDistance>40:
    lane=2
else:
    lane=1
if lane==1:
    marko1=20
else:
    marko1=24
    
#ang=η γονια που προχορα 
ang=0
while True:
    c.UpdateSendorData()
    c.SetSonarAngle(angle=-c.angle)
    c.MoveToAngle(targetAngle=ang,speed=50)
    
    if c.rightDistance < 0 or c.leftDistance< 0:
        turn=1
        if c.rightDistance> c.leftDistance:
            turn=-1
        c.StopAllMotors()
        break

c.Beep()
ang=40*turn

lane2ob=False
marko=c.fullDistance+ marko1
while True:
    c.UpdateSendorData()
    c.MoveToAngle(targetAngle=ang,speed=50)
    c.SetSonarAngle(angle=ang+10,speed=40)
    if c.fullDistance>marko:
        c.StopAllMotors()
        break
    if c.frontDistance<60:
        lane2ob=True
 
if lane2ob:
    c.Beep()
    mariposa = c.fullDistance + 25
    while True:
        c.UpdateSendorData()
        c.SetSonarAngle(angle=-(c.angle+10),speed=40)
        c.MoveToAngle(targetAngle=40*turn,speed=50)
        if c.frontDistance<30 and mariposa < c.fullDistance :
            c.StopAllMotors()
            break
    ang=90
    mariposa=c.fullDistance+40
    while True:
        c.UpdateSendorData()
        c.MoveToAngle(targetAngle=ang*turn,speed=50)
        c.SetSonarAngle(angle=0,speed=40)
        if c.fullDistance>mariposa:
            c.StopAllMotors()
            break
else:
    ang=90
    if lane==1:
        mariposa=c.fullDistance+40
    else:
        mariposa=c.fullDistance+20
    while True:
        c.UpdateSendorData()
        c.MoveToAngle(targetAngle=ang*turn,speed=50)
        c.SetSonarAngle(angle=0,speed=40)
        if c.fullDistance>mariposa:
            c.StopAllMotors()
            break

            





 








c.StopAllMotors()
# c.UpdateSendorData()
c.LogSensorData()
c.ExportDataToCSV(filename='data2.csv')
















c.Finish()


