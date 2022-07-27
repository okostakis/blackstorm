#!/usr/bin/env pybricks-micropython

# getLeftDistance getRightDistance
# calibrate gyro

# 12 περιστροφές

from operator import truediv
import myLib251 as myLib

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
c.UpdateSendorData()
c.UpdateSendorData()
left0=c.leftDistance
right0=c.rightDistance
c.LogSensorData()

# 1st calibrate steering Lane
flag=False
while True:
    c.UpdateSendorData()    
    c.MoveToAngle(speed=50,targetAngle=0,findSteeringCorrection=True)
    if c.fullDistance>=40:
            c.SetSteeringCorrection()
            break



lineAngle=0
turnFound=False
while True:
    if turnFound:
        c.UpdateSendorDataLimited()
    else:
        c.UpdateSendorData()
    # c.LogSensorData()
    c.SetSonarAngle(angle=-c.angle)
    c.MoveToAngle(targetAngle=lineAngle,speed=50)
    # Αν έχει εντοπιστεί ότι είμαι σε κατάσταση για στροφή και βρίσκομαι
    # σε συγκεκριμμένη απόσταση απο τον τοίχο
    if turnFound and c.frontDistance<70:
        c.StopAllMotors()
        break
    # Φρέθηκε κενό αριστερά ή δεξια οπότε έχω στροφή
    if c.rightDistance < 0 or c.leftDistance< 0:
        turnFound=True
        turn=1
        if c.rightDistance> c.leftDistance:
            turn=-1
    

if left0*turn>right0*turn:
    lane=2
else:
    lane=1

c.LogSensorData()

c.Turn3(speed=50,turn=turn,lane=lane,log=False)            
c.Print('lane=',lane, ' turn= ',turn )

c.StopAllMotors()
c.LogSensorData()
# c.UpdateSendorData()
c.LogSensorData()
c.ExportDataToCSV(filename='data2.csv')
















c.Finish()


