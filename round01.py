#!/usr/bin/env pybricks-micropython

# 12 περιστροφές

from operator import truediv
import myLib252 as myLib

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
front0=c.frontDistance
c.LogSensorData()

# start turn3a
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
c.Print('lane=',lane, ' turn= ',turn )

# end turn3


c.markDistance=c.fullDistance+120
rounds=3*4
for i in range(1,rounds):
    c.Print('------ Round:',i//4+1,'-',i%4+1, '--------')
    c.markDistance=c.fullDistance+100

    laneAngle=i*90
    c.Turn(angle=90*turn,speed=50)

    # Πάει ευθεία
    turnFound=False
    c.leftDistance=30
    c.rightDistance=30
    angleList=[]
    marko=c.fullDistance+100
    while True:
        c.UpdateSendorData()
        angleList.append(c.angle)
        if c.leftDistance*c.rightDistance<0:
            fix=0
        else:

            if turn ==-1:
                fix=c.CalcTargetAngleFromLeftWallDistance(targetLeftDistance=20,acceptedError=5,maxCorrection=12)
            else:
                fix= c.CalcTargetAngleFromRightWallDistance(targetRightDistance=20,acceptedError=5,maxCorrection=12)
        c.MoveToAngle(targetAngle=laneAngle*turn+fix*0.7,speed=70)
        c.SetSonarAngle(angle=(laneAngle*turn)-c.angle,speed=40)
        if (c.fullDistance>marko and c.rightDistance*c.leftDistance<0):
            turnFound=True
            c.StopAllMotors()
            c.Beep()
            break
       
    # calc gyro error
    angleList.sort()
    target=round(c.angle/90)*90
    l=len(angleList)
    c.Print('lanAngle=',target,' mo angle= ',angleList[l//2],)
    c.gyro.angleCorrection=c.gyro.angleCorrection+target-angleList[l//2]  







c.StopAllMotors()
# c.UpdateSendorData()
c.LogSensorData()
c.ExportDataToCSV(filename='data2.csv')

c.Finish()


