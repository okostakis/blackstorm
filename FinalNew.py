#!/usr/bin/env pybricks-micropython

# 12 περιστροφές

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
c.Print('rightDistance=',c.rightDistance)

# 1st calibrate steering Lane
flag=False
while True:
    c.UpdateSendorData()    
    # c.LogSensorData()
    c.MoveToAngle(speed=50,targetAngle=0,findSteeringCorrection=True)
    if c.fullDistance>=40:
            c.SetSteeringCorrection()
            break
    c.SetSonarAngle(angle=-c.angle)
    

if c.leftDistance>40:
    lane=2
else:
    lane=1
c.markDistance=c.fullDistance+120
rounds=3*4
for i in range(rounds):
    c.Print('------ Round:',i//4+1,'-',i%4+1, '--------')
    c.markDistance=c.fullDistance+100
    laneAngle=i*90
    c.calcGyroAngleCorrection(init=True)
    while True:
        c.UpdateSendorData()

        # αν έχω φτάσει στον μπροστινό τοίχο
        if c.fullDistance>c.markDistance and  c.frontDistance<=42:
            if i <rounds-1:
                c.Turn(angle=90,speed=50)
            else:
                c.StopAllMotors()
                c.Print('Angle:',c.angle)
                c.Print('Angle/Round:',c.angle/(rounds-1))
                break
            if lane==1:
                c.markDistance=c.fullDistance+170              
            else:
                c.markDistance=c.fullDistance+165
            lane=1
            
            # c.SetAngleCorrection()
            # c.Print('angle correction=',c.angleCorrection)
            # lane=1
            break
        
        # αν υπάρχει μπροστά εμπόδιο
        elif c.frontDistance<=30:
            c.Beep()
            # in order to stop correction angle
            c.angleCorrectionState=5
            
            if lane==1:
                c.LeftToRightLane()
                c.markDistance=c.markDistance+20
                lane=2
            else:
                c.RightToLeftLane()
                c.markDistance=c.markDistance+20
                lane=1
            

        # υπολογίζει την γωνία πλοήγησης αναλόγως της απόστασης απο τον πλαινό τοίχο
        if lane==1:
            # υπολογίζει την διόρθωση του γυροσοπίου
            c.calcGyroAngleCorrection(forDistance=40,maxLeftDistance=70,log=True)
            # υπολογίζει την κατευθυνση βάση της απόστασης απο τον τοίχο
            wallCorrection=c.CalcTargetAngleFromWallDistance(targetLeftDistance=33,maxCorrection=12,acceptedError=5)
        else:
            # στην δεξια λωρίδα διορθώνων μόνο αν η απόσταση είναι πάνω απο 40
            # αλλιως σημαίνει ότι έχω αντικείμενο αριστερά
            if c.leftDistance>40 and c.leftDistance<90:
                wallCorrection=c.CalcTargetAngleFromWallDistance(targetLeftDistance=53,maxCorrection=12,acceptedError=5)
            else:
                wallCorrection=0
        # wallCorrection=0
        c.MoveToAngle(speed=50,targetAngle=laneAngle+wallCorrection)
        c.SetSonarAngle(angle=0)
            
c.StopAllMotors()
# c.UpdateSendorData()
c.LogSensorData()
c.ExportDataToCSV(filename='data2.csv')

c.Finish()


