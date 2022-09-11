#!/usr/bin/env pybricks-micropython

# Εισαγωγή Απαραίτητων Βιβλιοθηκών
import myLib262 as myLib
import time


c=myLib.myCar(loggingLevel=3)
# Το μοτέρ για το τιμόνι
c.motorA=myLib.myMotor(port='A',type='medium',inverted=False,loggingLevel=3,stallDetectionTime=0.5)
# Το μοτέρ για την κίνηση
c.motorB=myLib.myMotor(port='B',type='large',inverted=True,loggingLevel=3,stallDetectionTime=2)
# το μοτέρ για το sonar
c.motorC=myLib.myMotor(port='C',type='medium',inverted=False,loggingLevel=3,stallDetectionTime=0.5)

# Tα γυροσκόπιο
c.gyro=myLib.myGyro(port=4)
# Το μπροστινό sonar 
c.ultraSonic2 = myLib.myUltraSonic(port=2,loggingLevel=3)

# Οι αισθητήρες απόστασης για τον τοίχο
c.ultraSonic1 = myLib.myUltraSonic(port=1,loggingLevel=3)
c.ultraSonic3 = myLib.myUltraSonic(port=3,loggingLevel=3)

markDistance=0
# Έναρξη του Ρομπότ. Έλεγχοι μοτέρ αισθητήρων. Εκκίνηση με πλήκτρο ή άμεση εκκίνηση
c.Start(waitForButton=False)
# min , max distance for leftDistance, rightDistance
c.minSideDistance=1
c.maxSideDistance=100

# Τρία update για να διαβαστούν και οι 3 αισθητήρες απόστασης.
# Σε κάθε update διαβ'αζεται 1 μόνο αισθητήρας ώστε να μην υπάρχει αλληλεπίδραση 
c.UpdateSendorData()
c.UpdateSendorData()
c.UpdateSendorData()

# Τοποθετώ σε μεταβλητές τις τιμές των αισθητήρων απόστασης
left0=c.leftDistance
right0=c.rightDistance
front0=c.frontDistance
c.LogSensorData()
# καταγράσω και στο αρχείο csv τα δεδομένα των αισθτήρων

# ******************************************************************************
# 1ο στάδιο (calibratee steering Error)                                        *
# για  30 εκατοστά κινούμε ευθεία στην πίστα καταγράφοντας τις διορθώσεις      *
# Οι διορθώσεις ουσιαστικά αποτελούν το σφάλμα ευθυγράμισης του τιμονιού.      *
#                                                                              *
#                                                                              *
#                                                                              *
# ******************************************************************************

flag=False
while True:
    c.UpdateSendorData()    
    c.MoveToAngle(speed=50,targetAngle=0,findSteeringCorrection=True)
    if c.fullDistance>=30:
            c.SetSteeringCorrection()
            break

# ******************************************************************************
# 2ο στάδιο (Αναγνώριση της αρχικής θέσης, φοράς περιστροφής διαδρομής)        *
# πηγαίνω ευθεία μέχρι να βρώ κενό είτε δεξιά είτε αριστερά                    *
#                                                                              *
#                                                                              *
#                                                                              *
#                                                                              *
# ******************************************************************************

# η αρχική γωνία 
lineAngle=0
# Λογική μεταβλιτή που δείχνει αν είμαι στην διαδικασία της στροφής
turnFound=False

while True:
    if turnFound:
        # αν έχει εντοπιστεί η στροφή τσεκάρω μόνο το μπροστινό sonar
        c.UpdateSendorDataLimited()
    else:
        # αλλιώς εκ περιτροπής τους 3 αισθητήρες απόστασης
        c.UpdateSendorData()
    # Το sonar κοιτάζει συνεχώς ευθεία
    c.SetSonarAngle(angle=-c.angle)
    # προχωράω στην γωνία της λωρίδας μου (εδώ 0)
    c.MoveToAngle(targetAngle=lineAngle,speed=50)
    # Αν έχει εντοπιστεί η στροφή και βρίσκομαι
    # σε συγκεκριμμένη απόσταση απο τον τοίχο
    if turnFound and c.frontDistance<70:
        # ολοκλρώνεται το 2ο στάδιο
        c.StopAllMotors()
        break
    # Αν στα αριστερο,δεξιο sonar βρεθεί αρνητική τιμή
    # τότε βρέθηκε κενό αριστερά ή δεξια οπότε έχω στροφή
    # καταγράφω ότι βρέθηκε στροφή (turnFound) καθώς και την φορά της
    # περιστροφής 1 δεξια, -1 αριστερά 
    if c.rightDistance < 0 or c.leftDistance< 0:
        turnFound=True
        turn=1
        if c.rightDistance> c.leftDistance:
            turn=-1
    
# Καταγράφω και σε ποια λωρίδα βρίσκομαι (1 εξωτερική, 2 εσωτερική)
if left0*turn>right0*turn:
    lane=2
else:
    lane=1
# Καταγράφω τις πιο  πρόσφατες τιμές των αισθητήρων σε csv 
c.LogSensorData()
# Υπολοφισμός του σημείου έναρξης (section)
section=0
add=0
if c.frontDistance<50:
    add = 40
if turn==-1:
    if right0>60:
        section=1
    elif right0<32:
        section=3
    else:
        section=2

    if c.fullDistance>55+add:
        section = section +3
if turn==1:
    if left0>60:
        section=4
    elif left0<32:
        section=6
    else:
        section=5

    if c.fullDistance>60+add:
        section = section -3

# Εμφανίζω όλα τα στοιχεία που υπολόγισα
c.Print("left", left0 , "right" , right0 , "front" , c.fullDistance)
c.Print('lane=',lane, ' turn= ',turn )
c.Print("section" , section)


# ******************************************************************************
# 3ο στάδιο (Πορεία και στροφή μέχρι να βρεθώ για τον τερματισμό)              *
#                                                                              *
#                                                                              *
#                                                                              *
#                                                                              *
#                                                                              *
# ******************************************************************************

# Επιλέγω των αριθμό των περιστροφών στην πίστα
rounds=3*4
for i in range(1,rounds):
    c.Print('------ Round:',i//4+1,'-',i%4+1, '--------')
    # μαρκάρω την ελάχιστη απόσταση που θα κινηθώ στην λωρίδα
    c.markDistance=c.fullDistance+100
    # υπολογίζω την γωνία του συγκεκριμμένου lane απο τον γύρο στον οποίο βρίσκομαι
    laneAngle=i*90
    # εκτελώ πρώτα την στροφή
    c.Turn(angle=90*turn,speed=50)

    # μετά ξεκινάω πορεία εμπρός μέχρι το τέλος του Lane
    # Αρχικοποιώ την νεταβλητή που δηλώνει αν εντοπίστηκε στροφή
    turnFound=False
    # δεν είμαι σίγουρος αν οι επόμενες αρχικοποιήσεις έχουν κάποια σημασία
    c.leftDistance=30
    c.rightDistance=30
    # η λίστα angleList χρησιμοποιείτε για να υπολογιστεί το σφάλμα του γυροσκοπίου και να διορθωθεί
    angleList=[]
    marko=c.fullDistance+100
    # το επόμενο loop εκτελεί την πορεία μέχρι το τέλος του lane
    while True:
        c.UpdateSendorData()
        # καταγράγω την γωνία απο το γυροσκόπιο
        angleList.append(c.angle)
        # υπολογίζω την διόρθωση στην πορεία μου βάση της απόστασης απο τον εσωτερικό τοίχο
        if c.leftDistance*c.rightDistance<0:
            # αν δεν έχω επαφή με κάποιο τοίχο δεν κάνω καμία διόρθωση 
            fix=0
        else:
            # αλλιώς υπολογίζω την διόρθωση βάσει της απόστασης απο τον εσωτερικό τοίχο
            if turn ==-1:
                fix=c.CalcTargetAngleFromLeftWallDistance(targetLeftDistance=20,acceptedError=5,maxCorrection=12)
            else:
                fix= c.CalcTargetAngleFromRightWallDistance(targetRightDistance=20,acceptedError=5,maxCorrection=12)
        # κινώ το ρομπότ βάσει της γωνίας του lane και πιθανής διόρθωση βάσει της απόστασης απο τον τοίχο
        c.MoveToAngle(targetAngle=laneAngle*turn+fix*0.7,speed=70)
        # το μπροστινό sonar κοιτάει πρός τον μπροστινό τοίχο
        c.SetSonarAngle(angle=(laneAngle*turn)-c.angle,speed=40)
        # Αν στα αριστερο,δεξιο sonar βρεθεί αρνητική τιμή
        # τότε βρέθηκε κενό αριστερά ή δεξια οπότε έχω στροφή
        if (c.fullDistance>marko and c.rightDistance*c.leftDistance<0):
            turnFound=True
            g=i
            c.StopAllMotors()
            c.Beep()
            break
       
    # Υπολογίζω το σφάλμα γυροσκοπίου και εφαρμόζω την διόρθωση
    # (χρησιμοποιώντας την τεχνική median filter)
    angleList.sort()
    target=round(c.angle/90)*90
    l=len(angleList)
    c.Print('lanAngle=',target,' mo angle= ',angleList[l//2],)
    c.gyro.angleCorrection=c.gyro.angleCorrection+target-angleList[l//2]  

# One more lane

    # Υπολογίζω το σφάλμα γυροσκοπίου και εφαρμόζω την διόρθωση
    # (χρησιμοποιώντας την τεχνική median filter)
    angleList.sort()
    target=round(c.angle/90)*90
    l=len(angleList)
    c.Print('lanAngle=',target,' mo angle= ',angleList[l//2],)
    c.gyro.angleCorrection=c.gyro.angleCorrection+target-angleList[l//2]  

# ******************************************************************************
# 4ο στάδιο (Τερματισμός)                                                      *
#                                                                              *
#                                                                              *
#                                                                              *
#                                                                              *
#                                                                              *
# ******************************************************************************

c.UpdateSendorData()
c.UpdateSendorData()
c.UpdateSendorData()


c.Print('leftDistance=',c.leftDistance,' rightDistance=',c.rightDistance,' marko=',marko)

c.Turn(angle=90*turn,speed=50)
c.StopAllMotors()
marko=c.fullDistance+60
while True:
    c.UpdateSendorData()
    c.MoveToAngle(speed=60,targetAngle=(laneAngle+90)*turn)
    if marko<c.fullDistance:
        c.StopAllMotors()
        break


        
       
        
    







c.StopAllMotors()
# c.UpdateSendorData()
c.LogSensorData()
c.Finish()

c.ExportDataToCSV(filename='data2.csv')



