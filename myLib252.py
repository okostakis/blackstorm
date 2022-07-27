#!/usr/bin/env pybricks-micropython

import pybricks.ev3brick as brick

from  pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor
     ,GyroSensor)
from  pybricks.parameters import (Port,Stop,Direction,Button,Color,SoundFile,ImageFile,Align)

from pybricks.tools import(print,wait,StopWatch)
from  pybricks.robotics import DriveBase
from  pybricks.ev3devio import Ev3devSensor

import threading
import time
import math

class display2():
    """[summary]
    """
    def __init__(self):      
        self._disp = brick.display
        self._dv = self._disp._device
        self._fb = self._dv._fb
        self._font_height = 7
        self.set_line_height(10)
        self._font_width = 8
        self._auto_update = True
        self._prev_text_location = (0, -self._line_height)
    
    def set_auto_update(self, auto_update):
        """[summary]

        Args:
            auto_update ([type]): [description]
        """
        # auto_update to display on the screen once the methods are called
        # for much faster implementation, it is better to use set_auto_update(False),
        # and call update() after making the necessary changes to screen
        self._auto_update = auto_update
    
    def set_line_height(self, line_height):
        """[summary]

        Args:
            line_height ([type]): [description]
        """
        # the method to set the line height, useful for scrolling only
        self._line_height = int(line_height)
        self._padding_top = int((self._line_height - self._font_height) / 2)
    
    def _update(self, update):
        """[summary]

        Args:
            update ([type]): [description]
        """
        if (self._auto_update and update != False) or update:
            self.update()
    
    def update(self):
        """[summary]
        """
        # writes changes to screen
        self._dv._screen.update(self._dv._data)
    
    def text(self, text, coordinate=None, grayscale=0, update=None):
        """[summary]

        Args:
            text ([type]): [description]
            coordinate ([type], optional): [description]. Defaults to None.
            grayscale (int, optional): [description]. Defaults to 0.
            update ([type], optional): [description]. Defaults to None.
        """
        # get color
        grayscale = self._gray_to_hex(grayscale)
        # decide if scrolling is needed, otherwise, use the supplied tuple
        if coordinate is None:
            # auto positioning required
            new_coordinate = (self._prev_text_location[0], self._prev_text_location[1] + self._line_height)
            if new_coordinate[1] <= -self._padding_top:
                new_coordinate = (new_coordinate[0], 0)
            if new_coordinate[1] <= 128 - self._font_height - self._padding_top:
                # the next line is still visible on the screen
                coordinate = new_coordinate
                self._prev_text_location = coordinate
            else:
                coordinate = self._prev_text_location
                # scroll up by one line height
                self._fb.scroll(0, -self._line_height)
        # clear only the space where the font is written
        self._fb.fill_rect(coordinate[0], coordinate[1], len(text) * self._font_width, self._line_height, 0xffffff)
        # write the text
        self._fb.text(text, coordinate[0], coordinate[1] + self._padding_top, grayscale)
        # decide if update is required.
        self._update(update)
    
    def rectangle(self, coordinate, width, height, grayscale=0, update=None):
        """[summary]

        Args:
            coordinate ([type]): [description]
            width ([type]): [description]
            height ([type]): [description]
            grayscale (int, optional): [description]. Defaults to 0.
            update ([type], optional): [description]. Defaults to None.
        """
        grayscale = self._gray_to_hex(grayscale)
        self._fb.fill_rect(coordinate[0], coordinate[1], width, height, grayscale)
        self._update(update)
    
    def line(self, coordinate_start, coordinate_end, grayscale=0, update=None):
        """[summary]

        Args:
            coordinate_start ([type]): [description]
            coordinate_end ([type]): [description]
            grayscale (int, optional): [description]. Defaults to 0.
            update ([type], optional): [description]. Defaults to None.
        """
        grayscale = self._gray_to_hex(grayscale)
        self._fb.line(coordinate_start[0], coordinate_start[1], coordinate_end[0], coordinate_end[1], grayscale)
        self._update(update)
    
    def point(self, coordinate, grayscale=0, update=None):
        """[summary]

        Args:
            coordinate ([type]): [description]
            grayscale (int, optional): [description]. Defaults to 0.
            update ([type], optional): [description]. Defaults to None.
        """
        grayscale = self._gray_to_hex(grayscale)
        self._fb.pixel(coordinate[0], coordinate[1], grayscale)
        self._update(update)
    
    def scroll(self, pixels_in_x, pixels_in_y, update=None):
        """[summary]

        Args:
            pixels_in_x ([type]): [description]
            pixels_in_y ([type]): [description]
            update ([type], optional): [description]. Defaults to None.
        """
        self._fb.scroll(pixels_in_x, pixels_in_y)
        self._update(update)
    @staticmethod
    
    def _gray_to_hex(grayscale):
        """[summary]

        Args:
            grayscale ([type]): [description]

        Returns:
            [type]: [description]
        """
        grayscale = int(grayscale)
        if grayscale == 1:
            return 0x666666
        elif grayscale == 2:
            return 0xaaaaaa
        elif grayscale >= 3:
            return 0xffffff
        else:
            return 0

    def example(self,text1):
        self.set_line_height(8)
        self.text(text1)
        self.update()
        time.sleep(5)

#http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensor_data.html#ht-nxt-color-v2

#######################################################################################################
# for nxt Light Sensors
class lightSensor(Ev3devSensor):
    """
    Κλάση για την υποστήριξη των παλαιών nxt light sensors
    """
    _ev3dev_driver_name = 'lego-nxt-light'

    def ambient(self):        
        """[summary]

        Returns:
            [type]: [description]
        """
        self._mode('AMBIENT')
        return self._value(0) / 10

    def reflection(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        self._mode('REFLECT')
        return self._value(0) / 10

#######################################################################################################
# for HT Color Sensor 
class htColorSensor(Ev3devSensor):
    """
    Κλάση για την υποστήριξη των HT sensors
    """
    _ev3dev_driver_name = 'ht-nxt-color-v2'
    _number_of_values = 4
    def rgbw(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        self._mode('RGB')
        errors = 0
        while True:
            try:
                return self._value(0), self._value(1), self._value(2), self._value(3)
            except:
                errors += 1 # may give ENODEV error
                if errors > 1:
                    return 0, 0, 0, 0
    def color(self):
        """[summary]

        Returns:
            [type]: [description]
        """

        self._mode('COLOR')
        errors = 0
        while True:
            try:
                return self._value(0)
            except:
                errors += 1 # may give ENODEV error
                if errors > 1:
                    return 0

#######################################################################################################
class myLogger(object):
    """ 
    Είναι η κλάση που καταγράφει πληροφορίες και γεγονότα. 
    Οι πληροφορίες αυτές παρουσιάζονται σε οθόνη ή σε αρχείο η και στα δύο.
    Σε αρχείο γράφονται μόνο οι πληροφορίες απο επίπεδο warning και πάνω.
      
    Attributes: 
        name (str): Το όνομα του αντικειμένου. 
        filename (str): Το όνομα του αρχείου στο οποίο γίνεται η καταγραφή. 
        level (int): 1 info 2 debug 3 warning 4 error
        info (str): Κείμενο χαρακτηρισμένο ώς πληροφορία
        debug (str): Κείμενο χαρακτηρισμένο ώς αποσφαλμάτωση
        warning (str): Κείμενο χαρακτηρισμένο ώς σημαντικό ή προειδοποίηση
        error (str): Κείμενο χαρακτηρισμένο σαν προειδοποίηση
        output (list): Οι προορισμοί της καταγραφής ['screen','file']
    """
    name = 'main'
    filename = 'log.txt'
    level = 3
    info=''
    debug=''
    warning=''
    error=''
    out=''
    outputs=['screen','file']
    startTime=0
    parent = None


    def __init__(self, name='main', filename='log.txt', level=3, outputs=['screen','file']):
        """
        Αρχικοποιεί την κλάση myLogger

        Parameters:
            name (str): Ένα όνομα του αντικειμένου.
            filename (str): Το όνομα του αρχείου που αποθηκεύονται οι καταγραφές.
            level (int): 1 info 2 debug 3 warning 4 error.
            outputs (list): Η λίστα με τις εξόδους της καταγραφής ['screen','file'].  

        Return:
            None:   
        """
        self.startTime=time.perf_counter()
        self.name = name
        self.filename = filename
        self.level = level
        self.outputs=outputs
        if 'file' in self.outputs:
            file = open(self.filename, 'w')
            file.close()

    def SetLevel(self,level=3):
        """
        Καθορίζει το επίπεδο καταγραφής. 
        
        Parameters:
            level (int): 1 debug 2 info 3 warning 4 error.
        """
        self.level=level
    
    def SetFile(self,filename='log.txt'):
        """
        Καθορίζει το όνομα αρχείου που γράφει τις πληροφορίες η κλάση.

        Parameters:
            filename: Το όνομα αρχείου που θα δημιουργηθεί για την καταγραφή.
        """
        self.filename=filename
    
    def SetOutputs(self,outputs=['screen','file']):
        """
        Καθορίζει τις εξόδους για την κλάση.

        Parameters:
            outputs: Η λίστα με τις εξόδους της καταγραφής πχ ['screen','file']
        """
        self.outputs=outputs

    def Info(self, log='',createdBy=''):
        """
        Καταγράφει την πληροφορία που δέχεται σαν παράμετρο και την χαρακτηρίζει σαν πληροφορία
        
        Parameters:
            log (str): Η πληροφορία προς καταγραφή.
        
        Return:
            None:
        """
            
        if self.level <= 1:
            if createdBy=='':
                cBy=self.name
            else:
                cBy=createdBy
            try:
                self.parent.parent.logger.Info(log=log,createdBy=cBy)
            except  Exception as e:
                timeElapsed=round(time.perf_counter()-self.startTime,3)
                self.info = "\n"+str(timeElapsed)+': '+'INFO:\t' + cBy + ' ' + log
                self.out += "\n"+str(timeElapsed)+': '+'INFO:\t' + cBy + ' ' + log
                if 'screen' in self.outputs:
                    print(self.info)
           
    def Debug(self, log='',createdBy=''):
        """
        Καταγράφει την πληροφορία που δέχεται σαν παράμετρο και την χαρακτηρίζει σαν αποσφαλμάτωση
        
        Parameters:
            log (str): Η πληροφορία προς καταγραφή.
        
        Return:
            None:
        """
        if self.level <= 2:
            if createdBy=='':
                cBy=self.name
            else:
                cBy=createdBy
            try:
                self.parent.parent.logger.Debug(log=log,createdBy=cBy)
            except Exception as  e:
                timeElapsed=round(time.perf_counter()-self.startTime,3)
                self.debug = "\n"+str(timeElapsed)+': '+'DEBUG:\t' + cBy + ' ' + log 
                self.out += "\n"+str(timeElapsed)+': '+'DEBUG:\t' + cBy + ' ' + log 
                if 'screen' in self.outputs:
                    print(self.debug)
               

    def Warning(self, log='',createdBy=''):
        """
        Καταγράφει την πληροφορία που δέχεται σαν παράμετρο και την χαρακτηρίζει σαν Σημαντικό/Προειδοποίηση
        
        Parameters:
            log (str): Η πληροφορία προς καταγραφή.
        
        Return:
            None:
        """
        if self.level <= 3:
            if createdBy=='':
                cBy=self.name
            else:
                cBy=createdBy
            try:
                self.parent.parent.logger.Warning(log=log,createdBy=cBy)
            except Exception as  e:
                timeElapsed=round(time.perf_counter()-self.startTime,3)
                self.warning +="\n"+str(timeElapsed)+': '+ 'WARNING:\t'  +cBy+  ' ' + log 
                self.out +="\n"+str(timeElapsed)+': '+ 'WARNING:\t' +cBy + ' ' + log 
                if 'screen' in self.outputs:
                    print(str(timeElapsed)+': '+'WARNING:\t' + cBy + ' \n' +log)
                    # elif 'file' in self.outputs:
                    #     self.warning +=str(timeElapsed)+': '+ 'WARNING:\t' + self.name + ' ' + log 
            self.Save()    
    def Error(self, log='',createdBy=''):
        """
        Καταγράφει την πληροφορία που δέχεται σαν παράμετρο και την χαρακτηρίζει σαν σφάλμα
        
        Parameters:
            log (str): Η πληροφορία προς καταγραφή.
        
        Return:
            None:
        """
        if self.level <= 4:
            if createdBy=='':
                cBy=self.name
            else:
                cBy=createdBy
            
            try :
                self.parent.parent.logger.Error(log=log,createdBy=cBy)
            except Exception as  e:
                timeElapsed=round(time.perf_counter()-self.startTime,3)
                self.error +='\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                self.error += "\n"+str(timeElapsed)+': '+'ERROR:\t' + cBy +' ' + log 
                self.out += "\n"+str(timeElapsed)+': '+'ERROR:\t' + cBy + ' ' + log 
                if 'screen' in self.outputs:
                    print('\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                    print(str(timeElapsed)+': '+'ERROR:\n' + self.name + ' ' +log)
                    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n')
                # elif 'file' in self.outputs:
                #     self.error += str(timeElapsed)+': '+'ERROR:\n' + self.name + ' ' + log 
                self.error +='\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            self.Save()
    def Out(self, log='',createdBy=''):
        """
        Καταγράφει την πληροφορία που δέχεται σαν παράμετρο
        
        Parameters:
            log (str): Η πληροφορία προς καταγραφή.
        
        Return:
            None:
        """
        if createdBy=='':
            cBy=self.name
        else:
            cBy=createdBy
            
        try :
            self.parent.parent.logger.Out(log=log,createdBy=cBy)
        except Exception as  e:
            timeElapsed=round(time.perf_counter()-self.startTime,3)
            self.out += "\n"+str(timeElapsed)+': '+cBy+ ' ' + log 
            if 'screen' in self.outputs:
                print(str(round(timeElapsed,2))+': ' +cBy + ' ' +log)


    def Print(self, log=''):
        """
        Εμφανίζει το Μήνυμα
        
        Parameters:
            log (str): Η πληροφορία προς προβολή.
        
        Return:
            None:
        """
        print(log)


    def Save(self):
        """
        Αποθηκεύει όλες τις καταγραφές σε αρχείο !!! που έχουν χαρακτηριστεί σαν προειδοποίηση ή σφάλμα σε αρχείο.
        
        """
        if 'file' in self.outputs:
            if self.out!='':
                self.file = open(self.filename, 'a')
                self.file.write(self.out+'\r\n')
                self.file.close()
                self.out=''
'''
            if self.info!='':
                self.file = open(self.filename, 'a')
                self.file.write(self.info+'\r\n')
                self.file.close()
                self.info=''
            if self.debug!='':
                self.file = open(self.filename, 'a')
                self.file.write(self.debug+'\r\n')
                self.file.close()
                self.debug=''
            if self.warning!='':
                self.file = open(self.filename, 'a')
                self.file.write(self.warning+'\r\n')
                self.file.close()
                self.warning=''
            if self.error!='':
                self.file = open(self.filename, 'a')
                self.file.write(self.error+'\r\n')
                self.file.close()
                self.error=''
'''

class myPID(object):
    """
    Η κλάση PID μας παρέχει τις λειτουργίες για την διόρθωση σφαλμάτων
    μέσω της μεθοδολογίας P.I.D.

    Attributes: 
        name (str): Ένα όνομα για τον PID. (χρησιμοποιείτε στις καταγραφές)
        error (float): Το τρέχον σφάλμα του PID.
        lastError (float): Το προηγούμενο σφάλμα του PID.
        integral (float): Το συσσωρευτικό σφάλμα του PID.
        derivative (float): Το διαφορικό σφάλμα του PID.
        kp (float): Ο αναλογικός συντελεστής.
        ki (float): Ο συσσωρευτικός συντελεστής.
        kd (float): Ο διαφορικός συντελεστής.
        target (float): Η τιμή στόχος. Η απόκλιση απο το στόχο θεωρείτε το σφάλμα.
        direction (int): παίρνει τιμές 1 ή -1. Ουσιαστικά καθορίζει το πρόσιμο της διόρθωσης.
        lastTime (timeStamp): ο χρόνος που πραγματοποιήθηκε το προηγούμενο βήμα του PID.
        dt (float):
        rdt (float):
        pcList (list):
        errorList (list):
        i (int):
        j (int):
        sumError (float):
        loggingStats (bool): True αν επιθυμούμε να κρατάμε στατιστικά της χρήσης του PID



    """
    name=''    
    error = 0
    derivative = 0
    lastError = 0
    integral = 0
    kp=0
    ki=0
    kd=0
    target=0
    direction=1
    lastTime=0
    #dt=0
    rdt=0
    pcList=[]
    errorList=[]
    minCorrection=99999999
    maxCorrection=-99999999
    i=0
    j=0
    sumError=0
    loggingStats=True
    loggingLevel=2


    def __init__(self,name='', kp=0, kd=0, ki=0, direction=1, loggingStats=True,loggingLevel=2):
        """
        Η μέθοδος PID εφαρμόζεται για την διόρθωση σφαλμάτων.

        Parameters:
            kp: Ο αναλογικός συντελεστής για την διόρθωση. 
            kd: Ο διαφορικός συντελεστής για την διόρθωση.
            ki: Ο συσσωρευτικός συντελεστής για την διόρθωση.
            direction: 1 or -1 (για θετική ή αντίστροφη διόρθωση)
            loggingStats: if True the class logs statistic data fot pid usage
        :return
            None
        """    
        self.name=name
        self.i=0
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.direction=direction
        #self.dt=dt
        self.lastError = self.error = self.integral = 0
        self.sumError=0
        self.loggingStats=loggingStats
        self.loggingLevel=loggingLevel
        self.lastTime=time.perf_counter()
        self.rdt=0
        self.pcTime=0
        self.pc=0
        self.pcList=[]
        self.errorList=[]
        self.correctionList=[]
        #for k in range(200):
        #    self.errorList.append(0)

    def SetParameters(self,name='', kp=0, kd=0, ki=0, direction=1, loggingStats=True,loggingLevel=2):
        """[summary]

        Args:
            name (str, optional): [description]. Defaults to ''.
            kp (int, optional): [description]. Defaults to 0.
            kd (int, optional): [description]. Defaults to 0.
            ki (int, optional): [description]. Defaults to 0.
            direction (int, optional): [description]. Defaults to 1.
            loggingStats (bool, optional): [description]. Defaults to True.
            loggingLevel (int, optional): [description]. Defaults to 2.
        """
        self.name=name
        self.i=0
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.direction=direction
        #self.dt=dt
        self.lastError = self.error = self.integral = 0
        self.sumError=0
        self.loggingStats=loggingStats
        self.loggingLevel=loggingLevel
        self.lastTime=time.perf_counter()
        self.rdt=0
        self.pcTime=0
        self.pc=0
        self.pcList=[]
        self.errorList=[]
        self.correctionList=[]

    def ListToStr(self, list):
        """
        Μετατρέπει μια λίστα σε κείμενο

        Parameters:
            list: η λίστα για μετατροπή
        
        return: 
            str: το κείμενο που αντιστοιχεί στην λίστα
        """
        str1 = '[' + ', '.join(str(e) for e in list) + ']'
        return (str1)

    def Reset(self):
        """
        Αρχικοποιεί τον αλγόριθμο PID.

        return:
            None
        """
        self.lastError = self.error = self.integral = 0
        self.i=0
        self.sumError=0
        self.lastTime=time.perf_counter()
        #self.dt=0
        self.pcTime=0
        self.pc=0
        self.pcList=[]
        self.errorList=[]
        #for k in range(200):
        #    self.errorList.append(0)
               
    def Step(self, inputRead=0,target=0):
        """
        Υπολογίζει την διόρθωση βάσει του αλγόριθμου PID.

        Parameters:
            inputRead: Η τιμή εισόδου.            
            target: Η τιμή στόχος (στην τιμή αυτή το σφάλμα είναι 0)
        :return
            (float): Η υπολογιζόμενη διόρθωση βάση του αλγόριθμου PID.
        """
        # self.lastError = self.error = self.integral = 0
        self.target=target
        self.error = self.target - inputRead
        self.derivative = self.error - self.lastError
        #self.integral = float(0.5) * self.integral + self.error
        self.integral =  self.integral + self.error
        course = (self.kp * self.error + self.kd * self.derivative + self.ki * self.integral) * self.direction
        #print(course)
        if self.loggingStats:
            if course>self.maxCorrection:
                self.maxCorrection=course
            if course<self.minCorrection:
                self.minCorrection=course              
            if len(self.errorList)<200:
                self.errorList.append((self.error,course))
            else:
                self.errorList[self.i % 200]=(self.error,course)
                self.errorList[(self.i+1) % 200 ]= '-->|'
            self.i += 1            
            self.sumError += self.error**2
            currTime=time.perf_counter()                
            if self.rdt!=0:
                self.rdt=(self.rdt+currTime-self.lastTime)/2.0
            else:
                self.rdt=currTime-self.lastTime
            self.lastTime=currTime
            if self.lastError>0 and self.error<=0:
                pct=time.perf_counter()
                #print(pct)
                if self.pcTime!=0:
                    self.pc=pct-self.pcTime
                    if len(self.pcList)<100:
                        self.pcList.append(self.pc)
                self.pcTime=pct
        self.lastError = self.error
        return (course)

    def StepToCheckKc(self, inputRead=0,target=0):
        """
        Under Construction
        Εφαρμόζει μια συνάρτηση στην είσοδο με έξοδο -maxError + maxError
        Υπολογίζει την διόρθωση βάσει του αλγόριθμου PID.

        Parameters:
            inputRead: Η τιμή εισόδου.            
            target: Η τιμή στόχος (στην τιμή αυτή το σφάλμα είναι 0)
            maxError: Το μέγιστο δυνατό σφάλμα.
        :return
            (float): Η υπολογιζόμενη διόρθωση βάση του αλγόριθμου PID.
        """
        # self.lastError = self.error = self.integral = 0
        v=100-target
        self.target=target
        self.error = self.target - round(inputRead/v)*v # this for calc Kc
        self.derivative = self.error - self.lastError
        self.integral = float(0.5) * self.integral + self.error
        course = (self.kp * self.error + self.kd * self.derivative + self.ki * self.integral) * self.direction
        #print(course)
        if self.loggingStats:
            if course>self.maxCorrection:
                self.maxCorrection=course
            if course<self.minCorrection:
                self.minCorrection=course
            if len(self.errorList)<200:
                self.errorList.append((self.error,course))
            else:
                self.errorList[self.i % 200]=(self.error,course)
                self.errorList[(self.i+1) % 200 ]= '-->|'
            self.i += 1
            self.sumError += self.error**2
            currTime=time.perf_counter()
            if self.rdt!=0:
                self.rdt=(self.rdt+currTime-self.lastTime)/2.0
            else:
                self.rdt=currTime-self.lastTime
            self.lastTime=currTime
            if self.lastError>0 and self.error<=0:
                pct=time.perf_counter()
                #print(pct)
                if self.pcTime!=0:
                    self.pc=pct-self.pcTime
                    self.pcList.append(self.pc)
                self.pcTime=pct
        self.lastError = self.error
        return (course)

    def GetStatistics(self):
        """
        Επιστρέφει το deviationError και άλλα στατιστικά στοιχεία απο την τελευταία χρήση του PID
        
        Returns (str): Τα στατιστικά στοιχεία σε μορφή κειμένου.
        """
        s=''
        if self.loggingStats:
            s+='\n|--- '+self.name+' ---\n'  
            s+="| kp="+str(self.kp)+' ki='+str(self.ki)+' kd='+str(self.kd) +'\n' 

            if self.i!=0:                
                s+='| Deviation Error: '+str((self.sumError/self.i)**0.5)
                s+='\n| avg dt:'+str(self.rdt)
                s+='\n| Correction: from '+str(round(self.minCorrection,3))+' to ' +str(round(self.maxCorrection,3) )                   

            if self.loggingLevel==1:
                s+='\n| errorList: '+self.ListToStr(self.errorList)
                try:
                    s+='\n| max(pc):'+str(max(self.pcList))
                    s+='\n| min(pc):'+str(min(self.pcList))
                    s+='\n| avg(pc):'+str(sum(self.pcList)/len(self.pcList))
                    s+='\n| len(pc list):'+str(len(self.pcList))
                    s+='\n| pcList: '+self.ListToStr(self.pcList)
                except:
                    s+='\n Error to generate stats. Propably pcList is null'
            s+='\n|-------------------------------'    
        return(s)


# To tune your PID controller you follow these steps:
#
# Set the Ki and Kd values to zero, which turns those terms off and makes the controller act like a simple P controller.
# Set the Tp term to a smallish one. For our motors 25 might be a good place to start.
# Set the Kp term to a "reasonable" value. What is "reasonable"?
# I just take the maximum value we want to send to the motor's power control (100) and divide by the maximum useable error value. For our line following robot we've assumed the maximum error is 5 so our guess at Kp is 100/5=20. When the error is +5 the motor's power will swing by 100 units. When the error is zero the motor's power will sit at the Tp level.
# Or, just set Kp to 1 (or 100) and see what happens.
# If you have implemented that the K's are all entered as 100 times their actual value you have to take that into account here. 1 is entered as 100, 20 as 2000, 100 as 10000.
# Run the robot and watch what it does. If it can't follow the line and wanders off then increase Kp. If it oscillates wildly then decrease Kp. Keep changing the Kp value until you find one that follows the line and gives noticeable oscillation but not really wild ones. We will call this Kp value "Kc" ("critical gain" in the PID literature).
# Using the Kc value as Kp, run the robot along the line and try to determine how fast it is oscillating. This can be tricky but fortunately the measurement doesn't have to be all that accurate. The oscillation period (Pc) is how long it takes the robot to swing from one side of the line to the other then back to the side where it started. For typical Lego robots Pc will probably be in the range of about 0.5 seconds to a second or two.
# You also need to know how fast the robot cycles through it's control loop. I just set the loop to a fixed number of steps (like 10,000) and time how long the robot takes to finish (or have the robot do the timing and display the result.) The time per loop (dT) is the measured time divided by the number of loops. For a full PID controller, written in NXT-G, without any added buzzes or whistles, the dT will be in the range of 0.015 to 0.020 seconds per loop.
# Use the table below to calculate a set of Kp, Ki, and Kc values. If you just want a P controller then use the line in the table marked P to calculate the "correct" Kp (Ki' and Kd' are both zero). If you want a PI controller then use the next line. The full PID controller is the bottom line.
# If you have implemented that the K's are all entered as 100 times their actual value you don't have to take that into account in these calculations. That factor of 100 is already take into account in the Kp = Kc value you determined.
# Run the robot and see how it behaves.
# Tweak the Kp, Ki and Kd values to get the best performance you can. You can start with fairly big tweaks, say 30% then try smaller tweaks to get the optimal (or at least acceptable) performance.
# Once you have a good set of K's try to boost the Tp value, which controls the robot's straight speed.
# Re-tweak the K's or perhaps even go back to step 1 and repeat the entire process for the new Tp value.
# Keep repeating until the robot's behavior is acceptable.
# Ziegler–Nichols method giving K' values
# (loop times considered to be constant and equal to dT)
# Control Type	   Kp	    Ki'	        Kd'
# P	            0.50Kc	    0	0
# PI	        0.45Kc	1.2KpdT/ Pc	    0
# PD	        0.80Kc	0	        KpPc /(8dT)
# PID	        0.60Kc	2KpdT / Pc	KpPc / (8dT)
# The primes (apostrophes) on the Ki' and Kd' are just to remind you that they are calculated assume dT is constant and dT has been rolled into the K values.
#
# Here are the values I measured for my test robot (the one in the video linked later on). Kc was 300 and when Kp=Kc the robot oscillated at about 0.8 seconds per oscillation so Pc is 0.8. I measured Pc by just counting out loud every time the robot swung fully in a particular direction. I then compared my perception of how fast I was counting with "1-potato -- 2-potato -- 3-potato ...". That's hardly "precision engineering" but it works well enough so we'll call it "practical engineering". The loop time, dT, is 0.014 seconds/loop determined by simply running the program for 10,000 loops and having the NXT display the run time. Using the table above for a PID controller we get;
#
# Kp = (0.60)(Kc) = (0.60)(300) = 180
# Ki = 2(Kp)(dT) / (Pc) = 2(180)(0.014) / (0.8) = 6.3 (which is rounded to 6)
# Kd = (Kp)(Pc) / ((8)(dT)) = (180)(0.8) / ((8)(0.014)) = 1286




#######################################################################################################
# time class to measure events
class myTimer(object):
    """[timer class to measure events]

    Arguments:
        object {[type]} -- [description]
    """
    created=time.perf_counter()
    started=time.perf_counter()
    stopped=time.perf_counter()
    mark=time.perf_counter()
    dt=0.01
    lastDuration=0
    
    def __init__(self, dt=0.01):
        """[Αρχικοποίηση της κλάσης timer]

        Keyword Arguments:
            dt {float} -- [το βήμα του χρονικού βήματος] (default: {0.01})
        """
        self.dt=dt
        self.created=time.perf_counter()
        self.mark=self.created

    def Setdt(self,dt=0.01):
        """[summary]

        Args:
            dt (float, optional): [description]. Defaults to 0.01.
        """
        self.dt=dt

    def Start(self):
        """[Έναρξη της λειτουργίας μέτρησης χρόνου]
        """
        self.started=time.perf_counter()
        self.mark=self.started
    
    def Stop(self):
        """[Ολοκλήρωση της διαδικασίας μέτρησης χρόνου. Επιστρέφει την διάρκεια της χρονομέτρησης]
        """
        self.stopped=time.perf_counter()
        self.lastDuration=self.GetDuration()
        return(self.lastDuration)
    
    def GetDuration(self):
        """[Επιστρέφει τον χρόνο της τελευταίας χρονομέτρησης]
        """
        return(self.stopped-self.started)

    def Step(self):
        """[Περιμένει τουλάχιστον για ένα βήμα του μετρητή, (δηλαδή για χρόνο dt) απο την τελευταία λειτουργία του]
        """
        t=time.perf_counter()
        remain=self.dt-(t-self.mark)
        if remain>0:
            time.sleep(remain)
        self.mark=time.perf_counter()

    def GetTime(self):
        """[Επιστρέφει τον τρέχον χρόνο του χρονομέτρου]
        """
        return(time.perf_counter()-self.started)
#######################################################################################################
# parent class for all my classes
class legoPart(object):
    """
    Η πατρική κλάση για όλα τις κλάσεις
    Περιέχει τις κοινές μεθόδους για όλες τις κλάσεις 
    """
    name=''
    #pid=myPID()
    error = 0
    derivative = 0
    lastError = 0
    integral = 0
    # Μετρητής για τις διαδικασίες χρόνου του μοτέρ
    timer=myTimer()
    # κλάση PID διόρθωσης σφαλμάτων 
    pid=myPID()
    # το πατρικό αντικείμενο
    parent=None
    
    def SetParent(self,thisObject):
        self.parent=thisObject
        if self.logger!=None:
            if self.logger.out!='':
                thisObject.logger.out+=self.logger.out



    def Scale_0_100(self,value,minValue,maxValue):
        """
        Επιστρέφει την ανοιγμένη τιμή απο την κλίμακα minValue-maxValue στην κλίμακα 0-100
        
        Parameters:
            value (float): Η τιμή προς μετατροπή.
            minValue (float): Το κάτω όριο της αρχικής κλίμακας.
            value (float): Το πάνω όριο της αρχικής κλίμακας.
        
        Return:
            float: Η ανοιγμένη τιμή στην κλίμακα 0-100 
        """
        value2 = (value - minValue) * 100.0 / (maxValue - minValue)
        return(value2)
    
    def ListToStr(self, list):
        """
        Μετατρέπει μια λίστα σε κείμενο

        Parameters:
            list: η λίστα για μετατροπή
        
        return: 
            str: το κείμενο που αντιστοιχεί στην λίστα
        """
        str1 = '[' + ', '.join(str(e) for e in list) + ']'
        return (str1)

    def LimitBetween(self, minValue=-100, maxValue=100, values=[110, 90]):
        """
        Περιορίζει τις τιμές τις λίστας στα όρια minValue-maxValue

        Parameters:
            minValue (int):  Το κάτω αποδεκτό όριο
            maxValue (int):  Το πάνω αποδεκτό όριο
        return: 
            list: νέες τιμές εντός των ορίων minValue-maxValue
        """
        val = values
        if val[0] > maxValue:
            val[1] = val[1] - (val[0] - maxValue)
            val[0] = maxValue
        if val[1] > maxValue:
            val[0] = val[0] - (val[1] - maxValue)
            val[1] = maxValue
        if val[0] < minValue:
            val[1] = val[1] - (val[0] - minValue)
            val[0] = minValue
        if val[1] < minValue:
            val[0] = val[0] - (val[1] - minValue)
            val[1] = minValue
        return (val)

    def calcTimeShots(self, vStart, vMax, vFinish, s1, s2, s3):
        """[summary]

        Args:
            vStart ([type]): [description]
            vMax ([type]): [description]
            vFinish ([type]): [description]
            s1 ([type]): [description]
            s2 ([type]): [description]
            s3 ([type]): [description]

        Returns:
            [type]: [description]
        """
        t1 = abs(2.0 * s1 / (vStart+vMax))
        t2 = abs(s2 / vMax)
        t3 = abs(2.0 * s3 / (vMax + vFinish))
        return [t1,t2,t3]
        
    def calcDistances(self,vStart,vMax,vFinish,t1,t2,t3):
        """[summary]

        Args:
            vStart ([type]): [description]
            vMax ([type]): [description]
            vFinish ([type]): [description]
            t1 ([type]): [description]
            t2 ([type]): [description]
            t3 ([type]): [description]

        Returns:
            [type]: [description]
        """
        s1 = vStart*t1 +  (vMax-vStart) * t1 / 2.0
        s2 = vMax * t2
        s3 = vMax*t3 +  (vFinish-vMax) * t3  / 2.0
        return [s1,s2,s3]

    def CalcDistanceAndSpeed(self, vStart=0, vMax=40, vFinish=20, s1=0, s2=0, s3=0, t=0):
        """
        Υπολογίζει απόσταση, ταχύτητα σε συγκεκριμμένο χρόνο βάσει συναρτήσεων.

        Parameters:
            vStart: η ταχύτητα την στιγμή της εκκίνησης της ομαλάς επιταχυνόμενης κίνησης (χρόνος t=0)
            vMax: η ταχύτητα στο τέλος της ομαλά επιταχυνόμενης κίνησης και κατα την διάρκεια της ομαλής κίνησης.
            vFinish: η ταχύτητα κατα το τέλος της ομαλά επιβραδυνόμενης κίνησης
            s1: Το διάστημα (σε μοίρες) για το οποίο εφαρμόζεται ομαλά επιταχυνόμενη κίνηση
            s2: Το διάστημα (σε μοίρες) για το οποίο εφαρμόζεται ομαλή κίνηση
            s3: Το διάστημα (σε μοίρες) για το οποίο εφαρμόζεται ομαλά επιβραδυνόμενη κίνηση
            t:  Η χρονική στιγμή για την οποία θέλω να υπολογίσω διάστημα και ταχύτητα της κίνησης.

        return: 
            [s,u] (list): επιστρέφεται για την χρόνική στιγμή t, η συνολική απόσταση απο την αρχή της κίνησης s, και η ταχύτητα u 
        """
        s, u = 0, 0
        if vMax < 0:
            s1 = -s1
            s2 = -s2
            s3 = -s3
        if s1==0:
            t1=0
        else:
            t1 = 2 * s1 / (vStart + vMax)
            a = (vMax-vStart) / t1        
        if s2==0:
            t2=0
        else:
            t2 = s2 / vMax
        if s3==0:
            t3=0
        else:
            t3 = 2 * s3 / (vFinish + vMax)        
            b = - (vMax - vFinish) / t3
        # print(t1,t2,t3)
        if t <= t1:
            s = vStart * t + 1 / 2 * a * t ** 2
            u = vStart + a * t
        elif t <= t1 + t2:
            s = s1 + vMax * (t - t1)
            u = vMax
        elif t <= t1 + t2 + t3:
            s = s1 + s2 + vMax * (t - t1 - t2) + 1 / 2 * b * (t - t1 - t2) ** 2
            u = vMax + b * (t - t1 - t2)
        else:
            s = s1 + s2 + s3 + vFinish * (t - t1 - t2 - t3)
            u = vFinish
        return [s, u]

    def GetStatus(self):
        """
        Επιστρέφει σε ευρετήριο τις τιμές όλων μεταβλητών της τρέχουσας κλάσης
        
        return: 
            dict: ευρετήριο με τις μεταβλητές και τις τιμές τους της τρέχουσας κλάσης
        """
        t= {k:v for k, v in self.__dict__.items() if not (k.startswith('__') and k.endswith('__') )}
        #self.logger.info(t)
        return (t)

    def GetVectorDifference(self, a=[0, 0, 0], b=[0, 0, 0]):
        """[summary]

        Args:
            a (list, optional): [description]. Defaults to [0, 0, 0].
            b (list, optional): [description]. Defaults to [0, 0, 0].

        Returns:
            [type]: [description]
        """
        sum = 0
        a1 = 0
        b1 = 0
        s1 = 0
        for i in range(3):
            sum += a[i] * b[i]
            a1 += a[i] ** 2
            b1 += b[i] ** 2
            s1 += (a[i] - b[i]) ** 2
        a1 = 0.00001 + a1 ** 0.5
        b1 = 0.00001 + b1 ** 0.5
        return int(100 - 100 * sum / a1 / b1), int(s1 ** 0.5)

    def ConvertToPercent(self, list):
        """
        Converts a list of numbers in a list of percents
        :param list: the list of numbers to convert to percents
        :return: the list of percents of numbers
        """
        
        l1 = []
        for item in list:
            l1.append(round(10000 * item / (sum(list) + 0.0000001)) / 100.0)
        return (l1)

    def ToString(self,value):
        """[summary]

        Args:
            value ([type]): [description]
        """
        #print(value,type(value))
        if value==None:
            return('None')
        elif type(value) in (int,float):
            return(str(value))
        elif type(value) in (list,tuple):
            s='['
            i=0
            for item in value:
                if i>0:
                    s+=','                
                s+=self.ToString(item)
                i+=1
            s+=']'
            return(s)
        elif type(value)==bool:
            if value:
                return('True')
            else:
                return('False')
        else:
            return(value)

    def Print(self,*args):
        """[summary]
        """
        s=''
        for val in args:
            try:
                s+=self.ToString(val)+' '
            except Exception as ex:
                self.logger.Error(repr(ex))
                brick.light(Color.YELLOW)

        self.logger.Out(s)    

#######################################################################################################
# my ColorSensor class
class myColorSensor(legoPart):
    """
    Η κλάση που χειρίζεται τον αισθητήρα χρώματος

    Attributes: 
        name (str): Το όνομα του αντικειμένου. 
        mins (dict): Οι τιμές ελαχίστου για αντανάκλαση και RGB
        maxs (dict): Οι τιμές μεγίστου  για αντανάκλαση και RGB
        originalValue (float): Η τελευταία τιμή της αντανάκλασης
        value (float): Η ανοιγμένη τιμή της τελευταίας αντανάκλασης (κλίμακα 0-100)
    """
    # η μεταλητή initialized γίνεται αληθής όταν ολοκληρωθεί επιτυχώς η αρχικοποίηση του αισθητήρα
    initialized=False
    colorNames = ['no color', 'black', 'blue', 'green', 'yellow', 'red', 'white','brown']
    # στα επόμενα ευρετήρια καταγράφονται μέγιστες και ελάχιστες τιμές για
    # αντανάκλαση (white) και red, green, blue του αισθητήρα χρώμματος.
    maxs={'white':0,'red':0,'green':0,'blue':0}
    mins={'white':100,'red':255,'green':255,'blue':255}
    # μεταβλητή που χρησιμοποιείτε για συγχρονισμό των threads που συμμετέχει ο αισθητήρας
    actionDone = False
    # Όταν η μεταβλητή smartAdjust είναι αληθής, γίνεται αναγωγή των τιμών R,G,B,
    # στην κλίμακα 0-100 (βάσει των τιμών min,max των RGB)
    smartAdjust=False
    # Η τελευταία ανοιγμένη τιμή της αντανάκλασης (στην κλίμακα 0-100)
    value = 0
    # στοίβα με αντανακλάσεις (ώθηση μέσω μεθόδου Push)
    lightStack=[]
    # Η τελευταία πραγματική τιμή της αντανάκλασης (όπως διαβάστηκε απο τον αισθητήρα)
    originalValue = 0
    # η λίστα με τις τελευταίες τιμές αντανάκλασης που διαβάστηκαν απο τον αισθητήρα χρώμματος
    values=[]
    # η λίστα με τις τελευταίες τιμές RGB και Color που διαβάστηκαν και αναγνωρίστηκαν
    # απο τον αισθητήρα χρώμματος
    RGBCvalues=[]
    colors = []
    colorsRGB = {2:[0,0,0],3:[0,0,0],4:[0,0,0],5:[0,0,0]}
    
    #minValue = 100
    #maxValue = 0
    # η τιμή όριο μεταξύ άσπρου και μαύρου (πραγματική και όχι ανοιγμένη τιμή).
    medValue = 50
    #minRGBValue = 0
    #maxRGBValue = 10
    #medRGBValue = 5
    # το αντικείμενο sensor της Lego Micropython
    sensor=None
    # Λογική μεταβλητή για τον έλεγχο της ολοκλήρωσης του νήματος
    # ενημερώνεται στην scanForColors,ScanRGB
    threadCompleted=True

    i=0
    j=0
    # οι τελευταίες τιμές των αντανακλάσεων RGB
    red = 0
    green = 0
    blue = 0
    # η τελευταία τιμή RGB που διαβάστηκε απο τον αισθητήρα
    RGB = []
    # η τελευταία τιμή RGB (χωρίς αναγωγή) που διαβάστηκε απο τον αισθητήρα
    originalRGB = []
    
    # Το τελευταίο Χρώμα που διαβάστηκε απο τον αισθητήρα
    color=0
    # στοίβα με χρώμματα (ώθηση μέσω μεθόδου Push)
    colorStack=[]
    # χρησιμοποιείτε όταν υπάρχουν πολλαπλές αναγνώσεις RGB για την παραγωγή
    # μιας τελικής τριάδας RGB
    RGB_Read_Set=[]
    # Λογικές μεταβλητές για τον τύπο του αισθητήρα
    isEv3=False
    isNxt=False
    isHt=False
    # Το ακόλουθο ευρετήριο  μετατρέπει τα χρώματα του ht σε αυτά του ev3
    ht2Ev3={0:0,1:7,2:2,3:2,4:3,5:4,6:4,7:5,8:5,9:5,10:5,11:6,12:6,13:6,14:6,15:6,16:6,17:6}
   
    # Η πατρική κλάση
    parent=None
    # μετρητής λαθών και προβλημάτων
    errors=0
    # το τελευταίο καταγεγραμμένο μήνυμα
    lastMessage=''


    def __init__(self, port=0, sensorType='ev3', loggingLevel=3,
        mins={'white':100,'red':255,'green':255,'blue':255}, 
        maxs={'white':0,'red':0,'green':0,'blue':0}):
        """
        Αρχικοποιεί το αντικείμενο Αισθητήρα Χρώμματος.

        :parameters
            port (int): The port where sensor is connected. Choose a value from 1,2,3,4.
            sensorType (str): Choose  'Lego' or 'nxt' or 'ht'
            loggingLevel (int): choose one of the following 1 (debug) 2 (info), 3 (warning), 4 (error), 5 (critical)      
            mins (dict): min values for manual calibration
            maxs (dict): max values for manual calibration
            
        :return
            ColorSensor
        """
        #https://github.com/ev3dev/ev3dev/issues/1292
        self.lastMessage=''
        self.color=0
        if port in (1,2,3,4):
            self.i=0
            self.port = port
            self.name = 'color'+str(port)
            self.maxs=maxs.copy()
            self.mins=mins.copy()
            self.values=[]
            self.RGBCvalues=[]
            self.colorStack=[]
            self.lightStack=[]
            
            self.logger = myLogger(name=self.name,filename='log.txt',level=loggingLevel)
            self.logger.parent=self
            # self.logger.Debug(self.name+' initized')
            self.sensorType = sensorType
            if maxs['red']+maxs['green']+maxs['blue']==0:
                self.smartAdjust=False
            else:
                self.smartAdjust=True
            if port==1:
                myPort=Port.S1
            elif port==2:
                myPort=Port.S2
            elif port==3:
                myPort=Port.S3
            elif port==4:
                myPort=Port.S4
            else:
                self.logger.Error('The port number '+str(port)+' is not defined')
            try:
                if self.sensorType.upper() in ('LEGO','EV3'):
                    self.sensor=ColorSensor(myPort)
                elif self.sensorType.upper().startswith('NXT'):
                    self.sensor=lightSensor(myPort)
                elif self.sensorType.upper().startswith('HT'):
                    self.sensor=htColorSensor(myPort)
                self.initialized = True
                # self.logger.Debug('ok')
                self.logger.Out('initialized ok')
            except Exception as e:
                self.logger.Out('Error in Initialize Color Sensor on Port '+str(port)+'\n\t'+repr(e))
                brick.light(Color.RED)
                self.errors+=1
                

            
        # create bools for sensortype
        if sensorType.upper() in ('LEGO','EV3'):
            self.isEv3=True
        elif sensorType.upper().startswith('NXT'):
            self.isNxt=True
        elif sensorType.upper().startswith('HT'):
            self.isHt=True
           
    def Beep(self):
        """[summary]
        """
        thread1 = threading.Thread(target=brick.sound.beep,kwargs={})
        thread1.start()

    def PushColor(self):
        self.colorStack.append(self.color)

    def PushLight(self):
        self.lightStack.append(self.value)


    def ColorFromRGB(self, RGB, fromColorList=[2, 3, 4, 5], lightThreshold=10,
                     colorsRGB={0:[0, 0, 0], 1:[18, 18, 19], 2:[34, 86, 127], 3:[36, 42, 41], 4:[289, 185, 54], 5:[190, 41, 36],
                                6: [0, 0, 0], 7:[370, 407, 272]}):
        """
        Recognize the closest color of giver RGB values
        :param RGB: The RGB Values
        :param fromColorList: The List of clolor to choose from
        :param lightThreshold: the minimum light to accept the color (otherwise is 'no color')
        :return: the closest color from Color List
        """
        
        pos = 0
        d = 999999999999999999999999999
        if sum(RGB) > lightThreshold:
            for i in fromColorList:
                d1 = self.GetVectorDifference(self.ConvertToPercent(colorsRGB[i]), self.ConvertToPercent(RGB))[1]
                if d1 < d:
                    d = d1
                    pos = i
        return (pos)

    def ColorFromRGB2(self, RGB, fromColorList=[1, 2, 3, 4, 5, 6], lightThreshold=10,
                      colorsRGB={0:[0, 0, 0], 1:[18, 18, 19], 2:[34, 86, 127], 3:[36, 42, 41], 4:[289, 185, 54], 5:[190, 41, 36],
                                 6:[370, 407, 272]}):
        """
        Recognize the closest color of giver RGB values
        :param RGB: The RGB Values
        :param fromColorList: The List of clolor to choose from
        :param lightThreshold: the minimum light to accept the color (otherwise is 'no color')
        :return: the closest color from Color List
        """
        pos = 0
        d = 999999999999999999999999999
        if sum(RGB) > lightThreshold:
            for i in fromColorList:
                if RGB == colorsRGB[i]:
                    return (pos)
                else:
                    d1 = self.GetVectorDifference(RGB, colorsRGB[i])[0]
                    if d1 < d:
                        d = d1
                        pos = i
            if pos in [1, 6]:
                d2 = self.GetVectorDifference(RGB, colorsRGB[1])[1]
                d3 = self.GetVectorDifference(RGB, colorsRGB[6])[1]
                if d2 < d3:
                    pos = 1
                else:
                    pos = 6
        return (pos)

    def GetLight(self,color='white',adjusted=True):
        """        
        Διαβάζει την τιμή της αντανάκλασης από τον αισθητήρα χρώμματος.

        :parameters 
            color (string): Ο τύπος της αντανάκλασης που επιθυμώ να διαβάσω. ('white', 'any', 'red', 'green', 'blue') 
            adjusted (bool): αν είναι True θα γίνει αναγωγή της αντανάκλαση στην κλίμακα 0-100.
        
        :return 
            light (float): η αντανάκλαση του αισθητήρα 
            Αν adjusted = True η τιμή είναι με αναγωγή στην κλίμακα 0-100
            διαφορετικά είναι η πραγματική τιμή του αισθητήρα 
        """
  
        # self.originalValue=self.sensor.reflection()        
        if self.isEv3:
            self.originalValue=self.sensor.reflection()
            # if color=='white':
            #     self.originalValue=self.sensor.reflection()                                   
            # elif color=='any':
            #     col=self.sensor.color()
            #     if col==6:
            #         self.originalValue=100
            #         self.value=100
            #         return(100)
            #     else:
            #         self.originalValue=0
            #         self.value=0
            #         return(0)
                
            # elif color=='red':
            #     rgb=self.sensor.rgb()
            #     self.originalValue=rgb[0]
            # elif color=='green':
            #     rgb=self.sensor.rgb()
            #     self.originalValue=rgb[1]
            # elif color=='blue':
            #     rgb=self.sensor.rgb()
            #     #red = rgb[0]
            #     #green = rgb[1]
            #     #blue = rgb[2]
            #     self.originalValue=rgb[2]
        elif self.isNxt:
            self.originalValue=self.sensor.reflection() 
        elif self.isHt:
            self.originalValue = self.sensor.rgbw()[3]
        

        if (self.originalValue > self.maxs[color]):
            self.maxs['white'] = self.originalValue
        if (self.originalValue < self.mins[color]):
            self.mins['white'] = self.originalValue
        self.medValue = (self.maxs[color] + self.mins[color]) / 2.0

        if self.mins[color] == self.maxs[color]:
            self.value = 48
            # self.value=self.originalValue
        else:
            self.value = (self.originalValue - self.mins[color]) * 100.0 / (self.maxs['white'] - self.mins['white'])
        self.i +=1
        if len(self.values)<9:
                self.values.append(self.value)
        else:
            self.values[self.i % 9]=self.value
        # self.lastMessage= 'GetLight: Original Value='+str(self.originalValue)+' Value='+str(self.value)   
            
        if self.logger.level<3:
            ss1='GetLight: Original Value='+str(self.originalValue)+' Value='+str(self.value)
            self.logger.Info(ss1)
        if adjusted:
            return (self.value)
        else:
            return(self.originalValue)

    def OnLightChange(self,percent=30):        
        self.GetLight()
        try:
            ma=max(self.values)
            mi=min(self.values)
            k=(ma-mi)/(mi+1)
            # self.Print(k)
            # time.sleep(1)
            return(k>=percent/100)
        except:
            return(False)

    def GetRGB(self, readCount=1,threshold=1,adjusted=True):
        """
        Διαβάζει τις τιμές RGB από τον αισθητήρα χρώμματος.

        :parameters 
            readCount (int): Το πλήθος των αναγνώσεων.
            threhold (int): Τιμή πάνω απο την οποία γίνεται καταγραφή του RGB στο ιστορικό
        
        :return 
            RGB (list): [ sum(RED), sum(GREEN), sum(BLUE) ]
        """
        # self.lastMessage=''
        self.red = 0
        self.green = 0
        self.blue = 0
        self.RGB_Read_Set = []
        if self.isEv3:
            for i in range(readCount):
                rgb=self.sensor.rgb()
                red = rgb[0]
                green = rgb[1]
                blue = rgb[2]
                #calc maxs
                if red>self.maxs['red']:
                    self.maxs['red']=red
                if green>self.maxs['green']:
                    self.maxs['green']=green
                if blue>self.maxs['blue']:
                    self.maxs['blue']=blue
                #calc mins
                if red<self.mins['red']:
                    self.mins['red']=red
                if green<self.mins['green']:
                    self.mins['green']=green
                if blue<self.mins['blue']:
                    self.mins['blue']=blue
                self.red += red
                self.green += green
                self.blue += blue
                if self.logger.level<3:
                    self.RGB_Read_Set.append([red, green, blue])
        elif self.isHt:
            # the following loop is to ensure right rgb values
            # for i in range(3):
            #     rgbw=self.sensor.rgbw()
            for i in range(readCount):
                rgbw=self.sensor.rgbw()
                red = rgbw[0]
                green = rgbw[1]
                blue = rgbw[2]
                #calc maxs
                if red>self.maxs['red']:
                    self.maxs['red']=red
                if green>self.maxs['green']:
                    self.maxs['green']=green
                if blue>self.maxs['blue']:
                    self.maxs['blue']=blue
                #calc mins
                if red<self.mins['red']:
                    self.mins['red']=red
                if green<self.mins['green']:
                    self.mins['green']=green
                if blue<self.mins['blue']:
                    self.mins['blue']=blue
                self.red += red
                self.green += green
                self.blue += blue
                if self.logger.level<3:
                    self.RGB_Read_Set.append([red, green, blue])
        else:
            self.logger.Error(self.sensorType+' sensorType does not support rgb mode')
        self.originalRGB = [self.red/readCount, self.green/readCount, self.blue/readCount]
        if self.smartAdjust and self.maxs['red']>self.mins['red']  and self.maxs['green']>self.mins['green']  and self.maxs['blue']>self.mins['blue']:
            r=(self.red/readCount - self.mins['red']) * 100.0 / (self.maxs['red'] - self.mins['red'])
            g=(self.green/readCount - self.mins['green']) * 100.0 / (self.maxs['green'] - self.mins['green'])
            b=(self.blue/readCount - self.mins['blue']) * 100.0 / (self.maxs['blue'] - self.mins['blue'])
            self.RGB = [r, g, b]
            
        else:
            self.RGB = self.originalRGB
            
        if self.logger.level<3:
            self.j +=1
            if sum(self.RGB)>=threshold:
                if len(self.RGBCvalues)<100:
                    self.RGBCvalues.append(self.RGB)
                else:
                    self.RGBCvalues[self.j % 100]=self.RGB
                    self.RGBCvalues[(self.j+1) % 100]='-->|'
            ss1='GetRGB(): '+str(self.RGB)
            self.logger.Info(ss1)
        # else:
        #     self.lastMessage='GetRGB(): '+str(self.RGB)
        return (self.RGB)

    def GetColor(self, method=1,fromColorList=[2,3,4,5],readCount=1,forRGB=[],lightThreshold=5,yellowThreshold = 50,redThreshold = 79,colorsRGB={0:[0, 0, 0], 1:[18, 18, 19], 2:[34, 86, 127], 3:[36, 42, 41], 4:[289, 185, 54], 5:[190, 41, 36],6:[370, 407, 272]},beep=False):
        '''       
        Υπολογίζω το χρώμα για τις τιμές RGB (αν δίνονται) αλλιώς που διαβάζονται απο τον αισθητήρα.

        :parameters 
            method (integer):   1 αναγνωρίζει τα χρώμματα 2,3,4,5. 
                                2 αναγνωρίζει όλα τα χρώμματα (απαιτεί ColorsRGB) 
                                3.Αναγνώριση χρώματος με ενσωματωμένη μέθοδο micropython
            readCount (integer): Το πλήθος των αναγνώσεων.
            forRGB (list): Οι τιμές RGB για τις οποίες υπολογίζω το χρώμα. 
                           Αν είναι κενή λίστα διαβάζω τιμές με την GetRGB() 
            lightThreshold: Η τιμή κάτω απο την οποία επιστρέφεται noColor
            colorsRGB: Ευρετήριο με τα RGB των χρωμματιστών κύβων
        
        :return 
            color (integer): Το χρώμα που αναγνωρίστηκε. 
        '''
        if len(forRGB)>0:
            rgb=forRGB
        elif method==3:
            pass
        else:
            rgb = self.GetRGB(readCount=readCount)
            if sum(rgb)<lightThreshold:
                self.color=0
                return(self.color)
                
        if method==1:            
            minRGB = min(rgb)
            o3 = rgb[0] - minRGB
            o4 = rgb[1] - minRGB
            o5 = o3 / (o3 + o4 + 0.0000001) * 100
            self.color=0
            if o5 <= yellowThreshold:
                if rgb[1] >= rgb[2]:
                    self.color=3
                else:
                    self.color = 2
            elif o5 <= redThreshold:
                self.color = 4
            else:
                self.color = 5
            
            if self.color in (2,3,4,5):
                if sum(rgb)>sum(self.colorsRGB[self.color]):
                    self.colorsRGB[self.color]=rgb
        elif method==2:
            o5=-1
            self.color=self.ColorFromRGB2(rgb,lightThreshold=lightThreshold,fromColorList=fromColorList,colorsRGB=colorsRGB)
        elif method==3:
            self.color=0
            if self.isEv3:
                self.color=self.sensor.color()
                if self.color==None:
                    self.color=0 
            elif self.isHt:        
                color1=self.sensor.color()
                if color1==None:
                    color1=0 
                self.color=self.ht2Ev3[color1]
                if self.logger.level<3:
                    ss1='Original HT color: '+str(color1)
                    self.logger.Debug(ss1)
            else:
                self.Print('Only ev3 and ht supports color detection')
            if self.logger.level<3:
                ss1='GetColor(): '+str(self.color)
                self.lastMessage=ss1
                self.logger.Info(ss1)
            else:
                pass
                # s=self.name+'.GetColor(method='+str(method)+',readCount='+str(readCount)+')\n'
                # s+='\tRGB:'+self.ListToStr([self.red,self.green,self.blue])
                # s+=' color:'+str(self.color)+' '
                # s+=self.colorNames[self.color]
                # self.lastMessage=s
               
            return(self.color)
        # elif method==4:            
        #     self.color=self.GuessColor(rgb=rgb,fromColors=fromColorList)
        #     return(self.color)
        elif method==4:
            self.color=self.GuessColor(rgb=rgb)
            return self.color

        if self.logger.level<3:
            s=self.name+'.GetColor(method='+str(method)+',readCount='+str(readCount)+')\n'
            s+='\tRGB:'+self.ListToStr(rgb)+' o5:'+str(o5)+' color:'+str(self.color)+' '+self.colorNames[self.color]
            self.logger.Debug(log=s)
            self.j +=1
            if len(self.RGBCvalues)<100:
                self.RGBCvalues.append(self.color)
            else:
                self.RGBCvalues[self.j % 100]=self.color
                self.RGBCvalues[(self.j+1) % 100]='-->|'
            #return (self.RGB)
            ss1='GetColor(): '+str(self.color)
            self.logger.Info(ss1)
            # self.lastMessage=s+ss1
        else:
            pass
            # s=self.name+'.GetColor(method='+str(method)+',readCount='+str(readCount)+')\n'
            # s+='\tRGB:'+self.ListToStr(rgb)+' o5:'+str(o5)+' color:'+str(self.color)+' '+self.colorNames[self.color]
            # self.lastMessage=s
        if beep and self.color>0:
            self.Beep()
        return self.color

    def GuessColor(self,rgb=[0,0,0],fromColors=[2,3,4,5],mem=[
         [30.26527674,-11.56932035,-1.9742275,-25.53514191,0.79092679,63.66678566],
         [-65.87950933,-9.06991468,-38.37557288,-3.38484566,20.10769047,11.67465409],
         [-10.07628349,-3.65444225,48.84689961,36.6800622,-42.01086579,-60.47810759],
         [2.56607179,17.75068146,-21.15853989,-17.82066103,2.24615626,-115.52829999]]):


        red=rgb[0]
        green=rgb[1]
        blue=rgb[2]
        max1=red
        min1=red
        if green>max1:
            max1=green
        if green<min1:
            min1=green
        if blue>max1:
            max1=blue
        if blue<min1:
            min1=blue
            
        if max1>0:
            in0 = min1/max1
        else:
            return(0)
        sum1=sum(rgb)
        if sum1!=0:
            in1 = rgb[0]/sum1
            in2 = rgb[1]/sum1
            in3 = rgb[2]/sum1
        else:
            return(0)
        prc=[0,0,0,0,0,0,0]
        max=-999999999999
        color=0
        guessColor=0
        for color in fromColors:
            i=color-1
            prc[color]=in0*mem[0][i]+in1*mem[1][i]+in2*mem[2][i]+in3*mem[3][i]
            # print(color,prc[color],max)
            if prc[color]>max:
                max=prc[color]
                guessColor=color
        # print(rgb,in0,in1,in2,in3)
        # print('color=',guessColor,' % sure:',max)
        return (guessColor)


    def CheckColor(self,lightThreshold=1,yellowThreshold = 50,redThreshold = 79):
        """[summary]

        Args:
            lightThreshold (int, optional): [description]. Defaults to 1.
            yellowThreshold (int, optional): [description]. Defaults to 50.
            redThreshold (int, optional): [description]. Defaults to 79.
        """
        time.sleep(0.5)
        self.smartAdjust=False
        while not any(brick.buttons()) :
            rgb = self.GetRGB(threshold=lightThreshold,readCount=3)
            t1Start=time.perf_counter()
            color1=self.GetColor(forRGB=rgb,lightThreshold=lightThreshold)
            t1Stop=time.perf_counter()
            
            t2Start=time.perf_counter()
            color2=self.ColorFromRGB(rgb,lightThreshold=lightThreshold)
            t2Stop=time.perf_counter()
            
            t3Start=time.perf_counter()
            color3=self.ColorFromRGB2(rgb,lightThreshold=lightThreshold)
            t3Stop=time.perf_counter()
            
            print(color1,color2,color3, rgb,'dt1:',round(t1Stop-t1Start,3),'dt2:',round(t2Stop-t2Start,3),'dt3:',round(t3Stop-t3Start,3))
            time.sleep(0.5)
    
    def GetStatus(self):
        """
        Επιστρέφει κείμενο με λεπτομέρειες της κατάστασης του αισθητήρα χρώμματος

        :return
            (str):
        """
        s= '\n-----------------------------------------------------------'
        s+='\n| Status '+self.name+' ('+self.sensorType+')'
        s+='\n| white:\t'+str(self.mins['white'])+'-'+str(self.maxs['white'])
        s+='\n| Adj. Light Value:\t'+str(self.value)
        s+='\n| Orig.Light Value:\t'+str(self.originalValue)
        if self.sensorType.upper() !='NXT':
            s+='\n| red  :\t'+str(self.mins['red'])+'-'+str(self.maxs['red'])
            s+='\n| green:\t'+str(self.mins['green'])+'-'+str(self.maxs['green'])
            s+='\n| blue :\t'+str(self.mins['blue'])+'-'+str(self.maxs['blue'])
            s+='\n| RGB  :\t'+str(self.RGB)
            s+='\n| color  :\t'+str(self.color)
        s+='\n| values :\t'+self.ListToStr(self.values)
        s+='\n| RGB :\t'+self.ListToStr(self.RGB)        
        s+='\n| originalRGB :\t'+self.ListToStr(self.originalRGB)        
        s+='\n| RGBCvalues :\t'+self.ListToStr(self.RGBCvalues)        
        s+='\n| colorStack :\t'+self.ListToStr(self.colorStack)        
        s+='\n| lightStack :\t'+self.ListToStr(self.lightStack)        
        if self.sensorType.upper() !='NXT':
            s+='\n| colorsRGB :\t'+'{ 2:'+self.ListToStr(self.colorsRGB[2])+' 3:'+self.ListToStr(self.colorsRGB[3])
            s+=' 4:'+self.ListToStr(self.colorsRGB[4])+' 5:'+self.ListToStr(self.colorsRGB[5])+' }'
        s+='\n| lastMessage :\t'+self.lastMessage  
        s+='\n----------------------------------------------------------'
        # print(s)
        return(s)

    def PrintStatus(self,comments=''):
        s=comments+'\n'+self.GetStatus()
        self.Print(s)

    def Test0(self,tests=['white','rgb','color']):
        """
        Εφαρμόζει 1000 δοκιμές σε αντανάκλαση, ανάγνωση τιμών RGB και αναγνώριση χρώματος 
        Εμφανίζει στατιστικά απόδοσης (dt) για κάθε επιμέρους λειτορυργία

        Parameters:
            tests (list): Η λίστα με τις δοκιμές που θέλουμε να συμπεριληφθούν. ['white','rgb','color']  
        
        return:
            None
        """
        if 'white' in tests:
            l1=self.logger.level
            self.logger.level=10
            t0=time.perf_counter()
            for i in range(1000):
                self.GetLight()
            t1=time.perf_counter()
            self.logger.level=l1
            self.logger.Out('GetLight dt:'+str((t1-t0)/1000))
        if 'rgb' in tests:
            l1=self.logger.level
            self.logger.level=10
            t0=time.perf_counter()
            for i in range(1000):
                self.GetRGB()
            t1=time.perf_counter()
            self.logger.level=l1
            self.logger.Out('GetRGB dt:'+str((t1-t0)/1000))
        if 'color' in tests:
            l1=self.logger.level
            self.logger.level=10
            t0=time.perf_counter()
            for i in range(1000):
                self.GetColor()
            t1=time.perf_counter()
            self.logger.level=l1
            self.logger.Out('GetColor dt:'+str((t1-t0)/1000))

    def GenerateInitString(self):
        """[summary]
        """
        s=  "\nr.color"+str(self.port)+"=myLib.myColorSensor(port="+str(self.port)+",sensorType='"+self.sensorType
        s +="',mins={'white':"+str(self.mins['white'])+",'red':"+str(self.mins['red'])+",'green':"+str(self.mins['green'])+",'blue':"+str(self.mins['blue'])+"}"
        s +=",maxs={'white':"+str(self.maxs['white'])+",'red':"+str(self.maxs['red'])+",'green':"+str(self.maxs['green'])+",'blue':"+str(self.maxs['blue'])+"})"
        return(s)

    # def Print(self,str="ColorSensor Print"):
    #     self.logger.Info(self.ToString(str))

    def __ScanForColors(self, seconds=120, dt=0.01):
        """[summary]

        Args:
            lightThreshold (int, optional): [description]. Defaults to 30.
            seconds (int, optional): [description]. Defaults to 120.
            fromColorList (list, optional): [description]. Defaults to [2,3,4,5].
            dt (float, optional): [description]. Defaults to 0.01.
        """
        start = time.time()
        self.threadCompleted=False
        # self.Beep()
        #self.results = []
        # maxSum = 0
        # rgb = [0, 0, 0]
        # maxRGB = rgb
        # sumRGB = sum(rgb)
        # threshold = lightThreshold
        # maxs = [0]
        # maxsRGB = [[0, 0, 0]]
        # index = 0
        self.actionDone = False
        self.colorStack=[]
        while not self.actionDone and time.time() - start < seconds:            
            
            col=self.GetColor(method=3)
            if col not in self.colorStack:
                self.PushColor()

            # rgb = self.GetRGB(readCount=1,threshold=lightThreshold)
            # sumRGB = sum(rgb)
            # if sumRGB> maxs[index]:
            #     maxs[index] = sumRGB
            #     maxsRGB[index] = rgb
            # elif sumRGB < threshold and maxs[index] > threshold:
            #     self.Beep()
            #     index += 1
            #     maxs.append(0)
            #     maxsRGB.append(rgb)
            time.sleep(dt)
        # self.colors=[]
        # self.RGBCvalues=[]
        # for item in maxsRGB:
        #     if type(item)==list:
        #         self.RGBCvalues.append(item)
        #         self.colors.append(self.GetColor(method=1,fromColorList=fromColorList,forRGB=item,lightThreshold=lightThreshold) )
        # self.logger.Warning(str(self.colors))
        # self.logger.Warning(str(self.RGBCvalues))
        self.threadCompleted=True
        
        #self.results.append(self.colors)
        # if self.logging:
        #     self.log +='\n**** ScanForColors(lightThreshold='+str(lightThreshold)+' seconds='+str(seconds)+', dt='+str(dt)+') *'
        #     self.log +='\n* maxs RGB: '+str(maxsRGB)
        #     self.log +='\n* colorsDetected: '+str(self.colors)
        #     self.log +='\n**********************************************'

    def __ScanForColors0(self,lightThreshold=30, seconds=120, fromColorList=[2,3,4,5], dt=0.01):
        """[summary]

        Args:
            lightThreshold (int, optional): [description]. Defaults to 30.
            seconds (int, optional): [description]. Defaults to 120.
            fromColorList (list, optional): [description]. Defaults to [2,3,4,5].
            dt (float, optional): [description]. Defaults to 0.01.
        """
        start = time.time()
        self.threadCompleted=False
        self.Beep()
        #self.results = []
        maxSum = 0
        rgb = [0, 0, 0]
        maxRGB = rgb
        sumRGB = sum(rgb)
        threshold = lightThreshold
        maxs = [0]
        maxsRGB = [[0, 0, 0]]
        index = 0
        self.actionDone = False
        while not self.actionDone and time.time() - start < seconds:            
            rgb = self.GetRGB(readCount=1,threshold=lightThreshold)
            sumRGB = sum(rgb)
            if sumRGB> maxs[index]:
                maxs[index] = sumRGB
                maxsRGB[index] = rgb
            elif sumRGB < threshold and maxs[index] > threshold:
                self.Beep()
                index += 1
                maxs.append(0)
                maxsRGB.append(rgb)
            time.sleep(dt)
        self.colors=[]
        self.RGBCvalues=[]
        for item in maxsRGB:
            if type(item)==list:
                self.RGBCvalues.append(item)
                self.colors.append(self.GetColor(method=1,fromColorList=fromColorList,forRGB=item,lightThreshold=lightThreshold) )
        self.logger.Warning(str(self.colors))
        self.logger.Warning(str(self.RGBCvalues))
        self.threadCompleted=True
        
        #self.results.append(self.colors)
        # if self.logging:
        #     self.log +='\n**** ScanForColors(lightThreshold='+str(lightThreshold)+' seconds='+str(seconds)+', dt='+str(dt)+') *'
        #     self.log +='\n* maxs RGB: '+str(maxsRGB)
        #     self.log +='\n* colorsDetected: '+str(self.colors)
        #     self.log +='\n**********************************************'


    def ScanForColors(self,comments="",seconds=120, dt=0.01):
        """[Αναγνώριση χρώμματος με κάποιον απο τους 3 τρόπους αναγνώρισης χρώμματος]

        Args:
            comments (str, optional): [description]. Defaults to "".
            seconds (int, optional): [description]. Defaults to 120.
            dt (float, optional): [description]. Defaults to 0.01.

        Returns:
            [color]: [Το χρώμα που αναγνωρίστηκε]
        """
        self.logger.Warning(comments)
        self.thread = threading.Thread(target=self.__ScanForColors,
                                       kwargs={'seconds': seconds,  'dt': dt})
        self.thread.start()
        return (self.thread)


    def __ScanRGB(self, seconds=5, dt=0.01):
        """[summary]

        Args:
            seconds (int, optional): [description]. Defaults to 120.
            dt (float, optional): [description]. Defaults to 0.01.
        """
        start = time.perf_counter()
        self.threadCompleted=False
        maxSum = 0
        rgb = [0, 0, 0]
        # self.colorStack=[]
        maxRGB = rgb
        sumRGB = sum(rgb)
        self.actionDone = False
        while not self.actionDone and time.perf_counter() - start < seconds:
            rgb = self.GetRGB(readCount=1)
            sumRGB = sum(rgb)
            if sumRGB> maxSum:
                maxSum = sumRGB
                maxRGB=rgb
            # col=self.GetColor(method=3)
            # if col not in self.colorStack:
            #     self.PushColor()
            time.sleep(dt)
        self.RGB =maxRGB
        self.threadCompleted=True
        # self.logger.Warning(str(self.RGB))
        self.threadCompleted=True
        


    def ScanRGB(self,comments="",seconds=5, dt=0.01):
        """[Ανάγνωση RGB στο BackGround. 
        Ολοκλήρωση του Scanning όταν ολοκληρωθεί ο χρόνος ή το Actiondone γίνει True
        ScanRGBResult για να επιστραφεί το αποτέλεσμα του Scanning]

        Args:
            comments (str, optional): [description]. Defaults to "".
            seconds (int, optional): [description]. Defaults to 5.
            dt (float, optional): [description]. Defaults to 0.01.

        Returns:
            [RGB]: [Το μέγιστο RGB set που διαβάστηκε]
        """
        self.logger.Warning(comments)
        self.thread = threading.Thread(target=self.__ScanRGB,
                                       kwargs={'seconds': seconds,  'dt': dt})
        self.thread.start()
        return (self.thread)

    def ScanRGBResult(self):
        while not self.threadCompleted:
            time.sleep(0.01)
        return(self.RGB)

    def ScanColorsResult(self):
        while not self.threadCompleted:
            time.sleep(0.01)
        return(self.colorStack)


    def WaitForColor(self,colors=[2,3,4,5,6],dt=0.005):
        """[Αναμονή για κάποιο χρώμα στον αισθητήρα]

        Args:
            colors (list, optional): [Τα χρώματα στα οποία θα ολοκληρωθεί η αναμονή]. Defaults to [2,3,4,5,6].
            dt (float, optional): [ο χρόνος ανάμεσα στις αναγνώσεις χρώματος]. Defaults to 0.005.
        """
        col=0
        if self.isNxt:
            self.logger.Error('Nxt sensor do not support color recognition')
        else:
            self.timer.Setdt(dt=dt)
            self.timer.Start()
            col=self.GetColor(method=3)
            while col not in colors:
                self.timer.Step()
                col=self.GetColor(method=3)
        return(col)

    def ColorInColors(self,colors=[2,3,4,5,6]):
        
        if self.isNxt:
            self.logger.Error('Nxt sensor do not support color recognition')
            return(False)
        else:
            col=self.GetColor(method=3)
            return(col in colors)

    def Test(self):
        count=100
        s=''
        # getLight()
        t0=time.perf_counter()
        for i in range(count):
            self.GetLight()
        t1=time.perf_counter()
        s='\n-----------------------------\nGetLight dt: '+str((t1-t0)/count)
        if self.sensorType.upper()!='NXT':
            # getColor(method=1)
            t0=time.perf_counter()
            for i in range(count):
                self.GetColor(method=1)
            t1=time.perf_counter()
            s+='\nGetColor(method=1) dt: '+str((t1-t0)/count)
            # getColor(method=2)
            t0=time.perf_counter()
            for i in range(count):
                self.GetColor(method=2)
            t1=time.perf_counter()
            s+='\nGetColor(method=2) dt: '+str((t1-t0)/count)
            # getColor(method=3)
            t0=time.perf_counter()
            for i in range(count):
                self.GetColor(method=3)
            t1=time.perf_counter()
            s+='\nGetColor(method=3) dt: '+str((t1-t0)/count)

            t0=time.perf_counter()
            for i in range(count):
                self.GetColor(method=4)
            t1=time.perf_counter()
            s+='\nGetColor(method=4) dt: '+str((t1-t0)/count)


            t0=time.perf_counter()
            for i in range(count):
                self.GetRGB()
            t1=time.perf_counter()
            s+='\nGetRGB() dt: '+str((t1-t0)/count)
           
            t0=time.perf_counter()
            for i in range(count):
                self.GuessColor(rgb=self.GetRGB())
            t1=time.perf_counter()
            s+='\nGuessColor() dt: '+str((t1-t0)/count)
            s+="\n-----------------------------"

        self.logger.Out(s)
        self.logger.Save()
       
    def Calibrate(self):
        self.logger.SetLevel(3)
        self.Beep()
        self.logger.level=3
        self.smartAdjust=False
        self.mins={'white':100,'red':255,'green':255,'blue':255}
        self.maxs={'white':0,'red':0,'green':0,'blue':0}

        t0=time.perf_counter()
        while not any(brick.buttons()) or not time.perf_counter()-t0>2:
            self.GetLight()
            if self.sensorType.lower()!='nxt':
                self.GetColor()
        s=''
        s2=''
        if self.initialized:
            s+=self.GetStatus()+'\n'
            s2+=self.GenerateInitString()
        
        self.logger.Warning(s)
        self.logger.Warning(s2)
        

#######################################################################################################
class myMotor(legoPart):

    """[summary]

    Args:
        legoPart ([type]): [description]

    Returns:
        [type]: [description]
    """
    # η μεταλητή initialized γίνεται αληθής όταν ολοκληρωθεί επιτυχώς η αρχικοποίηση του μοτέρ
    initialized = False
    # η πόρτα στην οποία συνδέεται το μοτέρ
    port = 'B'
    # string για την καταγραφή πληροφοριών. Έχει αντικατασταθεί απο την κλάση logger
    # log = '-'

    # micropython motor object.
    motor=None
    # η τρέχουσα ισχύς του μοτέρ
    power = 0    
    # χρήση σε περιπτώσεις thread για συγχρονισμό
    actionDone = False
    # για την μετατροπή ισχύος σε ταχύτητα (10 για large motors, 14 για medium motors)
    speedRatio = 10
    # μετρητής μοιρών
    degrees = 0
    # για την καταγρασή του εσωτερικού μετρητή μοιρών του μοτέρ
    lastPosition = 0
    # καταγράφει την στιγμή της αρχικοποίησης του μοτέρ
    # start = time.perf_counter()
    # καταγράφεται το τελευταίο thread που έγινε αρχικοποίηση απο το μοτέρ.
    thread = threading.Thread()
    # η τρέχουσα κατάσταση του μοτέρ 0: Off  1: On 2: Stalled
    state=0
    # Το πλήθος των διαδοχικών φορών που το μοτέρ έχει μηδενική ταχύτητα
    zeroSpeedCount=0
    # Ο χρόνος που απαιτείτε για να χαρακτηριστεί η κατάσταση του μοτέρ ώς κολημένο (stalled)
    stallDetectionTime=0.02
    # Η μέγιστη ταχύτητα που έπιασε το μοτέρ στην τελευταία περίοδο λειτουργίας του
    topSpeed=0
    # Η μέγιστη ισχύς που έπιασε το μοτέρ στην τελευταία περίοδο λειτουργίας του
    topPower=0
    # Η πατρική κλάση
    parent=None
    # Ο χρόνος του τελευταίου Reset
    resetTime=0
    #  Μετρητής λαθών & προβλημάτων
    errors=0
    


    def __init__(self, port='', type='large', inverted=False,loggingLevel=3,kp=0.2,ki=0.05,kd=0,dt=0.01,stallDetectionTime=0.2):
        """[Αρχικοποιεί το αντικείμενο Μοτερ.]

        Keyword Arguments:
            port {str} -- [A ή B ή C ή D (αναλόγως την πόρτα που έχει συνδεθεί)] (default: {''})
            type {str} -- [Medium ή Large (αναλόγως τον τύπο του μοτέρ)] (default: {'large'})
            inverted {bool} -- [True ή False (αν επιθυμούμε αντεστραμένη ή ορθή περιστροφή)] (default: {False})
            loggingLevel {int} -- [description] (default: {3})
            kp {float} -- [Ο αναλογικός συντελεστής για την διόρθωση της ταχύτητας] (default: {0.05})
            dt {float} -- [το χρονικό διάστημα ανάμεσα στις εντολές προς το μοτέρ] (default: {0.01})
        """
        self.initialized = False
        self.port = port
        self.state=0
        self.timer=myTimer(dt=dt)
        self.stallDetectionTime=stallDetectionTime
        
        # προεπιλεγμένες ρυθμίσεις για τον PID (χρήση απο την SetSpeed)
        self.pid=myPID(name='motor'+port.upper()+' pid',kp=kp,ki=ki,kd=kd,loggingLevel=loggingLevel)       
        if port.upper() in ('A','B','C','D'):
            self.name = 'motor'+port.upper()
            self.logger = myLogger(name=self.name)
            self.logger.parent=self
            self.logger.level=loggingLevel
            if type.upper()=='MEDIUM':
                self.speedRatio=14
            else:
                self.speedRatio=10
            self.power = 0
            # self.inverted=inverted
            self.actionDone = True
            if port.upper()=='A':
                myPort=Port.A
            elif port.upper()=='B':
                myPort=Port.B
            elif port.upper()=='C':
                myPort=Port.C
            elif port.upper()=='D':
                myPort=Port.D
            else:
                self.logger.Error('The port number '+str(port)+' is not defined')
            try:
                if inverted==False:
                    self.motor = Motor(myPort)
                    self.logger.Debug(' ok')
                else:
                    self.motor = Motor(myPort,Direction.COUNTERCLOCKWISE)
                    self.logger.Debug(' ok')
                self.initialized = True
                self.logger.Out('initialized ok')
            except Exception as ex:
                self.logger.Out('Error initialize moter on port '+str(port)+'\n\t'+repr(ex))
                brick.light(Color.RED)
                self.errors+=1
  
    def GetSpeed(self):
        """
        Επιστρέφει την πραγματική ταχύτητα του Μοτερ.

        Returns:
            [float] -- [η ταχύτητα του μοτέρ στην κλίμακα -100..100]
            
        """
        return(self.motor.speed()/self.speedRatio)

    def GetSpeedAvg(self):
        """
        Επιστρέφει την μέση ταχύτητα του Μοτερ (απο το τελευταίο Reset).

        Returns:
            [float] -- [η ταχύτητα του μοτέρ σε degrees/second ]
            
        """
        return(self.GetDegrees()/(time.perf_counter()-self.resetTime)/self.speedRatio)

    def SetSpeed(self, speed=20.0,accelerationTime=0,minSpeed=0,maxSpeed=100):
        """[Θέτει την ταχύτητα του μοτέρ. Η ενέργεια δεν ολοκληρώνεται σε ένα βήμα.
        Αυτό αποτελέι ένα βήμα προς την τιμή στόχο που είναι το speed.
        Απαιτείται ο pid του μοτέρ να έχει kp>0. Ειδάλως δεν ξεκινάει το μοτέρ ]

        Keyword Arguments:
            speed {float} -- [η ταχύτητα στόχος του μοτέρ] (default: {20.0})
            accelerationTime {float} -- [Ο χρόνος επιτάχυνσης για να επιτευθεί η τιμή στόχος] (default: {0})
            minSpeed {float} -- [η ελάχιστη ταχύτητα που θα δοθεί στο μοτέρ] (default: {0})
            maxSpeed {float} -- [η μέγιστη ταχύτητα που θα δοθεί στο μοτέρ] (default: {100})
        """
        currentSpeed=self.GetSpeed()
        
        if self.state==0:
            self.timer.Start()
            self.topSpeed=0
            self.state=1 
            self.zeroSpeedCount=int(self.stallDetectionTime/self.timer.dt)
        else:
            if currentSpeed==0:
                self.zeroSpeedCount-=1
            if self.zeroSpeedCount<0:
                self.state=2
            if abs(currentSpeed)>abs(self.topSpeed):
                self.topSpeed=currentSpeed
            
        if accelerationTime!=0:
            speed=min(speed,speed*self.timer.GetTime()/accelerationTime)    
        error=(speed-currentSpeed)
        corr=self.pid.Step(inputRead=currentSpeed,target=speed)
        self.power=  speed+corr
        if abs(self.topPower)<abs(self.power):
            self.topPower = self.power
        if self.power>maxSpeed:
            self.power=maxSpeed
        if self.power<-maxSpeed:
            self.power=-maxSpeed
        if abs(self.power)<abs(minSpeed):
            self.power=minSpeed*self.power/abs(self.power)    
        # περίμενε να ολοκληρωθεί τουλάχιστον χρόνο dt
        self.timer.Step()
        self.motor.dc(self.power)
        #self.Print(str(self.power))
        #self.Print('power:'+str(speed)+'cur speed '+str(currentSpeed)+' speed+corr '+str(speed+corr))
  
    def SetAngle(self, angle=20.0,maxSpeed=60,minSpeed=10,maxAngle=100,plusMinus=2,stopAtEnd=True):
        currentAngle=self.GetDegrees()
        
        if self.state==0:
            self.state=1 
        if abs(angle)>abs(maxAngle):
            if angle>0:
                angle=abs(maxAngle)
            else:
                angle=-abs(maxAngle)
                
        error=(angle-currentAngle)
        if abs(error)<=plusMinus:
            if stopAtEnd:
                self.Off()
            
        elif abs(error)>maxAngle/10:
            self.power=error/abs(error)*maxSpeed
        else:            
            if minSpeed!=maxSpeed:
                self.power=error/abs(error) *minSpeed+   error*(maxSpeed-minSpeed)/maxAngle
            elif error!=0:
                self.power=error/abs(error)*minSpeed
            else:
                self.power=minSpeed
        self.SetSpeed(self.power)
        return(error)
  

    def SetAngle2(self, angle=20.0,speed=20,plusMinus=5,stopAtEnd=True):
        currentAngle=self.GetDegrees()
        if self.state==0:
            self.state=1 
                
        error=(angle-currentAngle)
        
        if abs(error)<=plusMinus:
            if stopAtEnd:
                self.Off()
        elif error>=0:
            self.SetSpeed(speed=speed)
        else:
            self.SetSpeed(speed=-speed)
        return(error)
  
 

    def IsStalled(self):
        """
        Επιστέφει αν έχει κολήσει το μοτέρ
        :return: Bool
        """
        return(self.state==2)

    def GetPosition(self):
        """[Επιστρέφει τις μοίρες του μοτέρ απο την αρχή της λειτουργίας του.]
        
        Returns:
            [int] -- [οι μοίρες του μοτέρ απο την αρχή της λειτουργίας του.]
        """
        d = 0
        try:
            d = self.motor.angle()
            self.lastPosition = d
            done = True
        except Exception as ex:
            self.logger.Error(self.name+' error in GetPosition() '+'\n\t'+repr(ex))
            d = self.lastPosition
            brick.light(Color.YELLOW)
        return (d)

    def ResetDegrees(self):
        """[Μαρκάρει την τρέχουσα θέση του μοτέρ. Αυτή η θέση αντιπροσωπεύει τις 0 μοίρες.]
        """
        self.degrees = self.GetPosition()
        self.resetTime = time.perf_counter()

    def GetDegrees(self):
        """[Επιστρέφει τις μοίρες που κινήθηκε το μοτέρ απο τον τελευταίο μηδενισμό.]
        
        Returns:
            [int] -- [Οι μοίρες απο τον τελευταίο μηδενισμό (ResetDegrees())]
        """
        return (self.GetPosition() - self.degrees)

    def _Go(self,speed=20,waits=[],stallChecking=False,stallDetectionTime=0.06,dt=0.01,stopAtEnd=True):
        """[summary]

        Args:
            speed (int, optional): [description]. Defaults to 20.
            waits (list, optional): [description]. Defaults to [].
            stallChecking (bool, optional): [description]. Defaults to False.
            stallDetectionTime (float, optional): [description]. Defaults to 0.06.
            dt (float, optional): [description]. Defaults to 0.01.
            stopAtEnd (bool, optional): [description]. Defaults to True.
        """
        self.actionDone = False
        speed2=speed*0.6
        s = "\n=================================================================="
        s+="\n| Motor"+self.port.upper()+".Go(speed="+str(speed)+",waits="+self.ToString(waits)
        s +=",stallChecking="+self.ToString(stallChecking)+",stallDetectionTime="+str(stallDetectionTime)+",dt="+str(dt)
        s +=",stopAtEnd=" + self.ToString(stopAtEnd)+ ")"
        if len(waits)>0:
            wait=waits.pop(0)
        else:
            wait=None
        startingDegrees=self.GetDegrees()
        self.timer.dt=dt
        self.stallDetectionTime=stallDetectionTime
        isStalled=False
        i=1
        start=time.perf_counter()
        self.SetSpeed(speed=speed)
        while wait!=None :            
            if self.IsStalled() and stallChecking:
                isStalled=True
                break
            if type(wait) in (int,float):
                speed2=speed*(wait-abs(self.GetDegrees()-startingDegrees))/wait*0.6
                if abs(self.GetDegrees()-startingDegrees)>=wait:                    
                    s+="\n| wait degrees="+str(wait)
                    wait=None
            elif type(wait)==str:
                # δημιούργησε απο το string το ζεύγος key,value
                kv=wait.split('=')
                key=kv[0]
                value=kv[1]
                if key in ('time','seconds'):
                    if time.perf_counter()-start>=float(value):
                        s+="\n| wait: "+wait
                        wait=None
                elif key == 'speed':
                    speed=float(value)
                    s+="\n| wait: "+wait
                    wait=None
            if wait==None:
                if len(waits)>0:
                    wait=waits.pop(0)
            i+=1
            self.SetSpeed(speed=speed*0.4+speed2)
        totalTime=time.perf_counter()-start

        if stopAtEnd:
            self.Off()
        s+='\n| totalTime='+str(totalTime)
        s+='\n| dt='+str(totalTime/i)
        s+='\n| Starting degrees='+str(startingDegrees)
        s+='\n| degrees='+str(self.GetDegrees())
        s+='\n| Total degrees='+str(self.GetDegrees()-startingDegrees)
        s+='\n| isStalled='+self.ToString(isStalled)
        s+='\n==================================================='
        self.logger.Debug(s)
        self.actionDone = True

    def Go(self,speed=20,waits=[],runInBackGroud=True,stallChecking=False,stallDetectionTime=0.06,dt=0.01,stopAtEnd=True):
        """[summary]

        Args:
            speed (int, optional): [Ταχύτητα του μοτέρ]. Defaults to 20.
            waits (list, optional): [λίστα αναμονών]. Defaults to [].
            runInBackGroud (bool, optional): [αν θα τρέχει στο background]. Defaults to True.
            stallChecking (bool, optional): [Έλεγχος κολήματος του μοτέρ]. Defaults to False.
            stallDetectionTime (float, optional): [χρόνος ακινησίας του μοτέρ για να χαρακτηριστεί κολημένο]. Defaults to 0.06.
            dt (float, optional): [χρόνος ανάμεσα στους ελέγχους του μοτέρ]. Defaults to 0.01.
            stopAtEnd (bool, optional): [Να σβήσει το μοτέρ στο τέλος]. Defaults to True.

        Returns:
            None
        
        Examples:
            r.motorB.Go(speed=20,waits=[400,'speed=40',800])
            r.motorB.Go(speed=20,waits=['seconds=3'])
        """
        if runInBackGroud:
            self.thread = threading.Thread(target=self._Go,
                                       kwargs={'speed': speed, 'waits': waits, 'stallChecking':stallChecking, 
                                       'stallDetectionTime': stallDetectionTime,'dt':dt,'stopAtEnd':stopAtEnd})
            self.thread.start()
            return (self.thread)
        else:
            self._Go(speed=speed,waits=waits,stallChecking=stallChecking,stallDetectionTime=stallDetectionTime,dt=dt,stopAtEnd=stopAtEnd)

    def On(self, power=20.0, unregulated=True):
        """[Ενεργοποιεί το μοτέρ
        Keyword Arguments:
            power {float} -- [Η ισχύς που δίνουμε στο μοτέρ (-100 .. 100)] (default: {20.0})
            unregulated {bool} -- [Χωρίς PID έλεγχο των μοτέρ] (default: {True})
        
        Returns:
            [int] -- [Οι μοίρες απο τον τελευταίο μηδενισμό (resetDegrees())]
        """
        # αλλάζει την κατάσταση του μοτέρ
        self.state=1
        self.power = min(power,100)
        self.power = max(self.power,-100)
        if unregulated:
                self.motor.dc(self.power)            
        else:
            self.motor.run(self.power * self.speedRatio)
        return(0)    

    def On_2(self, power=20.0,degrees=0, seconds=0.0, stallChecking=False, wait=True, unregulated=True):
        """[Ενεργοποιεί το μοτέρ
        Αν η παράμετρος degrees δεν είναι 0, το μοτέρ κινείτε μέχρι να περιστραφεί τόσες μοίρες
        Αν η παράμετρος seconds δεν είναι 0, το μοτέρ κινείτε για αντίστοιχο χρόνο
        Αν η παράμετρος stallChecking είναι Αληθής, το μοτέρ κινείτε μέχρι βρεί εμπόδιο
        Αν η παράμετρος wait είναι True τότε η συνάρτηση περιμένει μέχρι να ολοκληρωθεί η κίνηση.
        Σε κάθε άλλη περίπτωση δίνεται εντολή να ανοίξει το μοτέρ. (Απαιτείτε εντολή Off() για να σταματήσει)]
        
        Keyword Arguments:
            power {float} -- [Η ισχύς που δίνουμε στο μοτέρ (-100 .. 100)] (default: {20.0})
            degrees {int} -- [Οι μοίρες που θέλουμε να κινηθεί το μοτέρ (0 σημαίνει απροσδιόριστο)] (default: {0})
            seconds {float} -- [Ο χρόνος σε δεπτερόλεπτα που θέλουμε να κινηθεί το μοτέρ (0 σημαίνει απροσδιόριστο)] (default: {0.0})
            stallChecking {bool} -- [True αν θέλουμε το μοτέρ να κινηθεί μέχρι να βρεί εμπόδιο και να σταματήσει. ] (default: {False})
            wait {bool} -- [True αν θέλουμε να περιμένουμε μέχρι ώτου ολοκληρωθεί η εντολή πρίν το πρόγραμμα συνεχίσει στην επόμενη εντολή] (default: {True})
            unregulated {bool} -- [Χωρίς PID έλεγχο των μοτέρ] (default: {False})
        
        Returns:
            [int] -- [Οι μοίρες απο τον τελευταίο μηδενισμό (resetDegrees())]
        """
        # αλλάζει την κατάσταση του μοτέρ
        self.state=1
        self.power = power 
        if self.power>100:
            self.power=100
        if self.power<-100:
            self.power=-100
        if unregulated:
                self.motor.dc(self.power)            
        elif stallChecking:
            if degrees==0:
                self.motor.run_until_stalled(power * self.speedRatio, Stop.HOLD, 50)
            else:                 
                pass
        elif degrees!=0:
            self.motor.run_angle(self.power * self.speedRatio, degrees, Stop.HOLD, wait)
        elif seconds!=0:
            self.motor.run_time(self.power * self.speedRatio, seconds*1000, Stop.HOLD, wait)
        else:
            self.motor.run(self.power * self.speedRatio)
        #self.logger.Debug(self.name+'.On(power='+str(power)+',degrees='+str(degrees)+',seconds='+str(seconds)
        return(0)    
            # if power < 0:
            #     degrees = -degrees
            # # do not reset degrees inside the method, but before the method.
            # # self.ResetDegrees()
            # if stop_action.upper()=='COAST':
            #     stop_type=Stop.COAST
            # elif stop_action.upper()=='HOLD':
            #     stop_type=Stop.HOLD
            # else:
            #     stop_type=Stop.BRAKE
            # distance0 = self.GetPosition()
            # if seconds!=0:
            #     self.motor.run_timed(speed=power*self.speedRatio,time=seconds*1000,stop_type=stop_type,wait=wait)
            # elif degrees!=0:
            #     self.motor.run_target(speed=power*self.speedRatio, target_angle=degrees, stop_type=stop_type, wait=wait)
            #     # or self.motor.run_angle(speed=power*self.speedRatio, target_angle=degrees, stop_type=stop_type, wait=wait)
            #     # self.motor.target_angle (angle=degrees)
            # elif stallChecking:
            #     self.motor.run_until_stalled(speed=power*self.speedRatio, stop_type=stop_type)        
            # else:   
            #     self.motor.run(self.power * self.speedRatio)
                   
            # distance=self.GetPosition()-distance0
            # self.logger.Debug(self.name+'.On(power='+str(power)+',degrees='+str(degrees)+',seconds='+str(seconds)
            # +',stop_action="'+stop_action+'", stallChecking='+str(stallChecking)+', wait='+str(wait)+')\r\n\tdistance:'+str(distance))
            # return (distance)
            
    def Off(self,stop_action='brake'):
        """[Σταματάει το μοτέρ]
        
        Keyword Arguments:
            stop_action {str} -- [brake, coast or hold] (default: {'brake'})
        
        Returns:
            [int] -- [οι μοίρες που κινήθηκε το μοτέρ απο το τελευταίο ResetDegrees()]
        """
       
        self.actionDone = True
        self.motor.dc(0)
        if stop_action.upper()=='COAST':
            stop_type=Stop.COAST
        elif stop_action.upper()=='HOLD':
            stop_type=Stop.HOLD
        else:
            stop_type=Stop.BRAKE
        self.motor.stop(stop_type)
        # just to ensure that previous action completed
        #self.motor.run(0)
        if self.logger.level<=2:
            s='Off(stop_action="'+stop_action+'")'
            s+=' degrees:'+str(self.GetDegrees())
            self.logger.Debug(s)
        self.power=0
        self.state=0
        #self.pid.Reset()
        return(self.GetDegrees())

    def WaitForDistance(self, degrees= 100,stallChecking=False,stallDetectionTime=0.02):
        """[summary]

        Args:
            degrees (int, optional): [description]. Defaults to 100.
            stallChecking (bool, optional): [description]. Defaults to False.
            stallDetectionTime (float, optional): [description]. Defaults to 0.02.
        """
        startingPosition=self.GetPosition()
        d=0
        i=0
        l=list(range(0,int(stallDetectionTime/0.005)))
        if stallChecking:
            time.sleep(stallDetectionTime*5)
        while abs(d)<abs(degrees):
            time.sleep(0.005)
            d=abs(self.GetPosition()-startingPosition)
            l[i % len(l)]=d            
            i+=1
            if max(l)==min(l) and stallChecking:
                break
        d=abs(self.GetPosition()-startingPosition)
        return(d)

    def WaitToCompleteAction(self):
        """Αναμονή του μοτέρ να ολοκληρωθούν οι ενέργειες του που εκτελούνται στο background
        """
        time.sleep(0.01)
        while not self.actionDone:
            time.sleep(0.005)

    def Test(self,power=10,seconds=4,unregulated=True):
        """[Εκτελεί διαγνωστικό έλεγχο στο μοτέρ]
        
        Keyword Arguments:
            power {int} -- [Η ισχύς των μοτέρ κατά την διαδικασία] (default: {10})
            seconds {int} -- [Ο χρόνος που θα διαρκέσει η διαδικασία] (default: {4})
            unregulated {bool} -- [Χωρίς PID έλεγχο των μοτέρ] (default: {True})
        """
        s="\nPhase1 On (unregulated test)"
        self.ResetDegrees()
        t0=time.perf_counter()
        self.On(power=power,unregulated=True)
        time.sleep(seconds)
        self.Off()
        t1=time.perf_counter()
        dist=self.GetDegrees()
        t=t1-t0
        s+='\npower:'+str(power)+' speed(degrees/sec):'+str(dist/t)+ ' degrees:'+ str(dist)
        s+="\nPhase2 SetSpeed test"
        self.ResetDegrees()
        t0=time.perf_counter()
        t2=t0
        while time.perf_counter()-t0<=seconds:
            self.SetSpeed(speed=power)
            if t0==t2 and self.GetSpeed()>=power:
                t2=time.perf_counter()

        self.Off()
        t1=time.perf_counter()
        dist=self.GetDegrees()
        t=t1-t0
        s+='\nspeed:'+str(power)+' speed(degrees/sec):'+str(dist/t)+ ' degrees:'+ str(dist)+' acceleration time:'+str(round(t2-t0,3))
        self.logger.Out(s)

    # def Print(self,str="Motor Print"):
    #     self.logger.Info(str)

    def GetStatus(self):
        """
        Επιστρέφει κείμενο με λεπτομέρειες της κατάστασης του moter

        :return
            (str):
        """
        s= '\n-----------------------------------------------------------'
        s+='\n| Status '+self.name
        s+='\n| degrees     :\t'+str(self.GetDegrees())
        s+='\n| topSpeed (%):\t'+str(round(self.topSpeed,2))
        s+='\n| avgSpeed (%):\t'+str(round(self.GetSpeedAvg(),2))
        s+='\n| topPower (%):\t'+str(round(self.topPower,2))
        s+='\n----------------------------------------------------------'
        # print(s)
        return(s)

    def PrintStatus(self):
        self.Print(self.GetStatus())

class myGyro(legoPart):
    """[summary]

    Args:
        legoPart ([type]): [description]
    """
    initialized=False
    driftError=0
    angleCorrection=0
    port=0
    name='gyro'
    started=0
    i=0
    value=0
    values=[]
    # Η πατρική κλάση
    parent=None
    # μετρητής λαθών και προβλημάτων
    errors=0
    
    def __init__(self,port=0,loggingLevel=3):
        """[Αρχικοποιεί το γυροσκόπιο]

        Keyword Arguments:
            port {int} -- [Η θύρα που είναι συνδεδεμένο το γυροσκόπιο] (default: {0})
            loggingLevel {int} -- [Το επίπεδο καταγραφής] (default: {3})
        """
        self.timer=myTimer(dt=0.01)
        self.name = 'gyro'+str(port)
        self.driftError=0
        if port in (1,2,3,4):
            if port==1:
                myPort=Port.S1
            elif port==2:
                myPort=Port.S2
            elif port==3:
                myPort=Port.S3
            elif port==4:
                myPort=Port.S4
            else:
                self.logger.Error('The port number '+str(port)+' is not defined')
            self.port = port
            self.logger = myLogger(name=self.name,filename='log.txt',level=loggingLevel)
            self.logger.parent=self
            try:
                self.sensor = GyroSensor(myPort,Direction.CLOCKWISE)
                # self.logger.Debug(' ok')
                self.initialized=True
                self.Reset()
                t0=time.perf_counter()
                self.started=t0
                time.sleep(2)
                if self.GetAngle()!=0:
                    time.sleep(10)
                    angle=self.GetAngle()
                    t1=time.perf_counter()
                    self.driftError=angle/(t1-t0)
                    self.logger.Out('Drift Detected '+str(self.driftError)+' degress/sec')
                    brick.light(Color.YELLOW)
                    self.errors+=1
                self.logger.Out('initialized ok')
            except Exception as ex:
                self.logger.Out('Error initialize Gyro on port '+str(port)+'\n\t'+repr(ex))
                brick.light(Color.RED)
                self.errors+=1
                
                

                

    def Reset(self,angle=0):
        """
        Reset Gyro Angle
        :return:  None
        """
        self.sensor.reset_angle(angle)

    def GetAngle(self):
        """[Επιστρέφει την γωνία που μας δίνει το γυροσκόπιο]
        """
        self.value=self.sensor.angle()+self.angleCorrection
        if self.driftError!=0:
            t1=time.perf_counter()
            self.value=self.value-self.driftError*(t1-self.started)
        self.i +=1
        if len(self.values)<30:
            self.values.append(self.value)
        else:
            self.values[self.i % 30]=self.value
            
        return(self.value)

    def WaitForAngle(self, angle=90, distance=2):
        """[Περίμενε μέχρι να έχουμε την γωνία στόχο.
        Επιστρέφει την πραγματική γωνία]

        Keyword Arguments:
            angle {int} -- [Η γωνία που θέλουμε να έχουμε] (default: {90})
            distance {int} -- [Το αποδεχτό περιθώριο ανοχής απο την τιμή στόχο] (default: {2})
        """
        angle0=self.GetAngle()
        angle1=angle0        
        while abs(angle1-angle0-angle)>distance:
            #time.sleep(0.01)
            self.timer.Step()
            angle1=self.GetAngle()
            #print(angle1-angle0)
        return(angle1)

    def GetStatus(self):
        """
        Επιστρέφει κείμενο με λεπτομέρειες της κατάστασης του αισθητήρα gyro

        :return
            (str):
        """
        s= '-----------------------------------------------------------'
        s+='\n| Status '+self.name
        s+='\n| values :\t'+self.ListToStr(self.values)
        s+='\n----------------------------------------------------------'
        # print(s)
        return(s)

    # def Print(self,str="Gyro Print"):
    #     self.logger.Info(str)
    
    def Test(self):
        count=200
        s=''
        # getLight()
        t0=time.perf_counter()
        for i in range(count):
            self.GetAngle()
        t1=time.perf_counter()
        s='GetAngle dt: '+str((t1-t0)/count)
        self.logger.Warning(s)
        
class myUltraSonic(legoPart):
    """[Η κλάση που χειρίζεται το αισθητήρα απόστασης]
    """
    initialized=False
    port=0
    name='UltrSonic'
    started=0
    i=0
    value=0
    values=[]
    mask=[0,0,0]
    # Η πατρική κλάση
    parent=None
    # μετρητής λαθών και προβλημάτων
    errors=0
    
    def __init__(self,port=0,loggingLevel=3):
        """[Αρχικοποιεί το αισθητήρα απόστασης]

        Keyword Arguments:
            port {int} -- [Η θύρα που είναι συνδεδεμένο το γυροσκόπιο] (default: {0})
            loggingLevel {int} -- [Το επίπεδο καταγραφής 1 έως 5] (default: {3})
        """
        self.timer=myTimer(dt=0.01)
        self.name = 'ultrasonic'+str(port)
                
        if port in (1,2,3,4):
            if port==1:
                myPort=Port.S1
            elif port==2:
                myPort=Port.S2
            elif port==3:
                myPort=Port.S3
            elif port==4:
                myPort=Port.S4
            else:
                self.logger.Error('The port number '+str(port)+' is not defined')
            self.port = port
            self.logger = myLogger(name=self.name,filename='log.txt',level=loggingLevel)
            self.logger.parent=self
            try:
                self.sensor = UltrasonicSensor(myPort)
                # self.logger.Debug(self.name+' initilized')
                self.initialized=True
                self.logger.Out('initialized ok')
                t0=time.perf_counter()
                self.started=t0

            except Exception as ex:
                self.logger.Error('Error initialize UltraSonic on port '+str(port)+'\n\t'+repr(ex))
                brick.light(Color.RED)
                self.errors+=1


    def Reset(self):
        """[Αρχικοποιεί τον αισθητήρα απόστασης]
        """
        self.i=0;

    def GetDistance(self):
        """[Επιστρέφει την απόσταση που διαβάζει ο αισθητήρας]

        Args:
            readCount (int, optional): [Το πλήθος των αναγνώσεων απο τον αισθητήρα]. Defaults to 3.

        Returns:
            [float]: [Η απόσταση που διάβασε ο αισθητήρας]
        """
        self.value=self.sensor.distance()/10
        return (self.value)
    
    def GetDistance2(self,readCount=3):
        """[Επιστρέφει την απόσταση που διαβάζει ο αισθητήρας]

        Args:
            readCount (int, optional): [Το πλήθος των αναγνώσεων απο τον αισθητήρα]. Defaults to 3.

        Returns:
            [float]: [Η απόσταση που διάβασε ο αισθητήρας]
        """
        if self.i==0:
            for k in range(3):
                self.value=self.sensor.distance()/10
                self.mask[self.i%3]=self.value
                self.i+=1
        else:
            for k in range(readCount):
                self.value=self.sensor.distance()/10
                self.mask[self.i%3]=self.value
                self.i+=1
        a=self.mask
        a.sort()
        self.value=a[1]
        if self.logger.level<3:
            # store last 100 values
            if len(self.values)<100:
                self.values.append(self.value)
            else:
                self.values[self.i % 100]=self.value
                self.values[(self.i+1) % 100]='-->|' 
        return (self.value)
    
    def WaitForDistance(self, distance=20):
        """[Περίμενε μέχρι να έχουμε την Απόσταση στόχο.
        Επιστρέφει την πραγματική απόσταση]

        Keyword Arguments:
            distance {float} -- [Η απόσταση σε cm για την οποία περιμένουμε] (default: {20})
        """
        distance0=self.GetDistance(readCount=3)
        distance1=distance0  
        k=1
        t0=time.perf_counter()   
        if distance0==distance:
            return(distance)
        elif distance<distance0:
            while distance<distance1:
                self.timer.Step()
                distance1=self.GetDistance(readCount=1)
                # self.logger.Info(str(distance1))
                k+=1
        else:
            while distance>distance1:

                self.timer.Step()
                distance1=self.GetDistance(readCount=1)
                # self.logger.Info(str(distance1))
                k+=1
        dt=(time.perf_counter()-t0)/k
        self.logger.Debug('WaitForDistance')
        self.logger.Debug('Final Distance:'+str(distance1))
        self.logger.Debug('real dt:'+str(dt))
        return(distance1)

    def GetStatus(self):
        """
        Επιστρέφει κείμενο με λεπτομέρειες της κατάστασης του αισθητήρα

        :return
            (str):
        """
        s= '-----------------------------------------------------------'
        s+='\n| Status '+self.name
        s+='\n| values :\t'+self.ListToStr(self.values)
        s+='\n----------------------------------------------------------'
        # print(s)
        return(s)

    def Test(self):
        """Εκτελεί διαγνωστικό έλεγχο στον αιθητήρα. Το σημαντικότερο αποτέλεσμα είναι 
        ο χρόνος ανάγνωσης (dt).
        """
        count=200
        s=''
        # getLight()
        t0=time.perf_counter()
        for i in range(count):
            self.GetDistance(readCount=1)
        t1=time.perf_counter()
        s='GetDistance dt: '+str((t1-t0)/count)
        self.logger.Warning(s)


#######################################################################################################
class myRobot(legoPart):
    """Η βασική κλάση διαχείρισης του Ρομπότ
    """
    display=brick.display
    motorA = myMotor()
    motorB = myMotor()
    motorC = myMotor()
    motorD = myMotor()
    color1 = myColorSensor()
    color2 = myColorSensor()
    color3 = myColorSensor()
    color4 = myColorSensor()
    buttons=brick.buttons
    color = None
    gyro = myGyro()
    pi=3.14159265359
    ultraSonic = myUltraSonic()
    axleTrack=176
    axleTrack50=176
    axleTrack100=176
    wheelDiameter=62.4
    wheelsToSensorsdistance=50
    sensorToSensorDistance=20
    extraDegrees=0
    lastAngle=0
    nextSpeed=0
    lastMessage=''
    # waits list
    waits=[]
    # το τελευταίο action απο τα waits
    action=None
    # για την απόσταση που υπήρξε το τελευταίο wait 
    markDegrees=0
    # για να ελέγχω την απόσταση μέσα απο τα waits
    soDegrees=0
    # για να ελέγχω την μέγιστη ταχύτητα μέσα απο τα waits
    maxSpeed=0
    # για στατιστικά
    speedsB=[]
    speedsC=[]
    # για FollowLine
    zeroErrorSteering=0
    # time mark
    started=0
    duration=0

    def __init__(self, loggingLevel=3, wheelDiameter=62.4, axleTrack=170, wheelsToSensorsdistance=50, sensorToSensorDistance=42):
        """[Αρχικοποιεί το αντικείμενο Ρομποτ.]

        Keyword Arguments:
            loggingLevel {int} -- [Το επίπεδο καταγραφής] (default: {3})
            wheelDiameter {float} -- [Η διάμετρος του τροχού σε mm] (default: {62.4})
            axleTrack {int} -- [Η απόσταση σε mm μεταξύ των τροχών (μετατρόχιο)] (default: {170})
            wheelsToSensorsdistance {int} -- [Η απόσταση απο τους τροχούς μέχρι τους αισθητήρες αντανάκλασης] (default: {50})
            sensorToSensorDistance {int} -- [Η απόσταση μεταξύ των αισθητήρων της κίνησης] (default: {42})
        """
        self.name = 'myRobot'
        self.lastMessage=''
        self.timer=myTimer(dt=0.01)
        self.logger = myLogger(name=self.name)
        self.logger.level=loggingLevel
        if type(axleTrack) in (int,float):
            self.axleTrack=axleTrack
            self.axleTrack50=axleTrack
            self.axleTrack100=axleTrack
        else:
            self.axleTrack=axleTrack[1]
            self.axleTrack50=axleTrack[0]
            self.axleTrack100=axleTrack[1]

        self.wheelDiameter=wheelDiameter
        self.wheelsToSensorsdistance=wheelsToSensorsdistance
        self.sensorToSensorDistance=sensorToSensorDistance
        self.loggingLevel=loggingLevel
        

    def Start(self,waitForButton=False,beep=True):  
        """Έναρξη της λειτουργίας του Ρομποτ. Ενεγοποίηση του εσωτερικού χρονομέτρου.
        1) Ενημερώνεται στα μέρη του ρομπότ η πατρική κλάση.
        2) Υπάρχει η δυνατότητα να περιμένει για πάτημα κουμπιού για έναρξη της λειτουργίας.

        Args:
            waitForButton (bool): [Να περιμένει για πάτημα πλήκτρου του Ρομπότ.]. Defaults to 'False'.
        """     
        
        if self.motorA.initialized or self.motorA.errors>0:
            self.motorA.SetParent(self)
        if self.motorB.initialized or self.motorB.errors>0:
            self.motorB.SetParent(self)
        if self.motorC.initialized or self.motorC.errors>0:
            self.motorC.SetParent(self)
        if self.motorD.initialized  or self.motorD.errors>0:
            self.motorD.SetParent(self)
        if self.color1.initialized or self.color1.errors>0:
            self.color1.SetParent(self)
        if self.color2.initialized or self.color2.errors>0:
            self.color2.SetParent(self)
        if self.color3.initialized or self.color3.errors>0:
            self.color3.SetParent(self)
        if self.color4.initialized or self.color4.errors>0:
            self.color4.SetParent(self)
        if self.gyro.initialized or self.gyro.errors>0:
            self.gyro.SetParent(self)
        if self.ultraSonic.initialized or self.ultraSonic.errors>0:
            self.ultraSonic.SetParent(self)
        self.Check()
        if beep:
            self.Beep()
        if waitForButton:
            # self.Display(text='Duration: '+str(self.duration))
            self.Display(text='Press any key to Start')
            self.WaitForButton()
        brick.display.image(ImageFile.AWAKE)
        self.started=time.perf_counter()

    def Finish(self,waitForButton=False,printStatus=False):
        """Όλοκλήρωση της αποστολής του Ρομπότ. Καταγραφή του χρόνου λειτουργίας του Ρομπότ. 
        1) Εμφάνιση της κατάσταση κάθε αισθητήρα και μοτέρ. Αποθήκευση του αρχείου με τις καταγραφές.
        22) Υπάρχει η δυνατότητα να περιμένουμε για πάτημα κουμπιού πρίν την ολοκλήρωση του προγράμματος.

        Args:
            waitForButton (bool, optional): [Αν επιθυμούμε την αναμονή για πλήκτρο στο τέλος της Αποστολής]. Defaults to False.
        """
        parts=[self.color1,self.color2,self.color3,self.color4,self.ultraSonic,self.gyro,self.motorA,self.motorB,self.motorC,self.motorD]
        self.duration=-self.started+time.perf_counter()
        self.logger.Out('Mission Duration: '+str(self.duration))
        if printStatus:
            for part in parts:
                if part.initialized:
                    self.logger.Out(part.GetStatus())

        # if self.color1.initialized:
        #     self.logger.Warning(self.color1.GetStatus())
        # if self.color2.initialized:
        #     self.logger.Warning(self.color2.GetStatus())
        # if self.color3.initialized:
        #     self.logger.Warning(self.color3.GetStatus())
        # if self.color4.initialized:
        #     self.logger.Warning(self.color4.GetStatus())
        # if self.gyro.initialized:
        #     self.logger.Warning(self.gyro.GetStatus())
        # if self.ultraSonic.initialized:
        #     self.logger.Warning(self.ultraSonic.GetStatus())
        self.logger.Save()
        
        if waitForButton:
            self.Display(text='Duration: '+str(self.duration))
            self.Display(text='Press a Robot button to continue')
            self.logger.Warning('Press a Robot button to continue')
            self.WaitForButton()
        

    def Check(self):  
        """
        Έλεγχει τα μέρη του Ρομπότ για σφάλματα αρχικοποίησης
        """     
        self.logger.Save()
        imgs=[]
        
        if self.motorA.errors>0:
            imgs.append("./images/a.png")
        if self.motorB.errors>0:
            imgs.append("./images/b.png")
        if self.motorC.errors>0:
            imgs.append("./images/c.png")
        if self.motorD.errors>0:
            imgs.append("./images/d.png")
        if self.color1.errors>0:
            imgs.append("./images/1.png")
        if self.color2.errors>0:
            imgs.append("./images/2.png")
        if self.color3.errors>0:
            imgs.append("./images/3.png")
        if self.color4.errors>0:
            imgs.append("./images/4.png")
        if self.gyro.errors>0:
            imgs.append("./images/alert.png")
        if self.ultraSonic.errors>0:
            imgs.append("./images/alert.png")
        if len(imgs)>0:
            try:
                brick.display.image(imgs[0])                
            except Exception as e:
                brick.display.image(ImageFile.WARNING)
                self.logger.Error('error in display image '+imgs[0]+' (in robot.check)')
                
        else:
            img1="./images/tap.png"
            try:
                brick.display.image(img1)
            except Exception as e:
                brick.display.image(ImageFile.UP)
                self.logger.Error('error in display image '+img1+' (in robot.check)')

    def DegreesToDistance(self,degrees=0):
        """Μετατρέπει τις μοίρες του τροχού σε πραγματική απόσταση σε mm. (Βάσει των στοιχείων γεωμετρίας του Ρομπότ)

        Args:
            degrees (int, optional): [description]. Defaults to 0.

        Returns:
            [float]: [Η υπολογιζόμενη απόσταση σε mm]
        """
        return int(degrees*3.14159*self.wheelDiameter/360)
        
    def DistanceToDegrees(self,distance=0):
        """Μετατρέπει απόσταση (mm)σε τις μοίρες του τροχού. (Βάσει των στοιχείων γεωμετρίας του Ρομπότ)

        Args:
            distance (int, optional): [description]. Defaults to 0.
        """
        return(distance*360/3.14159265359/self.wheelDiameter)
           
    def Display(self,text='',coordinate=None):
        """Εμφανίζει κείμενο στην οθόνη του Ρομπότ

        Args:
            text (str, optional): [description]. Defaults to ''.
            coordinate ([type], optional): [description]. Defaults to None.
        """
        self.display.text(text,coordinate)
    
    def Beep(self):
        """Στέλνει ένα ήχο Beep στο ηχείο του Brick
        """
        thread1 = threading.Thread(target=brick.sound.beep,kwargs={})
        thread1.start()

    def ButtonPressed(self):
        """Επιστρέφει αν πατήθηκε κάποιο πλήκτρο στο Robot

        Returns:
            [bool]: [αν πατήθηκε πλήκτρο]
        """
        return(brick.buttons())

    def WaitForButton(self,delayTimeBefore=0.3,dt=0.01):
        """Περιμένει μέχρι να πατηθεί κάποιο πλήκτρο στο Brick.

        Parameters:
            delayTimeBefore (float): Η καθυστέρηση πρίν ξεκινήσει η αναμονή για πλήκτρο
            dt (float): Η καθυστέρηση ανάμεσα στους ελέγχους για πάτημα πλήκτρου  
    
        return:
            None
        """
        if delayTimeBefore>0:
            time.sleep(delayTimeBefore)
        while not self.ButtonPressed():
            time.sleep(dt)
        self.Beep()

    def Calibrate(self, sensors=[2,3]):
        """
        Εκτελεί διαγνωστική λειτουργία καταγραφής των τιμών min max για
        τους αισθητήρες χρώματος που περιλαμβάνονται στην λίστα sensors. 
        Στην συνέχεια εκτυπώνει και αποθηκεύει τα αποτελέσματα

        Parameters:
            sensors (list): Η λίστα με τους αισθητήρες χρώμματος που θέλουμε
                            να συμπεριληφθούν. Για παράδειγμα [1,2,3,4]  
     
        return:
            None
        """
        self.logger.SetLevel(3)
        self.Beep()
        if 1 in sensors and self.color1.initialized:
            self.color1.logger.level=4
            self.color1.smartAdjust=False
            self.color1.mins={'white':100,'red':255,'green':255,'blue':255}
            self.color1.maxs={'white':0,'red':0,'green':0,'blue':0}
        if 2 in sensors and self.color2.initialized:
            self.color2.logger.level=4
            self.color2.smartAdjust=False
            self.color2.mins={'white':100,'red':255,'green':255,'blue':255}
            self.color2.maxs={'white':0,'red':0,'green':0,'blue':0}

        if 3 in sensors and self.color3.initialized:
            self.color3.logger.level=4
            self.color3.smartAdjust=False
            self.color3.mins={'white':100,'red':255,'green':255,'blue':255}
            self.color3.maxs={'white':0,'red':0,'green':0,'blue':0}

        if 4 in sensors and self.color4.initialized:
            self.color4.logger.level=4
            self.color4.smartAdjust=False
            self.color1.mins={'white':100,'red':255,'green':255,'blue':255}
            self.color1.maxs={'white':0,'red':0,'green':0,'blue':0}
           

        t0=time.perf_counter()
        while not any(brick.buttons()) or not time.perf_counter()-t0>2:
            if 1 in sensors and self.color1 != None:
                self.color1.GetLight()
                if self.color1.sensorType.lower()!='nxt':
                    self.color1.GetColor()
            if 2 in sensors and self.color2 != None:
                self.color2.GetLight()
                if self.color2.sensorType.lower()!='nxt':
                    self.color2.GetColor()
            if 3 in sensors and self.color3 != None:
                self.color3.GetLight()
                if self.color3.sensorType.lower()!='nxt':
                    self.color3.GetColor()
            if 4 in sensors and self.color4 != None:
                self.color4.GetLight()
                if self.color4.sensorType.lower()!='nxt':
                    self.color4.GetColor()
        s=''
        s2=''
        if 1 in sensors and self.color1.initialized:
            s+=self.color1.GetStatus()+'\n'
            s2+=self.color1.GenerateInitString()
        if 2 in sensors and self.color2.initialized:
            s+=self.color2.GetStatus()+'\n'
            s2+='\n'+self.color2.GenerateInitString()
        if 3 in sensors and self.color3.initialized:
            s+=self.color3.GetStatus()+'\n'
            s2+='\n'+self.color3.GenerateInitString()
        if 4 in sensors and self.color4.initialized:
            s+=self.color4.GetStatus()+'\n'
            s2+='\n'+self.color4.GenerateInitString()
        #print(s)
        
        self.logger.Out(s)
        self.logger.Out(s2)
        self.logger.Save()

    def CalibrateWheelsToSensorsDistance(self):
        self.motorB.ResetDegrees()
        self.color2.GetLight()
        while self.color2.GetLight()>50:
            self.motorB.SetSpeed(speed=-15)
            self.motorC.SetSpeed(speed=-15)
        self.motorB.Off()
        self.motorC.Off()
        s="wheelsToSensorsDistance="+str(self.DegreesToDistance(-self.motorB.GetDegrees()))+' mm'
        self.wheelsToSensorsdistance=self.DegreesToDistance(-self.motorB.GetDegrees())
        s="wheelsToSensorsDistance="+str(self.wheelsToSensorsdistance)+' mm'
        self.logger.Warning(s)
        

    def CalibrateAxleTrack(self,steering=100):
        """Υπολογίζει το axleTrack του ρομπότ. Απαιτείται να είναι συνδεδεμένο το
        γυροσκόπιο.

        Args:
            steering (int, optional): [description]. Defaults to 100.
        """
        minErr=9999999
        steer=steering
        axle=0
        for i in range(10):
            self.gyro.Reset()
            self.Turn(steering=steer,speed=40,angle=90)
            err1=90-self.gyro.GetAngle()
            self.gyro.Reset()
            self.Turn(steering=steer,speed=-40,angle=90)
            err2=(90+self.gyro.GetAngle())
            if abs(err1+err2)<abs(minErr):
                minErr=err1+err2
                if (abs(steer)+25) // 50<=1:
                    axle=self.axleTrack50
                else:
                    axle=self.axleTrack100
            elif abs(err1+err2)==abs(minErr):
                if (abs(steer)+25) // 50<=1:
                    axle=(axle+self.axleTrack50)/2
                else:
                    axle=(axle+self.axleTrack100)/2

            self.logger.Warning('********** steering:'+str(steer)+' gyro error:'+str(err1+err2)+ ' axletrack:['+str(self.axleTrack50)+','+str(self.axleTrack100)+'] '+ ' axletrack:'+str(axle))
            if (abs(steer)+25) // 50<=1:
                self.axleTrack50=self.axleTrack50+(err1+err2)*0.3
            else:
                self.axleTrack100=self.axleTrack100+(err1+err2)*0.3
            
        if (abs(steer)+25) // 50<=1:
            self.axleTrack50=axle
        else:
            self.axleTrack100=axle
        self.logger.Warning('\nRecomended axleTrack=['+ str(self.axleTrack50)+','+str(self.axleTrack100)+'] calc error:'+str(err2+err2))
        self.logger.Save()

    def CalibrateWheels(self):
        self.CalibrateWheelsToSensorsDistance()
        self.CalibrateAxleTrack(steering=50)
        self.CalibrateAxleTrack(steering=100)
        self.GenerateInitString()
        
    def GenerateInitString(self):
        s='\n======   Robot Init String  =========='
        s+='\nr=myLib.myRobot(loggingLevel='+str(self.loggingLevel)+',wheelDiameter='+str(self.wheelDiameter)+',axleTrack=['+str(self.axleTrack50)+','+str(self.axleTrack100)+'],wheelsToSensorsdistance='+str(self.wheelsToSensorsdistance)+',sensorToSensorDistance='+str(self.sensorToSensorDistance)+')'
        s+='\n'
        self.logger.Warning(s)


    def Steering(self, steering, power):
        """
        Μετατρέπει τα (steering & power) σε κατάλληλες τιμές (LeftPower,RightPower)
        
        Parameters:
            steering: Το steering εισόδου
            power:  Το Power εισόδου
        return:
         (float,float): (leftPower,rightPower)
        """
        if type(power)!=type([]):
            power_left = power_right = power
            s = (50 - abs(float(steering))) / 50
            if steering >= 0:
                power_right *= s
                if steering > 100:
                    power_right = - power
            else:
                power_left *= s
                if steering < -100:
                    power_left = - power
            return (int(power_left), int(power_right)) 
        else:
            out=[]
            for p in power:
                power_left = power_right = p
                s = (50 - abs(float(steering))) / 50
                if steering >= 0:
                    power_right *= s
                    if steering > 100:
                        power_right = - p
                else:
                    power_left *= s
                    if steering < -100:
                        power_left = - p
                out.append((int(power_left), int(power_right)) )
            return (out)

    def Steering2(self, steering, power):
        """
        Μετατρέπει τα (steering & power) σε κατάλληλες τιμές (LeftPower,RightPower)
        
        Parameters:
            steering: Το steering εισόδου
            power:  Το Power εισόδου
        return:
         (float,float): (leftPower,rightPower)
        """
        if steering >= 0:
            if steering > 100:
                power_right = 0
                power_left = power
            else:
                power_left = power
                power_right = power - ((power * steering) / 100)
        else:
            if steering < -100:
                power_left = 0
                power_right = power
            else:
                power_right = power
                power_left = power + ((power * steering) / 100)
        return (int(power_left), int(power_right))

    def CalcSteering(self,powerLeft=50,powerRight=50):
        """Υπολογίζει τα power, steering για συγκεκριμμένα LeftPower, RightPower 

        Args:
            powerLeft (int, optional): [description]. Defaults to 50.
            powerRight (int, optional): [description]. Defaults to 50.
        """
        if abs(powerRight)>abs(powerLeft):
            power=powerRight
            steering=(powerLeft/powerRight-1)*50
        elif abs(powerRight)<abs(powerLeft):
            power=powerLeft
            steering=(1-powerRight/powerLeft)*50
        elif powerRight>powerLeft:
            power=powerRight
            steering=(powerLeft/powerRight-1)*50
        else :
            power=powerLeft
            steering=(1-powerRight/powerLeft)*50
    
        if steering>100:
            steering=100
        if steering<-100:
            steering=-100
            
        return(power,steering)

    def __CalcSynchroDistances(self, vLeft=[10,50,20], vRight=[10,50,20], s1=0, s2=0, s3=0):
        """[summary]

        Args:
            vLeft (list, optional): [description]. Defaults to [10,50,20].
            vRight (list, optional): [description]. Defaults to [10,50,20].
            s1 (int, optional): [description]. Defaults to 0.
            s2 (int, optional): [description]. Defaults to 0.
            s3 (int, optional): [description]. Defaults to 0.

        Returns:
            [type]: [description]
        """
        if sum(vLeft) < 0:
            sl = -1
        else:
            sl = 1
        if sum(vRight) < 0:
            sr = -1
        else:
            sr = 1

        if abs(sum(vLeft)) > abs(sum(vRight)):
            tl=self.calcTimeShots(vLeft[0],vLeft[1],vLeft[2],s1,s2,s3)
            t1 = tl[0]
            t2 = tl[1]
            t3 = tl[2]
            sa=self.calcDistances(vRight[0],vRight[1],vRight[2],t1,t2,t3)
            s1a = sa[0]
            s2a = sa[1]
            s3a = sa[2]
            return (s1 * sl, s2 * sl, s3 * sl, s1a, s2a, s3a)
        elif abs(sum(vLeft)) < abs(sum(vRight)):
            tl=self.calcTimeShots(vRight[0],vRight[1],vRight[2],s1,s2,s3)
            t1 = tl[0]
            t2 = tl[1]
            t3 = tl[2]
            sa=self.calcDistances(vLeft[0],vLeft[1],vLeft[2],t1,t2,t3)
            s1a = sa[0]
            s2a = sa[1]
            s3a = sa[2]
            return (s1a, s2a, s3a, s1 * sr, s2 * sr, s3 * sr)
        else:
            return (abs(s1) * sl, abs(s2) * sl, abs(s3) * sl, abs(s1) * sr, abs(s2) * sr, abs(s3) * sr)

    def MoveSteering2(self, steering=0, power=[15, 40, 15], s1=0, s2=0, s3=0, kp=3.5, kd=1, dt=0.01, waits=[], stopAtEnd=True, stop_action='break', comments=''):
        """
        Η βασική μέθοδος που ελέγχει την πορεία του Ρομπότ

        Parameters:
            steering: από -100 (αριστερό spin) μέχρι 100 (δεξιό spin)
            power: λίστα με την εξέλιξη της κίνησης. power[vStart,vMax,vFinish]. 
                    Η κίνηση ξεκινάει με αρχική ταχύτητα vStart. 
                    Επιταχύνουμε μέχρι vMax (για απόσταση s1).
                    Συνεχίζουμε με ταχύτητα vMax (για απόσταση s2)
                    Επιβραδύνουμε μέχρι vFinish (για απόσταση s3) 
            s1: Η απόσταση για την οποία εφαρμόζουμς επιταχυνόμενη κίνηση
            s2: Η απόσταση για την οποία εφαρμόζουμε σταθερή ταχύτητα.
            s3: Η απόσταση για την οποία εφαρμόζουμε επιβραδυνόμενη κίνηση.
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
        :return: Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        """
        start0=time.perf_counter()
        self.waits=waits
        self.action=None
        #self.FL_Completed = False
        if self.gyro.initialized:
            startAngle=self.gyro.GetAngle()
        # s1=self.DistanceToDegrees(s1)
        # s2=self.DistanceToDegrees(s2)
        # s3=self.DistanceToDegrees(s3)
        bCompleted = False
        cCompleted = False
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        posB0 = self.motorB.GetPosition()
        posC0 = self.motorC.GetPosition()
        s = self.Steering(steering=steering, power=power)
        res = self.__CalcSynchroDistances([s[0][0],s[1][0],s[2][0]], [s[0][1],s[1][1],s[2][1]], self.DistanceToDegrees(s1), self.DistanceToDegrees(s2), self.DistanceToDegrees(s3))
        st = self.name+'\n| MoveSteering(steering='+str(steering)+', power='+self.ListToStr(power)+', s1='+str(s1)+', s2='+str(s2)+', s3='+str(s3)
        st += ', kp='+str(kp)+', kd='+str(kd)+', waits='+self.ListToStr(waits)+')'
        st1=''
        # print('res:',res)
        bTotalDistance = abs(res[0]) + abs(res[1]) + abs(res[2])
        cTotalDistance = abs(res[3]) + abs(res[4]) + abs(res[5])
        # self.logger.Debug('bTotalDistance: '+str(bTotalDistance)+' cTotalDistance: '+str(cTotalDistance))
        i = 0
        j = 0        
        if s[0][0]+s[1][0]+s[2][0] == 0:
            bCompleted = True
        if s[0][1]+s[1][1]+s[2][1] == 0:
            cCompleted = True

        # η επόμενη εντολή αποτελεί αρχικοποίηση του αλγόριθμου PID
        pidB=myPID(name='motorB.MoveSteering pid',kp=kp,kd=kd,direction=1)
        pidC=myPID(name='motorC.MoveSteering pid',kp=kp,kd=kd,direction=1)
        markDistance=0
        if len(waits)==0:
            markDistance=(bTotalDistance+cTotalDistance)/2
        if self.gyro.initialized:
            startAngle=self.gyro.GetAngle()
        start = time.perf_counter()
        start1=start
        while True:                          
            i += 1
            t1 = time.perf_counter()
            l1 = self.CalcDistanceAndSpeed(s[0][0]*self.motorB.speedRatio , s[1][0]*self.motorB.speedRatio, s[2][0]*self.motorB.speedRatio,
                                              abs(res[0]), abs(res[1]), abs(res[2]), t1 - start)
            #print(l1)
            target = l1[0]  # ideal position
            refReadB = self.motorB.GetPosition() - posB0  # position so far
            if abs(refReadB) >= bTotalDistance:
                bCompleted = True
            course = pidB.Step(inputRead=refReadB, target=target)
            nextSpeed = l1[1] /self.motorB.speedRatio + course
            if nextSpeed > 100 :
                nextSpeed = 100 
            if nextSpeed < -100 :
                nextSpeed = -100 
            self.motorB.On(power=nextSpeed,unregulated=True)
            # self.motorB.SetSpeed(speed=nextSpeed)
            
            t1 = time.perf_counter()
            t1 = self.CalcDistanceAndSpeed(s[0][1]*self.motorC.speedRatio , s[1][1]*self.motorC.speedRatio , s[2][1]*self.motorC.speedRatio ,
                                               abs(res[3]), abs(res[4]), abs(res[5]), t1 - start)
            target = l1[0]  # ideal position
            refReadC = self.motorC.GetPosition() - posC0  # position so far
            if abs(refReadC) >= cTotalDistance:
                cCompleted = True
                
            course = pidC.Step( inputRead=refReadC, target=target)
            nextSpeed = l1[1]/self.motorC.speedRatio + course
            if nextSpeed > 100 :
                nextSpeed = 100 
            if nextSpeed < -100 :
                nextSpeed = -100 
            self.motorC.On(power=nextSpeed,unregulated=True)
            # self.motorC.SetSpeed(speed=nextSpeed)
            
            if bCompleted and cCompleted:
                self.soDegrees= (abs(refReadB) + abs(refReadC)) / 2
                if self.actionsDone():
                    break


            self.timer.Step()
            
        if stopAtEnd:
            self.Stop()
        start2 = time.perf_counter()
        degrB=self.motorB.GetDegrees()
        degrC=self.motorC.GetDegrees()
        soDistance = (abs(degrB) + abs(degrC)) / 2
        extraDistance=soDistance-markDistance        
        if self.logger.level<=2:
            if comments!="":
                st +="\n| --> "+comments+" <--"
                st +="\n=================================================================="
            st += '\n| dt='+str((time.perf_counter()-start)/i)
            st += '\n| motorB:' + str(degrB) + ' (' + str(self.DegreesToDistance(degrB)) + ' mm)'
            st += '\n| motorC:' + str(degrC) + ' (' + str(self.DegreesToDistance(degrC)) + ' mm)'
            st += '\n| degrees/dt:'+str((degrB + degrC) / 2 / i)
            st += "\n| soDistance= "+str(soDistance)+' dgs ('+str(self.DegreesToDistance(soDistance))+' mm)'
            st += "\n| markDistance=" +str(markDistance)+' dgs '+str(self.DegreesToDistance(markDistance))+' mm'
            st += "\n| extraDistance= "+str(extraDistance)+' dgs  '+str(self.DegreesToDistance(extraDistance))+' mm'
            if self.gyro.initialized:
                endAngle=self.gyro.GetAngle()
                st += "\n| AngleFromGyro= "+str(endAngle-startAngle)
                if endAngle!=startAngle:
                    st += "\n| DegreesPerAngle= "+str(soDistance/(endAngle-startAngle))
            st += "\n=========================================================="
            if self.logger.level<=1:
                st1 += pidB.GetStatistics()
                st1 += pidC.GetStatistics()
            self.logger.Info(st1)
            self.logger.Debug(st)
            self.logger.Save()
        start3=time.perf_counter()
        preTime=start1-start0
        loopTime=start2-start1
        postTime=start3-start2        
        self.logger.Debug("PreTime:"+str(preTime)+ " loopsTime:"+str(loopTime)+" postTime:"+str(postTime))
        return(extraDistance)

    def MoveSteering3(self, steering=0, power=[15, 40, 15], s1=0, s2=0, s3=0, kp=3.5, kd=1, dt=0.01, stopAtEnd=True, stop_action='break', comments=''):
        """
        Η βασική μέθοδος που ελέγχει την πορεία του Ρομπότ

        Parameters:
            steering: από -100 (αριστερό spin) μέχρι 100 (δεξιό spin)
            power: λίστα με την εξέλιξη της κίνησης. power[vStart,vMax,vFinish]. 
                    Η κίνηση ξεκινάει με αρχική ταχύτητα vStart. 
                    Επιταχύνουμε μέχρι vMax (για απόσταση s1).
                    Συνεχίζουμε με ταχύτητα vMax (για απόσταση s2)
                    Επιβραδύνουμε μέχρι vFinish (για απόσταση s3) 
            s1: Η απόσταση για την οποία εφαρμόζουμς επιταχυνόμενη κίνηση
            s2: Η απόσταση για την οποία εφαρμόζουμε σταθερή ταχύτητα.
            s3: Η απόσταση για την οποία εφαρμόζουμε επιβραδυνόμενη κίνηση.
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
        :return: Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        """
        start0=time.perf_counter()
        #self.FL_Completed = False
        if self.gyro.initialized:
            startAngle=self.gyro.GetAngle()
        s1=self.DistanceToDegrees(s1)
        s2=self.DistanceToDegrees(s2)
        s3=self.DistanceToDegrees(s3)
        action=None
        bCompleted = False
        cCompleted = False
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        posB0 = self.motorB.GetPosition()
        posC0 = self.motorC.GetPosition()
        s = self.Steering(steering=steering, power=power)
        res = self.__CalcSynchroDistances([s[0][0],s[1][0],s[2][0]], [s[0][1],s[1][1],s[2][1]], s1, s2, s3)
        st = self.name+'\n| MoveSteering(steering='+str(steering)+', power='+self.ListToStr(power)+', s1='+str(s1)+', s2='+str(s2)+', s3='+str(s3)
        st += ', kp='+str(kp)+', kd='+str(kd)+')'
        st1=''
        # print('res:',res)
        bTotalDistance = abs(res[0]) + abs(res[1]) + abs(res[2])
        cTotalDistance = abs(res[3]) + abs(res[4]) + abs(res[5])
        # self.logger.Debug('bTotalDistance: '+str(bTotalDistance)+' cTotalDistance: '+str(cTotalDistance))
        i = 0
        j = 0        
        if s[0][0]+s[1][0]+s[2][0] == 0:
            bCompleted = True
        if s[0][1]+s[1][1]+s[2][1] == 0:
            cCompleted = True

        # η επόμενη εντολή αποτελεί αρχικοποίηση του αλγόριθμου PID
        pidB=myPID(name='motorB.MoveSteering pid',kp=kp,kd=kd,direction=1)
        pidC=myPID(name='motorC.MoveSteering pid',kp=kp,kd=kd,direction=1)
        markDistance=0
        markDistance=(bTotalDistance+cTotalDistance)/2
        if self.gyro.initialized:
            startAngle=self.gyro.GetAngle()
        start = time.perf_counter()
        start1=start
        while True:    
            t0=time.perf_counter()           
            i += 1
            if not bCompleted:
                t1 = time.perf_counter()
                l1 = self.CalcDistanceAndSpeed(s[0][0]*self.motorB.speedRatio , s[1][0]*self.motorB.speedRatio, s[2][0]*self.motorB.speedRatio,
                                              abs(res[0]), abs(res[1]), abs(res[2]), t1 - start)
                #print(l1)
                target = l1[0]  # ideal position
                refReadB = self.motorB.GetPosition() - posB0  # position so far
                if abs(refReadB) >= bTotalDistance:
                    bCompleted = True
                    # if not stopAtEnd:
                    #     nextSpeed = l1[1]
                    #     self.motorB.On(power=nextSpeed,unregulated=True)
                else:
                    course = pidB.Step(inputRead=refReadB, target=target)
                    nextSpeed = l1[1] /self.motorB.speedRatio + course
                    if nextSpeed > 100 :
                        nextSpeed = 100 
                    if nextSpeed < -100 :
                        nextSpeed = -100 
                    self.motorB.On(power=nextSpeed,unregulated=True)
            if not cCompleted:
                t1 = time.perf_counter()
                l1 = self.CalcDistanceAndSpeed(s[0][1]*self.motorC.speedRatio , s[1][1]*self.motorC.speedRatio , s[2][1]*self.motorC.speedRatio ,
                                               abs(res[3]), abs(res[4]), abs(res[5]), t1 - start)
                #print(l1)
                target = l1[0]  # ideal position
                refReadC = self.motorC.GetPosition() - posC0  # position so far
                if abs(refReadC) >= cTotalDistance:
                    cCompleted = True
                    # if not stopAtEnd:
                    #     nextSpeed = l1[1]
                    #     self.motorC.On(power=nextSpeed,unregulated=True)
                else:
                    course = pidC.Step( inputRead=refReadC, target=target)
                    nextSpeed = l1[1]/self.motorC.speedRatio + course
                    if nextSpeed > 100 :
                        nextSpeed = 100 
                    if nextSpeed < -100 :
                        nextSpeed = -100 
                    self.motorC.On(power=nextSpeed,unregulated=True)
            
            if bCompleted and cCompleted:
                break


            t1=time.perf_counter()
            a=dt-t1+t0
            if a>0:
                wait(a*1000) 
        if stopAtEnd:
            self.Stop()
        start2 = time.perf_counter()
        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        soDistance = (abs( distB)+ abs(distC)) / 2
        extraDistance=soDistance-markDistance        
        if self.logger.level<=2:
            if comments!="":
                st +="\n| --> "+comments+" <--"
                st +="\n=================================================================="
            st += '\n| dt='+str((time.perf_counter()-start)/i)
            st += '\n| motorB:'+str(distB)
            st += '\n| motorC:'+str(distC)
            st += '\n| degrees/dt:'+str((distB+distC)/2/i)
            st += "\n| soDistance= "+str(soDistance)+' dgs '+str(self.DegreesToDistance(soDistance))+' mm'
            st += "\n| markDistance=" +str(markDistance)+' dgs '+str(self.DegreesToDistance(markDistance))+' mm'
            st += "\n| extraDistance= "+str(extraDistance)+' dgs  '+str(self.DegreesToDistance(extraDistance))+' mm'
            if self.gyro.initialized:
                endAngle=self.gyro.GetAngle()
                st += "\n| AngleFromGyro= "+str(endAngle-startAngle)
                if endAngle!=startAngle:
                    st += "\n| DegreesPerAngle= "+str(soDistance/(endAngle-startAngle))
            st += "\n=========================================================="
            if self.logger.level<=1:
                st1 += pidB.GetStatistics()
                st1 += pidC.GetStatistics()
            self.logger.Info(st1)
            self.logger.Debug(st)
            self.logger.Save()
        start3=time.perf_counter()
        preTime=start1-start0
        loopTime=start2-start1
        postTime=start3-start2        
        self.logger.Debug("PreTime:"+str(preTime)+ " loopsTime:"+str(loopTime)+" postTime:"+str(postTime))
        return(extraDistance)

    def MoveSteering4(self, steering=0, power=[15, 40, 15], s1=0, s2=0, s3=0, kp=3.5, kd=1, dt=0.01,  stopAtEnd=True, stop_action='break', comments=''):
        """
        Η βασική μέθοδος που ελέγχει την πορεία του Ρομπότ

        Parameters:
            steering: από -100 (αριστερό spin) μέχρι 100 (δεξιό spin)
            power: λίστα με την εξέλιξη της κίνησης. power[vStart,vMax,vFinish]. 
                    Η κίνηση ξεκινάει με αρχική ταχύτητα vStart. 
                    Επιταχύνουμε μέχρι vMax (για απόσταση s1).
                    Συνεχίζουμε με ταχύτητα vMax (για απόσταση s2)
                    Επιβραδύνουμε μέχρι vFinish (για απόσταση s3) 
            s1: Η απόσταση για την οποία εφαρμόζουμς επιταχυνόμενη κίνηση
            s2: Η απόσταση για την οποία εφαρμόζουμε σταθερή ταχύτητα.
            s3: Η απόσταση για την οποία εφαρμόζουμε επιβραδυνόμενη κίνηση.
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
        :return: Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        """
        start0=time.perf_counter()
        #self.FL_Completed = False
        if self.gyro.initialized:
            startAngle=self.gyro.GetAngle()
        s1=self.DistanceToDegrees(s1)
        s2=self.DistanceToDegrees(s2)
        s3=self.DistanceToDegrees(s3)
        action=None
        bCompleted = False
        cCompleted = False
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        posB0 = self.motorB.GetPosition()
        posC0 = self.motorC.GetPosition()
        s = self.Steering(steering=steering, power=power)
        res = self.__CalcSynchroDistances([s[0][0],s[1][0],s[2][0]], [s[0][1],s[1][1],s[2][1]], s1, s2, s3)
        st = self.name+'\n| MoveSteering(steering='+str(steering)+', power='+self.ListToStr(power)+', s1='+str(s1)+', s2='+str(s2)+', s3='+str(s3)
        st += ', kp='+str(kp)+', kd='+str(kd)+')'
        st1=''
        # print('res:',res)
        bTotalDistance = abs(res[0]) + abs(res[1]) + abs(res[2])
        cTotalDistance = abs(res[3]) + abs(res[4]) + abs(res[5])
        # self.logger.Debug('bTotalDistance: '+str(bTotalDistance)+' cTotalDistance: '+str(cTotalDistance))
        i = 0
        j = 0        
        if s[0][0]+s[1][0]+s[2][0] == 0:
            bCompleted = True
        if s[0][1]+s[1][1]+s[2][1] == 0:
            cCompleted = True

        # η επόμενη εντολή αποτελεί αρχικοποίηση του αλγόριθμου PID
        pidB=myPID(name='motorB.MoveSteering pid',kp=kp,kd=kd,direction=1)
        pidC=myPID(name='motorC.MoveSteering pid',kp=kp,kd=kd,direction=1)
        markDistance=0
        markDistance=(bTotalDistance+cTotalDistance)/2
        if self.gyro.initialized:
            startAngle=self.gyro.GetAngle()
        start = time.perf_counter()
        start1=start
        x=self.color2.GetColor(method=3)
        while x not in (6):    
            t0=time.perf_counter()           
            i += 1
            if not bCompleted:
                t1 = time.perf_counter()
                l1 = self.CalcDistanceAndSpeed(s[0][0]*self.motorB.speedRatio , s[1][0]*self.motorB.speedRatio, s[2][0]*self.motorB.speedRatio,
                                              abs(res[0]), abs(res[1]), abs(res[2]), t1 - start)
                #print(l1)
                target = l1[0]  # ideal position
                refReadB = self.motorB.GetPosition() - posB0  # position so far
                if abs(refReadB) >= bTotalDistance:
                    bCompleted = True
                    # if not stopAtEnd:
                    #     nextSpeed = l1[1]
                    #     self.motorB.On(power=nextSpeed,unregulated=True)
                else:
                    course = pidB.Step(inputRead=refReadB, target=target)
                    nextSpeed = l1[1] /self.motorB.speedRatio + course
                    if nextSpeed > 100 :
                        nextSpeed = 100 
                    if nextSpeed < -100 :
                        nextSpeed = -100 
                    self.motorB.On(power=nextSpeed,unregulated=True)
            if not cCompleted:
                t1 = time.perf_counter()
                l1 = self.CalcDistanceAndSpeed(s[0][1]*self.motorC.speedRatio , s[1][1]*self.motorC.speedRatio , s[2][1]*self.motorC.speedRatio ,
                                               abs(res[3]), abs(res[4]), abs(res[5]), t1 - start)
                #print(l1)
                target = l1[0]  # ideal position
                refReadC = self.motorC.GetPosition() - posC0  # position so far
                if abs(refReadC) >= cTotalDistance:
                    cCompleted = True
                    # if not stopAtEnd:
                    #     nextSpeed = l1[1]
                    #     self.motorC.On(power=nextSpeed,unregulated=True)
                else:
                    course = pidC.Step( inputRead=refReadC, target=target)
                    nextSpeed = l1[1]/self.motorC.speedRatio + course
                    if nextSpeed > 100 :
                        nextSpeed = 100 
                    if nextSpeed < -100 :
                        nextSpeed = -100 
                    self.motorC.On(power=nextSpeed,unregulated=True)
            
            if bCompleted and cCompleted:
                pass


            t1=time.perf_counter()
            a=dt-t1+t0
            if a>0:
                wait(a*1000) 
            xcol=self.color2.GetColor(color=2)
        if stopAtEnd:
            self.Stop()
        start2 = time.perf_counter()
        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        soDistance = (abs( distB)+ abs(distC)) / 2
        extraDistance=soDistance-markDistance        
        if self.logger.level<=2:
            if comments!="":
                st +="\n| --> "+comments+" <--"
                st +="\n=================================================================="
            st += '\n| dt='+str((time.perf_counter()-start)/i)
            st += '\n| motorB:'+str(distB)
            st += '\n| motorC:'+str(distC)
            st += '\n| degrees/dt:'+str((distB+distC)/2/i)
            st += "\n| soDistance= "+str(soDistance)+' dgs '+str(self.DegreesToDistance(soDistance))+' mm'
            st += "\n| markDistance=" +str(markDistance)+' dgs '+str(self.DegreesToDistance(markDistance))+' mm'
            st += "\n| extraDistance= "+str(extraDistance)+' dgs  '+str(self.DegreesToDistance(extraDistance))+' mm'
            if self.gyro.initialized:
                endAngle=self.gyro.GetAngle()
                st += "\n| AngleFromGyro= "+str(endAngle-startAngle)
                if endAngle!=startAngle:
                    st += "\n| DegreesPerAngle= "+str(soDistance/(endAngle-startAngle))
            st += "\n=========================================================="
            if self.logger.level<=1:
                st1 += pidB.GetStatistics()
                st1 += pidC.GetStatistics()
            self.logger.Info(st1)
            self.logger.Debug(st)
            self.logger.Save()
        start3=time.perf_counter()
        preTime=start1-start0
        loopTime=start2-start1
        postTime=start3-start2        
        self.logger.Debug("PreTime:"+str(preTime)+ " loopsTime:"+str(loopTime)+" postTime:"+str(postTime))
        return(extraDistance)

    def Stop(self,stop_action='brake'):
        """[summary]

        Args:
            stop_action (str, optional): [description]. Defaults to 'brake'.
        """
        # self.motorB.Off(stop_action=stop_action)
        # self.motorC.Off(stop_action=stop_action)            
        self.motorB.SetSpeed(speed=0)
        self.motorC.SetSpeed(speed=0)
        # περίμενε έχρι να σταματήσει τελείως
        tt0=time.perf_counter()
        pos=-999999999999999
        while pos!=self.motorB.GetDegrees()+self.motorC.GetDegrees():
            pos=self.motorB.GetDegrees()+self.motorC.GetDegrees()
            self.motorB.SetSpeed(speed=0)
            self.motorC.SetSpeed(speed=0)
            #self.motorB.Off(stop_action=stop_action)
            #self.motorC.Off(stop_action=stop_action)  
            #time.sleep(0.01)
        self.motorB.Off(stop_action=stop_action)
        self.motorC.Off(stop_action=stop_action)
        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        self.logger.Debug('Stop --> motorB:'+str(distB)+' ('+str(self.DegreesToDistance(distB))+' mm) - '+'motorC:'+str(distC)+' ('+str(self.DegreesToDistance(distC))+' mm) ')
        self.logger.Debug('Stop --> time to stop:'+str(time.perf_counter()-tt0))

    def actionsDone1(self):
        """[summary]
        """
        # αν δεν υπάρχει ενεργό action Βγάλε ένα απο την λίστα waits
        if self.action==None:
            if len(self.waits)>0:
                self.action = self.waits.pop(0)
                #print(action, type(action))
                if type(self.action) == int:
                    # αν είναι αριθμός κάνε μετατροπή απο mm σε μοιρες
                    self.action=int(self.DistanceToDegrees(self.action))
        # αν δεν υπάρχει action ολοκληρώνεται η διαδικασία με break
        if self.action==None:
            return(1)
        # αλλιώς υλοποιείτε το αντίστοιχο action    
        else:
            # αν το action είναι αριθμός 
            if type(self.action) == int:
                # αν έχει φτάσει στην απόσταση του action
                if self.action+self.markDegrees<=self.soDegrees:
                        # μάρκαρε την απόσταση 
                        self.markDegrees=self.soDegrees
                        self.logger.Debug(' waits.action ' + str(self.action) +' dgs (' + str(self.DegreesToDistance(self.action)) +' mm) real Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                        self.action=None
                        
                        if len(self.waits)>0:
                            self.action = self.waits.pop(0)
                            
            
            if type(self.action)==str:
                # δημιούργησε απο το string το ζεύγος key,value
                kv=self.action.split('=')
                key=kv[0]
                values=kv[1].split(';')
                value=values[0]
                self.Print(key,value)
                # αν το key είναι speed (τροποποίηση ταχύτητας)
                if key=='speed':                    
                    self.maxSpeed=float(value)
                    self.action=None
                # να το key είναι angle ελέγχει για γωνία.
                if key=='angle':
                    # self.Print(self.gyro.GetAngle(),' ',value)
                    ang=float(value)
                    if ang>0:
                        if abs( self.gyro.GetAngle()>= float(value)) :
                            self.action=None
                            return(0)
                    else:
                        if abs( self.gyro.GetAngle()<= float(value)) :
                            self.action=None
                            return(0)

                # αν το key είναι black (έλεγχος για μαύρο σε αισθητήρα χρώμματος). 
                # value είναι ο αριθμός του αισθητήρα που θέλουμε να κάνουμε έλεγχο για μαύρο
                # πχ 'black=2;40'  σημαίνει μαύρο στον 2 με τιμή μικρότερη το 40 
                elif key=='black':
                    # αν δεν υπάρχει ';' μαύρο θεωρείτε κάτω απο 50 
                    if len(values)<=1:
                        threshold=50
                    else:
                        threshold=float(values[1])
                    if value in ('1','2','3','4'):
                        if value=='1':
                            colorsen=self.color1
                        elif value=='2':
                            colorsen=self.color2
                        elif value=='3':
                            colorsen=self.color3
                        elif value=='4':
                            colorsen=self.color4

                        if colorsen.GetLight()<threshold:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())
                    
                    
                    # 23 σημαίνει ότι ψάχνουμε για μαύρο ταυτόχρονα σε αισθητήρες 2,3 ταυτόχρονα
                    elif value=='23':
                        if self.color2.value<threshold and self.color3.value<threshold:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())
                # αν το key είναι white σημαίνει ότι ψάχνουμε λευκό σε αισθητήρα χρώμματος  
                elif key=='white':
                    # αν δεν υπάρχει ';' μαύρο θεωρείτε κάτω απο 50 
                    if len(values)<=1:
                        threshold=50
                    else:
                        threshold=float(values[1])
                    if value in ('1','2','3','4'):
                        if value=='1':
                            colorsen=self.color1
                        elif value=='2':
                            colorsen=self.color2
                        elif value=='3':
                            colorsen=self.color3
                        elif value=='4':
                            colorsen=self.color4

                        if colorsen.GetLight()>threshold:
                            self.markDegrees=self.soDegrees
                            self.action=None
                    elif value=='23':
                        if self.color2.value>threshold and self.color3.value>threshold:
                            self.markDegrees=self.soDegrees
                            self.action=None
                    else:
                        self.logger.Error('waits.action ',key, ' not recognized as a valid action')
                # αν το key αλλάζουμε το kp του pid 
                elif key=='kp':
                    self.pid.kp=float(value)
                    self.action=None
                    if len(self.waits)==0:
                        return(1)
                    else:
                        return(self.actionsDone())
                # αν το key αλλάζουμε το ki του pid 
                elif key=='ki':
                    self.pid.ki=float(value)
                    self.action=None
                    if len(self.waits)==0:
                        return(1)
                    else:
                        return(self.actionsDone())
                # αν το key αλλάζουμε το kp του pid 
                elif key=='kd':
                    self.pid.kd=float(value)
                    self.action=None
                    if len(self.waits)==0:
                        return(1)
                    else:
                        return(self.actionsDone())
                # αν το key αλλάζουμε το dt του timer του robot 
                elif key=='dt':
                    self.timer.dt=float(value)
                    self.action=None
                # αν το key αλλάζουμε το zero Error Steering της ακολούθησης 
                elif key=='zes':
                    self.zeroErrorSteering=float(value)
                    self.action=None

                elif key in ('red','green','blue','yellow','whitecolor'):
                    scolor=0
                    if key=='red':
                        scolor=5
                    elif key=='blue':
                        scolor=2
                    elif key=='green':
                        scolor=3
                    elif key=='yellow':
                        scolor=4
                    else:
                        scolor=6
                    if value=='1':
                        colorsen=self.color1
                    elif value=='2':
                        colorsen=self.color2
                    elif value=='3':
                        colorsen=self.color3
                    elif value=='4':
                        colorsen=self.color4

                    if colorsen.GetColor(method=3)==scolor:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())

                elif key in ('nored','nogreen','noblue','noyellow','nowhitecolor'):
                    scolor=0
                    if key=='nored':
                        scolor=5
                    elif key=='noblue':
                        scolor=2
                    elif key=='nogreen':
                        scolor=3
                    elif key=='yellow':
                        scolor=4
                    else:
                        scolor=6
                    if value=='1':
                        colorsen=self.color1
                    elif value=='2':
                        colorsen=self.color2
                    elif value=='3':
                        colorsen=self.color3
                    elif value=='4':
                        colorsen=self.color4

                    if colorsen.GetColor(method=3)!=scolor:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())

                # πχ color3=4,5  (χρώμμα στο αισθητήρα 3 πορτοκαλί ή κόκκινο)
                elif key in ('color1','color2','color3','color4'):
                    if key=='color1':
                        colorsen=self.color1
                    elif key=='color2':
                        colorsen=self.color2
                    elif key=='color3':
                        colorsen=self.color3
                    elif key=='color4':
                        colorsen=self.color4

                    colors=value.split(',')
                    if str(colorsen.GetColor(method=3)) in colors:
                        self.markDegrees=self.soDegrees
                        self.action=None
                        self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                        if len(self.waits)==0:
                            return(1)
                        else:
                            return(self.actionsDone())

                elif key=='getlight':
                    if value in ('1','2','3','4'):
                        if value=='1':
                            colorsen=self.color1
                        elif value=='2':
                            colorsen=self.color2
                        elif value=='3':
                            colorsen=self.color3
                        elif value=='4':
                            colorsen=self.color4
                    else:
                        self.logger.Error(log='No such port: '+value)
                        return(1)
                    colorsen.GetLight()
                    colorsen.PushLight()
                    self.action=None
                elif key=='getcolor':
                    if value in ('1','2','3','4'):
                        if value=='1':
                            colorsen=self.color1
                        elif value=='2':
                            colorsen=self.color2
                        elif value=='3':
                            colorsen=self.color3
                        elif value=='4':
                            colorsen=self.color4
                    else:
                        self.logger.Error(log='No such port: '+value)
                        return(1)
                    colorsen.GetColor(lightThreshold=5,readCount=1)
                    colorsen.PushColor()
                    self.action=None
                else:
                    self.logger.Error(log='No such action: '+key)
                    self.action=None
                    return(1)                       


                
                # αν έχει ολοκληρωθεί το action εμφάνισε μήνυμα ολοκλήρωσης    
                if self.action==None:
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
        # επιστροφή 0 σημαίνει ότι δεν έχουν ολοκληρωθεί τα waits
        return(0)

    def actionsDone(self):
        """[summary]
        """
        # αν δεν υπάρχει ενεργό action Βγάλε ένα απο την λίστα waits
        if self.action==None:
            if len(self.waits)>0:
                self.action = self.waits.pop(0)
                #print(action, type(action))
                if type(self.action) == int:
                    # αν είναι αριθμός κάνε μετατροπή απο mm σε μοιρες
                    self.action=int(self.DistanceToDegrees(self.action))
        # αν δεν υπάρχει action ολοκληρώνεται η διαδικασία με break
        if self.action==None:
            return(1)
        # αλλιώς υλοποιείτε το αντίστοιχο action    
        else:
            # αν το action είναι αριθμός 
            if type(self.action) == int:
                # αν έχει φτάσει στην απόσταση του action
                if self.action+self.markDegrees<=self.soDegrees:
                        # μάρκαρε την απόσταση 
                        self.markDegrees=self.soDegrees
                        self.logger.Debug(' waits.action ' + str(self.action) +' dgs (' + str(self.DegreesToDistance(self.action)) +' mm) real Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                        self.action=None
                        return(0)
            # # αν το action είναι αριθμός
            
            
            
            if type(self.action)==str:
                # δημιούργησε απο το string το ζεύγος key,value
                kv=self.action.split('=')
                key=kv[0]
                value=kv[1]
                # self.Print(key,' ',value)
                # αν το key είναι speed (τροποποίηση ταχύτητας)
                if key=='speed':                    
                    self.maxSpeed=float(value)
                    self.action=None
                # να το key είναι angle ελέγχει για γωνία.
                if key=='angle':
                    # self.Print(self.gyro.GetAngle(),' ',value)
                    ang=float(value)
                    if ang>0:
                        if abs( self.gyro.GetAngle()>= float(value)) :
                            self.action=None
                            return(0)
                    else:
                        if abs( self.gyro.GetAngle()<= float(value)) :
                            self.action=None
                            return(0)

                # αν το key είναι black (έλεγχος για μαύρο σε αισθητήρα χρώμματος). 
                # value είναι ο αριθμός του αισθητήρα που θέλουμε να κάνουμε έλεγχο για μαύρο
                elif key=='black':
                    if value=='1':
                        if self.color1.GetLight()<50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())
                    
                    elif value=='2':
                        if self.color2.GetLight()<50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())
                    
                    elif value=='3':
                        if self.color3.GetLight()<50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())
                    
                    elif value=='4':
                        if self.color4.GetLight()<50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())
                    
                    # 23 σημαίνει ότι ψάχνουμε για μαύρο ταυτόχρονα σε αισθητήρες 2,3 ταυτόχρονα
                    elif value=='23':
                        if self.color2.value<50 and self.color3.value<50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())


                elif key in ('red','green','blue','yellow','blackcolor','whitecolor'):
                    scolor=0
                    if key=='red':
                        scolor=5
                    elif key=='blue':
                        scolor=2
                    elif key=='green':
                        scolor=3
                    elif key=='yellow':
                        scolor=4
                    elif key=='blackcolor':
                        scolor=1
                    elif key=='whitecolor':
                        scolor=6
                    else:
                        scolor=6
                    if value=='1':
                        colorsen=self.color1
                    elif value=='2':
                        colorsen=self.color2
                    elif value=='3':
                        colorsen=self.color3
                    elif value=='4':
                        colorsen=self.color4
                    thisColor=colorsen.GetColor(method=3)
                    # self.Print(thisColor)
                    if thisColor==scolor:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())


                elif key in ('nored','nogreen','noblue','noyellow','nowhitecolor'):
                    scolor=0
                    if key=='nored':
                        scolor=5
                    elif key=='noblue':
                        scolor=2
                    elif key=='nogreen':
                        scolor=3
                    elif key=='yellow':
                        scolor=4
                    else:
                        scolor=6
                    if value=='1':
                        colorsen=self.color1
                    elif value=='2':
                        colorsen=self.color2
                    elif value=='3':
                        colorsen=self.color3
                    elif value=='4':
                        colorsen=self.color4

                    if colorsen.GetColor(method=3)!=scolor:
                            self.markDegrees=self.soDegrees
                            self.action=None
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
                            if len(self.waits)==0:
                                return(1)
                            else:
                                return(self.actionsDone())


                # αν το key είναι white σημαίνει ότι ψάχνουμε λευκό σε αισθητήρα χρώμματος  
                elif key=='white':
                    if value=='1':
                        if self.color1.GetLight()>50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                    elif value=='2':
                        if self.color2.GetLight()>50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                    elif value=='3':
                        if self.color3.GetLight()>50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                    elif value=='4':
                        if self.color4.GetLight()>50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                    elif value=='23':
                        if self.color2.value>50 and self.color3.value>50:
                            self.markDegrees=self.soDegrees
                            self.action=None
                    else:
                        self.logger.Error('waits.action ',key, ' not recognized as a valid action')
                # αν το key αλλάζουμε το kp του pid 
                elif key=='kp':
                    self.pid.kp=float(value)
                    self.action=None
                    if len(self.waits)==0:
                        return(1)
                    else:
                        return(self.actionsDone())
                # αν το key αλλάζουμε το ki του pid 
                elif key=='ki':
                    self.pid.ki=float(value)
                    self.action=None
                    if len(self.waits)==0:
                        return(1)
                    else:
                        return(self.actionsDone())

                # αν το key αλλάζουμε το kp του pid 
                elif key=='kd':
                    self.pid.kd=float(value)
                    self.action=None
                    if len(self.waits)==0:
                        return(1)
                    else:
                        return(self.actionsDone())

                # αν το key αλλάζουμε το dt του timer του robot 
                elif key=='dt':
                    self.timer.dt=float(value)
                    self.action=None
                # αν το key αλλάζουμε το zero Error Steering της ακολούθησης 
                elif key=='zes':
                    self.zeroErrorSteering=float(value)
                    self.action=None
                
                elif key=='getlight':
                    if value in ('1','2','3','4'):
                        if value=='1':
                            colorsen=self.color1
                        elif value=='2':
                            colorsen=self.color2
                        elif value=='3':
                            colorsen=self.color3
                        elif value=='4':
                            colorsen=self.color4
                    else:
                        self.logger.Error(log='No such port: '+value)
                        return(1)
                    colorsen.GetLight()
                    colorsen.PushLight()
                    self.action=None
                elif key=='getcolor':
                    if value in ('1','2','3','4'):
                        if value=='1':
                            colorsen=self.color1
                        elif value=='2':
                            colorsen=self.color2
                        elif value=='3':
                            colorsen=self.color3
                        elif value=='4':
                            colorsen=self.color4
                    else:
                        self.logger.Error(log='No such port: '+value)
                        return(1)
                    colorsen.GetColor(lightThreshold=5,readCount=1)
                    colorsen.PushColor()
                    self.action=None
                 
                
                # αν έχει ολοκληρωθεί το action εμφάνισε μήνυμα ολοκλήρωσης    
                if self.action==None:
                            self.logger.Debug(' waits.action ' + key +' ' + value +' in Distance:' + str(self.soDegrees) + ' (' + str(self.DegreesToDistance(self.soDegrees)) + ' mm)')
        # επιστροφή 0 σημαίνει ότι δεν έχουν ολοκληρωθεί τα waits
        return(0)

    def FollowLine(self, ports=2, target=50, leftRight=-1, kp=0.1, ki=0, kd=0.3, dt=0.01, minDistance=0, maxDistance=9999999, minSpeed=10, maxSpeed=25,speedRatio=1, waits=[], comments="", useColor='white', stop_action='brake', stopAtEnd=True,centerLine=False,zeroErrorSteering=0,calcGyroAngleCorrection=False):
        """
        Η βασική μέθοδος ακολούθησης της γραμμής
        
        Parameters:
            ports: Οι πόρτες που είναι συνδεμένοι οι αισθητήρες που χρησιμοποιούνται στην ακολούθηση
            target: Η τιμή της αντανάκλασης που δεν χρειάζεται διόρθωση της πορείας
            leftRight: Η μεριά της γραμμής που γίνεται η ακολούθηση. (-1 για αριστερά, 1 για δεξιά)
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
            minDegrees: Η ελάχιστη απόσταση που θα διαρκέσει η διαδικασία ακολούθησης της γραμμής
            maxDegrees: Η μέγιστη απόσταση ακολούθησης της γραμμής
            minSpeed: Η αρχική ισχύς των μοτέρ κατα την εκκίνηση της διαδικασίας ακολούθησης
            maxSpeed: Η μέγιστη ισχύς των μοτέρ κατα την διαδικασία ακολούθησης
            stop_action: Ο τρόπος με τον οποίο σταματάνε τα μοτέρ στο τέλος της διαδικασίας ('brake','coast','hold')
            stopAtEnd: True: σβήσιμο των μοτέρ στο τέλος της διαδικασίας, False: τα μοτέρ παραμένουν ανοιχτά (περίπτωση που ακολουθεί νέα ακολούθηση)
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   black=1 σημαίνει περίμενε για μαύρο στον αισθητήρα 1
                   white=1 σημαίνει περίμενε για λευκό στον αισθητήρα 1
                   speed=30 σημαίνει άλλαξε την ταχύτητα σε 30
                   kp=0.2 σημαίνει άλλαξε το kp σε 0.2
                   όμοια kd=0
                   200 σημαίνει περίμενε γιαr 200 mm
                   για παράδειγμα  waits=['white=2','black=2',120] σημαίνει περίμενε για λευκό στον 2, μετά για μαυρο και τέλος για 120 mm)
        return:
            Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        
        """
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s+="\n| FollowLine(ports="+str(ports)+",target="+str(target)+",leftRight="+str(leftRight)+",kp="+str(kp)
        s +=",ki="+str(ki)+",kd="+str(kd)+",dt="+str(dt)+",minDistance="+str(minDistance)+", maxDistance="+str(maxDistance)
        s +=",minSpeed=" + str(minSpeed*speedRatio) + ",maxSpeed=" + str(maxSpeed*speedRatio) + ", waits=" + self.ListToStr(waits) + ",stop_action='" + stop_action + "')"
        if calcGyroAngleCorrection:
            self.gyro.values=[]
            self.gyro.i=0
        if centerLine:
            if target==0:
                self.CenterLine()
            elif ports in (2,23) and leftRight==-1:
                self.CenterLine()
            elif ports in (3,32) and leftRight==1:
                self.CenterLine()
            
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees=self.DistanceToDegrees(maxDistance)    
        self.timer.dt=dt
        self.action=None
        self.waits=waits
        if self.loggingLevel==1:
            # Αρχικοποίηση στατιστικών
            self.speedsB=[]
            self.speedsC=[]
            self.steeringList=[0,0,0]
            self.steeringListDistance=[0,0,0]
            for i in range(10):
                self.speedsB.append(0)
                self.speedsC.append(0)

        self.pid=myPID(name=self.name+'.FollowLine (pid)',kp=kp,ki=ki,kd=kd,loggingStats=True,loggingLevel=self.loggingLevel)
        i=0
        currSpeed=minSpeed*speedRatio
        self.maxSpeed=maxSpeed*speedRatio
        self.zeroErrorSteering=zeroErrorSteering
        powerStep=currSpeed/abs(currSpeed)
        self.markDegrees=self.DistanceToDegrees(minDistance)
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        soDistance = abs(self.motorB.GetDegrees() + self.motorC.GetDegrees()) / 2
 

        # διάβασε την 1η είσοδο & υπολόγισε την διόρθωση
        if ports==1:
            input1=self.color1.GetLight(color=useColor)
            cor=pid.Step(inputRead=input1,target=target)*leftRight
        if ports==2:
            input2=self.color2.GetLight(color=useColor)
            cor=self.pid.Step(inputRead=input2,target=target)*leftRight
        elif ports==3:
            input3=self.color3.GetLight(color=useColor)
            cor=self.pid.Step(inputRead=input3,target=target)*leftRight
        elif ports==23:
            input2=self.color2.GetLight(color=useColor)
            input3=self.color3.GetLight(color=useColor)
            if target==0:
                cor=-self.pid.Step(inputRead=(input2-input3),target=target)
            else:
                cor= self.pid.Step(inputRead=input2,target=target)*input3/100*leftRight
        elif ports==32:
            input2=self.color2.GetLight(color=useColor)
            input3=self.color3.GetLight(color=useColor)
            if target==0:
                cor=-selfpid.Step(inputRead=(input2-input3),target=target)
            else:
                cor= self.pid.Step(inputRead=input3,target=target)*input2/100*leftRight
        elif ports==4:
            input1=self.color4.GetLight(color=useColor)
            cor=self.pid.Step(inputRead=input1,target=target)*leftRight
        if maxSpeed<0:
            cor=-cor    
            

        start=time.perf_counter()
        # Εδώ αρχίζει η επανάληψη
        while  True:            
            #t0=time.perf_counter()            
            i+=1     
            distB=self.motorB.GetDegrees()
            distC=self.motorC.GetDegrees()
            self.soDegrees = abs(distB + distC) / 2
            # Αν η απόσταση είναι μεγαλύτερη απο maxDegrees ολοκληρώνεται η ακολούθηση
            if self.soDegrees>=maxDegrees:
                break
            # Αν η απόσταση είναι μεγαλύτερη απο minDegrees υλοποιείται η λίστα με τα waits            
            if self.soDegrees>=minDegrees:
                # εκτελούνται τα waits. Aν δεν υπάρχει action στα waits ολοκληρώνουμε 
                if self.actionsDone():
                    break
                        
            if self.zeroErrorSteering==0:
                self.motorB.SetSpeed(speed=(currSpeed+cor))
                self.motorC.SetSpeed(speed=(currSpeed-cor))
            else:                
                zstr=self.Steering(self.zeroErrorSteering,currSpeed)
                self.motorB.SetSpeed(speed=(zstr[0]+cor))
                self.motorC.SetSpeed(speed=(zstr[1]-cor))

            # self.motorB.On(power=(currPower+cor),unregulated=True)
            # self.motorC.On(power=(currPower-cor),unregulated=True)
            # self.Print(currSpeed,self.maxSpeed)
            # if abs(currSpeed)<abs(self.maxSpeed):
            #     currSpeed += powerStep

            if abs(currSpeed)<abs(self.maxSpeed):
                if abs(self.pid.error)<=(100-target)/5:
                    currSpeed += powerStep
            if abs(currSpeed)>abs(self.maxSpeed):
                currSpeed -= powerStep*2.5

            
            if self.loggingLevel==1:
                self.speedsB[i%10]=currSpeed+cor
                self.speedsC[i%10]=currSpeed-cor

                st1=self.CalcSteering(powerLeft=sum(self.speedsB)/10,powerRight=sum(self.speedsC)/10)[1]
                # self.Print('real steering: ',st1)
                self.steeringList.pop(0)
                self.steeringListDistance.pop(0)
                self.steeringList.append(st1)
                self.steeringListDistance.append(self.soDegrees)
                if self.steeringList[1]>self.steeringList[0] and self.steeringList[1]>self.steeringList[2]:
                    self.Print('steering ', self.DegreesToDistance(self.soDegrees), st1)
                if self.steeringList[1]<self.steeringList[0] and self.steeringList[1]<self.steeringList[2]:
                    self.Print('steering ', self.DegreesToDistance(self.soDegrees), st1)
                    

            self.timer.Step() 

            # διάβασε νέα είσοδο      
            if ports==1:
                input1=self.color1.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input1,target=target)*leftRight
            if ports==2:
                input2=self.color2.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input2,target=target)*leftRight
            elif ports==3:
                input3=self.color3.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input3,target=target)*leftRight
            elif ports==23:
                input2=self.color2.GetLight(color=useColor)
                input3=self.color3.GetLight(color=useColor)
                if target==0:
                    cor=-self.pid.Step(inputRead=(input2-input3),target=target)
                else:
                    cor= self.pid.Step(inputRead=input2,target=target)*input3/100*leftRight
            elif ports==32:
                input2=self.color2.GetLight(color=useColor)
                input3=self.color3.GetLight(color=useColor)
                if target==0:
                    cor=-selfpid.Step(inputRead=(input2-input3),target=target)
                else:
                    cor= self.pid.Step(inputRead=input3,target=target)*input2/100*leftRight
            elif ports==4:
                input1=self.color4.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input1,target=target)*leftRight
            if maxSpeed<0:
                cor=-cor    
            

            if calcGyroAngleCorrection:
                self.gyro.GetAngle()

            # # Αν 
            # t1=time.perf_counter()
            # a=dt-t1+t0
            # if a>0:
            #     wait(a*1000)   
        totalTime=time.perf_counter()-start
        if stopAtEnd:
            self.Stop()            
        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        self.soDegrees = abs(distB + distC) / 2
        extraDistance= self.soDegrees - self.markDegrees
        self.extraDegrees=extraDistance
        if self.logger.level<=1:
            #if self.logger.level<=1:
            s1 = self.pid.GetStatistics()
            self.logger.Info(s1)
        s +="\n| dt="+str(totalTime/i)
        s += '\n| degrees/dt:'+str((distB+distC)/2/i)+' ('+str(self.DegreesToDistance(distB+distC)/2/i)+' mm/dt)'
        s += '\n| motorB:'+str(distB)+" ("+str(self.DegreesToDistance(distB))+" mm)"
        s += '\n| motorC:'+str(distC)+" ("+str(self.DegreesToDistance(distC))+" mm)"
        s += "\n| soDistance= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
        s += "\n| markDistance=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
        s += "\n| topSpeed= "+str((self.motorB.topSpeed+self.motorC.topSpeed)/2.0)
        s += "\n| topPower= "+str((self.motorB.topPower+self.motorC.topPower)/2.0)
        s += "\n| extraDistance= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
        s += "\n=========================================================="
        
        if self.logger.level<=2:    
            self.logger.Debug(s)
        s2 ="\nFollowLine (Real) dt="+str(totalTime/i)
        self.Print(s2)
        
        # self.logger.Save()
        
        self.lastMessage=s
        # self.Print(s)
        if calcGyroAngleCorrection:
            self.gyro.values.sort()
            target=round(self.gyro.value/90)*90
            l=len(self.gyro.values)
            self.Print('angle before correction',self.gyro.GetAngle())            
            self.gyro.angleCorrection=self.gyro.angleCorrection+target-self.gyro.values[l//2]
            self.Print('angle after correction',self.gyro.GetAngle())
            self.Print('angle values=',self.gyro.values)
            self.Print('angle correction=',self.gyro.angleCorrection)

        return(self.extraDegrees)


    def FollowLineToColor(self, ports=2, target=50, leftRight=-1, kp=0.7, ki=0, kd=0.3, dt=0.01, minDistance=0, maxDistance=9999999, minSpeed=10, maxSpeed=25,speedRatio=1, waits=[], colorsToStop=[2,3,4,5,7],comments="", useColor='white', stop_action='brake', stopAtEnd=True,centerLine=False,zeroErrorSteering=0):
        """
        
        """
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s+="\n| FollowLineToColor(ports="+str(ports)+",target="+str(target)+",leftRight="+str(leftRight)+",kp="+str(kp)
        s +=",ki="+str(ki)+",kd="+str(kd)+",dt="+str(dt)+",minDistance="+str(minDistance)+", maxDistance="+str(maxDistance)
        s +=",minSpeed=" + str(minSpeed*speedRatio) + ",maxSpeed=" + str(maxSpeed*speedRatio) + ", waits=" + self.ListToStr(waits) + ",stop_action='" + stop_action + "')"
            
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees=self.DistanceToDegrees(maxDistance)    
        self.timer.dt=dt
        self.action=None
        self.waits=waits
        if self.loggingLevel==1:
            # Αρχικοποίηση στατιστικών
            self.speedsB=[]
            self.speedsC=[]
            self.steeringList=[0,0,0]
            self.steeringListDistance=[0,0,0]
            for i in range(10):
                self.speedsB.append(0)
                self.speedsC.append(0)

        self.pid=myPID(name=self.name+'.FollowLine (pid)',kp=kp,ki=ki,kd=kd,loggingStats=True,loggingLevel=self.loggingLevel)
        i=0
        currSpeed=minSpeed*speedRatio
        self.maxSpeed=maxSpeed*speedRatio
        self.zeroErrorSteering=zeroErrorSteering
        powerStep=currSpeed/abs(currSpeed)
        self.markDegrees=self.DistanceToDegrees(minDistance)
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        soDistance = abs(self.motorB.GetDegrees() + self.motorC.GetDegrees()) / 2
 

        # διάβασε την 1η είσοδο & υπολόγισε την διόρθωση
        if ports==1:
            input1=self.color1.GetLight(color=useColor)
            cor=pid.Step(inputRead=input1,target=target)*leftRight
        if ports==2:
            input2=self.color2.GetLight(color=useColor)
            cor=self.pid.Step(inputRead=input2,target=target)*leftRight
        elif ports==3:
            input3=self.color3.GetLight(color=useColor)
            cor=self.pid.Step(inputRead=input3,target=target)*leftRight
        elif ports==23:
            input2=self.color2.GetLight(color=useColor)
            input3=self.color3.GetLight(color=useColor)
            if target==0:
                cor=-self.pid.Step(inputRead=(input2-input3),target=target)
            else:
                cor= self.pid.Step(inputRead=input2,target=target)*input3/100*leftRight
        elif ports==32:
            input2=self.color2.GetLight(color=useColor)
            input3=self.color3.GetLight(color=useColor)
            if target==0:
                cor=-selfpid.Step(inputRead=(input2-input3),target=target)
            else:
                cor= self.pid.Step(inputRead=input3,target=target)*input2/100*leftRight
        elif ports==4:
            input1=self.color4.GetLight(color=useColor)
            cor=self.pid.Step(inputRead=input1,target=target)*leftRight
        if maxSpeed<0:
            cor=-cor    
            

        start=time.perf_counter()
        detectedColor=0
        # Εδώ αρχίζει η επανάληψη
        while  True:            
            #t0=time.perf_counter()            
            i+=1     
            distB=self.motorB.GetDegrees()
            distC=self.motorC.GetDegrees()
            self.soDegrees = abs(distB + distC) / 2
            # # Αν η απόσταση είναι μεγαλύτερη απο maxDegrees ολοκληρώνεται η ακολούθηση
            # if self.soDegrees>=maxDegrees:
            #     break
            # # Αν η απόσταση είναι μεγαλύτερη απο minDegrees υλοποιείται η λίστα με τα waits            
            # # if self.soDegrees>=minDegrees:
            # #     # εκτελούνται τα waits. Aν δεν υπάρχει action στα waits ολοκληρώνουμε 
            # #     if self.actionsDone():
            # #         break
            self.motorB.SetSpeed(speed=(currSpeed+cor))
            self.motorC.SetSpeed(speed=(currSpeed-cor))            


            if abs(currSpeed)<abs(self.maxSpeed):
                if abs(self.pid.error)<=(100-target)/5:
                    currSpeed += powerStep
            if abs(currSpeed)>abs(self.maxSpeed):
                currSpeed -= powerStep*2.5

            self.timer.Step() 

            # διάβασε νέα είσοδο      
            if ports==1:
                input1=self.color1.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input1,target=target)*leftRight
            if ports==2:
                if self.soDegrees>=minDegrees:
                    detectedColor=self.color3.GetColor(method=3)
                    if detectedColor in colorsToStop:
                        self.Stop()
                        break
                input2=self.color2.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input2,target=target)*leftRight
            elif ports==3:
                if self.soDegrees>=minDegrees:
                    detectedColor=self.color2.GetColor(method=3)
                    if detectedColor in colorsToStop:
                        self.Stop()
                        break
                input3=self.color3.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input3,target=target)*leftRight
            elif ports==23:
                if self.soDegrees<minDegrees:
                    input2=self.color2.GetLight(color=useColor)
                    input3=self.color3.GetLight(color=useColor)
                elif i % 2 == 1:
                    detectedColor=self.color3.GetColor(method=3)
                    if detectedColor in colorsToStop:
                        # self.Print("detected:",detectedColor)
                        self.Stop()
                        break
                    input2=self.color2.GetLight(color=useColor)
                else:
                    detectedColor=self.color2.GetColor(method=3)
                    if detectedColor in colorsToStop:
                        # self.Print("detected:",detectedColor)
                        self.Stop()
                        break
                    input3=self.color3.GetLight(color=useColor)

                if target==0:
                    cor=-self.pid.Step(inputRead=(input2-input3),target=target)
                else:
                    cor= self.pid.Step(inputRead=input2,target=target)*input3/100*leftRight
            elif ports==32:
                input2=self.color2.GetLight(color=useColor)
                input3=self.color3.GetLight(color=useColor)
                if target==0:
                    cor=-selfpid.Step(inputRead=(input2-input3),target=target)
                else:
                    cor= self.pid.Step(inputRead=input3,target=target)*input2/100*leftRight
            elif ports==4:
                input1=self.color4.GetLight(color=useColor)
                cor=self.pid.Step(inputRead=input1,target=target)*leftRight
            if maxSpeed<0:
                cor=-cor    
            



            # # Αν 
            # t1=time.perf_counter()
            # a=dt-t1+t0
            # if a>0:
            #     wait(a*1000)   
        totalTime=time.perf_counter()-start
        if stopAtEnd:
            self.Stop()            
        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        self.soDegrees = abs(distB + distC) / 2
        extraDistance= self.soDegrees - self.markDegrees
        self.extraDegrees=extraDistance
        if self.logger.level<=1:
            #if self.logger.level<=1:
            s1 = self.pid.GetStatistics()
            self.logger.Info(s1)
        s +="\n| dt="+str(totalTime/i)
        s += '\n| degrees/dt:'+str((distB+distC)/2/i)+' ('+str(self.DegreesToDistance(distB+distC)/2/i)+' mm/dt)'
        s += '\n| motorB:'+str(distB)+" ("+str(self.DegreesToDistance(distB))+" mm)"
        s += '\n| motorC:'+str(distC)+" ("+str(self.DegreesToDistance(distC))+" mm)"
        s += "\n| soDistance= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
        s += "\n| markDistance=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
        s += "\n| topSpeed= "+str((self.motorB.topSpeed+self.motorC.topSpeed)/2.0)
        s += "\n| topPower= "+str((self.motorB.topPower+self.motorC.topPower)/2.0)
        s += "\n| extraDistance= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
        s += "\n=========================================================="
        
        if self.logger.level<=2:    
            self.logger.Debug(s)
        # self.logger.Save()
        self.lastMessage=s
        self.Print('Color detected to stop:',detectedColor)
        s2 ="\nFollowLineToColor (Real) dt="+str(totalTime/i)
        self.Print(s2)

        return(self.extraDegrees)


    def CenterLine(self, kp=0.3, kd=1, dt=0.01,acceptedError=10):
        """
      examples:
      r.CenterLine(kp=0.3,kd=1,acceptedError=20) (for large motors and nxt)
      r.CenterLine(kp=0.4,kd=1,acceptedError=30) (for medium motors)
      r.FollowLine(ports=23,target=0,kp=0.1,kd=1.5,minSpeed=30,maxSpeed=80,minDistance=800,waits=['speed=20','black=2'])
 
        """
        self.timer.dt=dt
        self.pid=myPID(name=self.name+'.CenterLine (pid)',kp=kp,ki=0,kd=kd,loggingStats=True,loggingLevel=self.loggingLevel)
        i=0
        # διάβασε την είσοδο & υπολόγισε την διόρθωση
        input2=self.color2.GetLight()
        input3=self.color3.GetLight()
        cor=-self.pid.Step(inputRead=(input2-input3),target=0)

        start=time.perf_counter()
        # Εδώ αρχίζει η επανάληψη
        while  abs(input2-input3)>acceptedError:            
            #t0=time.perf_counter()            
            i+=1     
            self.motorB.SetSpeed(speed=cor)
            self.motorC.SetSpeed(speed=-cor)
            self.timer.Step() 

            # διάβασε νέα είσοδο      
            input2=self.color2.GetLight()
            input3=self.color3.GetLight()
            cor=-self.pid.Step(inputRead=(input2-input3),target=0)
        self.Stop()            
        totalTime=time.perf_counter()-start
        self.Print('Center Line Total steps: ',i,'in Time: ',totalTime)
        if self.logger.level<=1:
            #if self.logger.level<=1:
            s1 = self.pid.GetStatistics()
            self.logger.Info(s1)

    def Turn(self, steering=100, speed=40, angle=90):
        """[summary]

        Args:
            steering (int, optional): [description]. Defaults to 100.
            speed (int, optional): [description]. Defaults to 40.
            angle (int, optional): [description]. Defaults to 90.
        """
        self.lastAngle=self.lastAngle + angle*steering/abs(steering)
        if (abs(steering)+25) // 50<=1:
            m=self.axleTrack50
            # self.Print('steering close to 50')
            # self.logger.Warning('steering close to 50')
        else:
            m=self.axleTrack100
            # self.Print('steering close to 100')
            # self.logger.Warning('steering close to 100')
        # self.logger.Warning('turn-   steering '+str(steering)+' axletrack '+str(m)+' axlt50:'+str(self.axleTrack50)+' axltr100:'+str(self.axleTrack100))
        d=self.wheelDiameter
        out=self.Steering(steering, speed)
        #print("powers=",out)
        if max(out)!=0:
            k=min(out)/max(out)
        else:
            k=0
        #print("k=",k)
        a=k/(k-1)*m
        if steering*speed>0:
            r2=abs(a)
            r1=abs(m-a)
        else:
            r1=abs(a)
            r2=abs(m-a)
    
        #print("r_left,r_right)=",(r1,r2))
        p1=2*3.14159*r1*abs(angle)/(d*3.14159)
        p2=2*3.14159*r2*abs(angle)/(d*3.14159)
        #print("c_left,c_right)=",(p1,p2))
        # self.Print("angle Left:"+str(p1)+" angle right"+str(p2))
        sall=max(p1,p2)
        s1=min(sall/2,150)
        s3=s1
        s2=sall-s1-s3
        self.logger.Debug("s1:"+str(s1)+" s2:"+str(s2)+" s3:"+str(s3))
        self.MoveSteering3(steering=steering, power=[speed / 10, speed, speed / 10], s1=s1, s2=s2, s3=s3)

    def SetAngle(self, targetAngle=20.0,maxAngle=90,minSpeed=10,maxSpeed=40,movingSpeed=0,steering=100,stopAtEnd=True):
        currentAngle=self.gyro.GetAngle()
        finalAngle=targetAngle
                
        error=(finalAngle-currentAngle)
        if error==0:
            self.power=0
        else:
            y=error/maxAngle*90*math.pi/180
            sy=math.sin(y)
            v=sy*(maxSpeed-minSpeed)
            if sy!=0:
                v=v+sy/abs(sy)*minSpeed
            self.power=v    

        
        if steering ==50:
            self.motorB.SetSpeed(movingSpeed+self.power)  
        elif steering==-50:
            self.motorC.SetSpeed(movingSpeed +self.power)  
        elif steering==-100:
            self.motorB.SetSpeed(movingSpeed-self.power)
            self.motorC.SetSpeed(movingSpeed +self.power)
        else:
            self.motorB.SetSpeed(movingSpeed+self.power)
            self.motorC.SetSpeed(movingSpeed-self.power)

        return(error)
  
    def GyroTurn(self, targetAngle=90,minSpeed=5, maxSpeed=30,autoCorrectAngle=90,acceptedError=5,comments="",steering=100):
        """
        Η βασική μέθοδος περιστροφής σε γωνία με γυροσκόπειο
        
        Parameters:
            port: Ή πόρτα που είναι συνδεμένο το Γυροσκόπειο που χρησιμοποιείται στην ακολούθηση
            target: Η τιμή της γωνίας που δεν χρειάζεται διόρθωση της πορείας
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
            minDegrees: Η ελάχιστη απόσταση που θα διαρκέσει η διαδικασία ακολούθησης της γραμμής
            maxDegrees: Η μέγιστη απόσταση ακολούθησης της γραμμής
            minSpeed: Η αρχική ισχύς των μοτέρ κατα την εκκίνηση της διαδικασίας ακολούθησης
            maxSpeed: Η μέγιστη ισχύς των μοτέρ κατα την διαδικασία ακολούθησης
            stop_action: Ο τρόπος με τον οποίο σταματάνε τα μοτέρ στο τέλος της διαδικασίας ('brake','coast','hold')
            stopAtEnd: True: σβήσιμο των μοτέρ στο τέλος της διαδικασίας, False: τα μοτέρ παραμένουν ανοιχτά (περίπτωση που ακολουθεί νέα ακολούθηση)
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
        return:
            Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        
        """
        if not self.gyro.initialized:
            self.logger.Error(log="Gyro Sensor not Initialized")
        s = "\n=================================================================="
        currentAngle=self.gyro.GetAngle()
        
        finalAngle=round(currentAngle / autoCorrectAngle)*autoCorrectAngle+targetAngle
        # self.Print("Gyro Turn (fromAngle,target) Angle: (",currentAngle,",",finalAngle,")")
        sign=1
        
        if finalAngle<currentAngle:
            sign=-1
        while  True:            
            x=self.SetAngle(targetAngle=finalAngle,maxAngle=90*sign,minSpeed=minSpeed,maxSpeed=maxSpeed,steering=steering)
            if abs(x)<=acceptedError:
                self.Stop()
                break            
        self.Stop()
        
        # self.Print("Gyro Turn (fromAngle,target,final) Angle: (",currentAngle,",",finalAngle,",",self.gyro.GetAngle(),")")
        

    def TurnToHour(self, hour=9, steering=-100, minSpeed=20, maxSpeed=30):
        """[summary]

        Args:
            hour (int, optional): [description]. Defaults to 9.
            steering (int, optional): [description]. Defaults to -100.
            minSpeed (int, optional): [description]. Defaults to 15.
            maxSpeed (int, optional): [description]. Defaults to 50.
        """
        if steering in (-100,100):
            totalDegreesToGo=self.DistanceToDegrees(self.wheelsToSensorsdistance - self.extraDegrees)
            totalDistanceToGo=self.wheelsToSensorsdistance-self.extraDegrees
            
        elif steering in (50,-50):
            #totalDistanceToGo=self.DistanceToDegrees(-self.axleTrack/2+self.wheelsToSensorsdistance+10-self.extraDistance)
            totalDistanceToGo = self.DistanceToDegrees(-self.axleTrack50 / 2 + self.wheelsToSensorsdistance - self.extraDegrees)
        if hour in (2,10):
            #20 is line width
            totalDistanceToGo=totalDistanceToGo+self.DistanceToDegrees(self.sensorToSensorDistance / 2)+20
        if hour in (4,8):
            #20 is line width
            totalDistanceToGo=totalDistanceToGo-self.DistanceToDegrees(self.sensorToSensorDistance / 2)-20
        
        if hour in (9,-3,3,2,4,8,10):
            if self.extraDegrees+totalDistanceToGo>0:
                self.MoveSteering2(steering=0,power=[minSpeed,minSpeed,minSpeed],s2=abs(totalDistanceToGo))
                # self.MoveTank(speedB=maxSpeed, speedC=maxSpeed,minDistance=self.DegreesToDistance(totalDistanceToGo),maxDistance=self.DegreesToDistance(totalDistanceToGo))
            elif self.extraDegrees+totalDistanceToGo<0:
                self.MoveSteering2(steering=0,power=[-minSpeed,-minSpeed,-minSpeed],s2=abs(totalDistanceToGo))
                # self.MoveTank(speedB=maxSpeed, speedC=-maxSpeed, degrees=totalDistanceToGo)
            self.Turn(steering=steering, speed=maxSpeed, angle=90)
            self.Print("total distace: "+str(totalDistanceToGo))

    def FollowAngle(self, angle=0, kp=1, ki=0.0, kd=0, dt=0.01, minDistance=0, maxDistance=9999999, minSpeed=20, maxSpeed=50, waits=[], stop_action='brake', stopAtEnd=True, comments='',accelerationTime=0,autoCorrectAngle=90):
        """
        Η βασική μέθοδος ακολούθησης μια πορείας με γυροσκόπειο
        
        Parameters:
            port: Ή πόρτα που είναι συνδεμένο το Γυροσκόπειο που χρησιμοποιείται στην ακολούθηση
            target: Η τιμή της γωνίας που δεν χρειάζεται διόρθωση της πορείας
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
            minDegrees: Η ελάχιστη απόσταση που θα διαρκέσει η διαδικασία ακολούθησης της γραμμής
            maxDegrees: Η μέγιστη απόσταση ακολούθησης της γραμμής
            minSpeed: Η αρχική ισχύς των μοτέρ κατα την εκκίνηση της διαδικασίας ακολούθησης
            maxSpeed: Η μέγιστη ισχύς των μοτέρ κατα την διαδικασία ακολούθησης
            stop_action: Ο τρόπος με τον οποίο σταματάνε τα μοτέρ στο τέλος της διαδικασίας ('brake','coast','hold')
            stopAtEnd: True: σβήσιμο των μοτέρ στο τέλος της διαδικασίας, False: τα μοτέρ παραμένουν ανοιχτά (περίπτωση που ακολουθεί νέα ακολούθηση)
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
        return:
            Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        
        """
        currentAngle=self.gyro.GetAngle()
        target=round(currentAngle / autoCorrectAngle)*autoCorrectAngle+angle
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s +="\nFollowAngle(target="+str(target)+",kp="+str(kp)
        s +=",ki="+str(ki)+",kd="+str(kd)+",dt="+str(dt)+",minDistance="+str(minDistance)+", maxDegrees="+str(maxDistance)
        s +=",minspeed=" + str(minSpeed) + ",maxPower=" + str(maxSpeed) + ", waits=" + self.ListToStr(waits) + ",stop_action='" + stop_action + "')"
        self.gyro.timer.dt=dt
        self.maxSpeed=maxSpeed
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees=self.DistanceToDegrees(maxDistance)    
        
        self.gyro.pid=myPID(name=self.name+'.FollowAngle (pid)',kp=kp,ki=ki,kd=kd,loggingStats=True,direction=1)
        
        i=0
        # action=None
        self.action=None
        self.waits=waits
        currPower=minSpeed
        powerStep=currPower/abs(currPower)
        self.markDegrees=minDegrees
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        # διαβάζεις νέα είσοδο. 
        input=self.gyro.GetAngle()
        # Υπολογίζεις διόρθωση
        # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
        target1=target
        cor=self.gyro.pid.Step(inputRead=input,target=target)
        
        self.timer.Start()
        start=time.perf_counter()
        while  True:            
            i+=1
            # Ευρεση απόστασης
            distB=self.motorB.GetDegrees()
            distC=self.motorC.GetDegrees()
            self.soDegrees = abs(distB + distC) / 2
            # έλεγχος για υπέρβαση μέγιστης απόστασης
            if self.soDegrees>=maxDistance:
                break
            # αν έχει υλοποηθεί η ελάχιστη απόσταση ελέγχονται τα waits
            if self.soDegrees>=minDegrees:
                # εκτελούνται τα waits. Aν δεν υπάρχει action στα waits ολοκληρώνουμε 
                if self.actionsDone():
                    break
                        
            # self.Print("target=",target," input=",input," corr=",cor)
            # time.sleep(1)
            self.motorB.SetSpeed(speed=(currPower+cor),accelerationTime=accelerationTime)
            self.motorC.SetSpeed(speed=(currPower-cor),accelerationTime=accelerationTime)
            if abs(currPower)<abs(self.maxSpeed):
                currPower += powerStep
            if abs(currPower)>abs(self.maxSpeed):
                currPower -= powerStep*2.5
            # Περίμενε τουλάχιστον για χρόνο dt
            self.timer.Step()
            # διαβάζεις νέα είσοδο. 
            input=self.gyro.GetAngle()
            # Υπολογίζεις διόρθωση
            # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
            # self.Print('input:',input,'target=',target,'lastangle+target=',self.lastAngle+target,'cor=',cor)
            cor=self.gyro.pid.Step(inputRead=input,target=target)
            if self.gyro.logger.level<2:
                self.Print('input:',input,'target=',target,'cor=',cor,' b=',currPower+cor,' c=',currPower-cor)
    
        if stopAtEnd:
            self.Stop()
        totalTime=time.perf_counter()-start
       
        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        self.soDegrees = abs(distB + distC) / 2
        self.extraDegrees= self.soDegrees - self.markDegrees
                
        if self.logger.level<=2:
            s += self.gyro.pid.GetStatistics()        
            s +="\n| dt (real)="+str(totalTime/i)
            s += '\n| degrees/dt:'+str((distB+distC)/2/i)+' ('+str(self.DegreesToDistance(distB+distC)/2/i)+' mm/dt)'
            s += '\n| Gyro Drift (degrees/sec):'+str(self.gyro.driftError)
            s += '\n| motorB:'+str(distB)+" ("+str(self.DegreesToDistance(distB))+" mm)"
            s += '\n| motorC:'+str(distC)+" ("+str(self.DegreesToDistance(distC))+" mm)"
            s += "\n| topSpeed= "+str((abs(self.motorB.topSpeed)+abs(self.motorC.topSpeed))/2.0)
            s += "\n| topPower= "+str((abs(self.motorB.topPower)+abs(self.motorC.topPower))/2.0)
            s += "\n| soDistance= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
            s += "\n| markDistance=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
            s += "\n| extraDistance= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
            s += "\n=========================================================="
            self.logger.Debug(s)
        s2 ="\n| FollowAngle dt (real)="+str(totalTime/i)
        self.Print(s2)
        # self.logger.Save()
        return(self.extraDegrees)
 
    def FollowAngleToLine(self, angle=0, kp=1, ki=0.0, kd=0, dt=0.01, minDistance=0, maxDistance=9999999, minSpeed=20, maxSpeed=50, stop_action='brake',stopAtLineSensor=2, stopAtEnd=True, comments='',accelerationTime=0,autoCorrectAngle=90):
        """
        Η βασική μέθοδος ακολούθησης μια πορείας με γυροσκόπειο
        
        Parameters:
            port: Ή πόρτα που είναι συνδεμένο το Γυροσκόπειο που χρησιμοποιείται στην ακολούθηση
            target: Η τιμή της γωνίας που δεν χρειάζεται διόρθωση της πορείας
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
            minDegrees: Η ελάχιστη απόσταση που θα διαρκέσει η διαδικασία ακολούθησης της γραμμής
            maxDegrees: Η μέγιστη απόσταση ακολούθησης της γραμμής
            minSpeed: Η αρχική ισχύς των μοτέρ κατα την εκκίνηση της διαδικασίας ακολούθησης
            maxSpeed: Η μέγιστη ισχύς των μοτέρ κατα την διαδικασία ακολούθησης
            stop_action: Ο τρόπος με τον οποίο σταματάνε τα μοτέρ στο τέλος της διαδικασίας ('brake','coast','hold')
            stopAtEnd: True: σβήσιμο των μοτέρ στο τέλος της διαδικασίας, False: τα μοτέρ παραμένουν ανοιχτά (περίπτωση που ακολουθεί νέα ακολούθηση)
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
        return:
            Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        
        """
        currentAngle=self.gyro.GetAngle()
        target=round(currentAngle / autoCorrectAngle)*autoCorrectAngle+angle
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s +="\nFollowAngle(target="+str(target)+",kp="+str(kp)
        s +=",ki="+str(ki)+",kd="+str(kd)+",dt="+str(dt)+",minDistance="+str(minDistance)+", maxDegrees="+str(maxDistance)
        s +=",minspeed=" + str(minSpeed) + ",maxPower=" + str(maxSpeed) +  ",stop_action='" + stop_action + "')"
        self.gyro.timer.dt=dt
        self.maxSpeed=maxSpeed
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees=self.DistanceToDegrees(maxDistance)    
        
        self.gyro.pid=myPID(name=self.name+'.FollowAngle (pid)',kp=kp,ki=ki,kd=kd,loggingStats=True,direction=1)
        
        i=0
        # action=None
        self.action=None
        currPower=minSpeed
        powerStep=currPower/abs(currPower)
        self.markDegrees=minDegrees
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        # διαβάζεις νέα είσοδο. 
        input=self.gyro.GetAngle()
        # Υπολογίζεις διόρθωση
        # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
        target1=target
        cor=self.gyro.pid.Step(inputRead=input,target=target)
        
        self.timer.Start()
        start=time.perf_counter()
        iswhite=False
        while  True:            
            i+=1
            # Ευρεση απόστασης
            distB=self.motorB.GetDegrees()
            distC=self.motorC.GetDegrees()
            self.soDegrees = abs(distB + distC) / 2
            # έλεγχος για υπέρβαση μέγιστης απόστασης
            if self.soDegrees>=maxDistance:
                break
            # αν έχει υλοποηθεί η ελάχιστη απόσταση ελέγχονται τα waits
            if self.soDegrees>=minDegrees:
                if stopAtLineSensor==2:
                    sensor=self.color2
                elif stopAtLineSensor==3:
                    sensor=self.color3
                elif stopAtLineSensor==1:
                    sensor=self.color1
                else:
                    sensor=self.color4
                colorx=sensor.GetColor(method=3)
                if colorx==6:
                    iswhite=True
                if iswhite:
                    if colorx==1:
                        self.Stop()
                        break
                        
            # self.Print("target=",target," input=",input," corr=",cor)
            # time.sleep(1)
            self.motorB.SetSpeed(speed=(currPower+cor),accelerationTime=accelerationTime)
            self.motorC.SetSpeed(speed=(currPower-cor),accelerationTime=accelerationTime)
            if abs(currPower)<abs(self.maxSpeed):
                currPower += powerStep
            if abs(currPower)>abs(self.maxSpeed):
                currPower -= powerStep*2.5
            # Περίμενε τουλάχιστον για χρόνο dt
            self.timer.Step()
            # διαβάζεις νέα είσοδο. 
            input=self.gyro.GetAngle()
            # Υπολογίζεις διόρθωση
            # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
            # self.Print('input:',input,'target=',target,'lastangle+target=',self.lastAngle+target,'cor=',cor)
            cor=self.gyro.pid.Step(inputRead=input,target=target)
            if self.gyro.logger.level<2:
                self.Print('input:',input,'target=',target,'cor=',cor,' b=',currPower+cor,' c=',currPower-cor)
    
        if stopAtEnd:
            self.Stop()
        totalTime=time.perf_counter()-start

        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        self.soDegrees = abs(distB + distC) / 2
        self.extraDegrees= self.soDegrees - self.markDegrees
                
        if self.logger.level<=2:
            s += self.gyro.pid.GetStatistics()        
            s +="\n| dt (real)="+str(totalTime/i)
            s += '\n| degrees/dt:'+str((distB+distC)/2/i)+' ('+str(self.DegreesToDistance(distB+distC)/2/i)+' mm/dt)'
            s += '\n| Gyro Drift (degrees/sec):'+str(self.gyro.driftError)
            s += '\n| motorB:'+str(distB)+" ("+str(self.DegreesToDistance(distB))+" mm)"
            s += '\n| motorC:'+str(distC)+" ("+str(self.DegreesToDistance(distC))+" mm)"
            s += "\n| topSpeed= "+str((abs(self.motorB.topSpeed)+abs(self.motorC.topSpeed))/2.0)
            s += "\n| topPower= "+str((abs(self.motorB.topPower)+abs(self.motorC.topPower))/2.0)
            s += "\n| soDistance= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
            s += "\n| markDistance=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
            s += "\n| extraDistance= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
            s += "\n=========================================================="
            self.logger.Debug(s)
            #self.logger.Save()
        return(self.extraDegrees)
 

    def FollowWhiteLane(self, leftRight=-1,angle=0, kp=1,kp2=0.7, ki=0.0, kd=0, dt=0.01, minDistance=0, maxDistance=9999999, minSpeed=20, maxSpeed=50,stopAtLineSensor=2,distanceAfterLine=0,  stop_action='brake', stopAtEnd=True, comments='',accelerationTime=0,autoCorrectAngle=90):
        """
        Η βασική μέθοδος ακολούθησης μια πορείας με γυροσκόπειο
        
        Parameters:
            port: Ή πόρτα που είναι συνδεμένο το Γυροσκόπειο που χρησιμοποιείται στην ακολούθηση
            target: Η τιμή της γωνίας που δεν χρειάζεται διόρθωση της πορείας
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
            minDegrees: Η ελάχιστη απόσταση που θα διαρκέσει η διαδικασία ακολούθησης της γραμμής
            maxDegrees: Η μέγιστη απόσταση ακολούθησης της γραμμής
            minSpeed: Η αρχική ισχύς των μοτέρ κατα την εκκίνηση της διαδικασίας ακολούθησης
            maxSpeed: Η μέγιστη ισχύς των μοτέρ κατα την διαδικασία ακολούθησης
            stop_action: Ο τρόπος με τον οποίο σταματάνε τα μοτέρ στο τέλος της διαδικασίας ('brake','coast','hold')
            stopAtEnd: True: σβήσιμο των μοτέρ στο τέλος της διαδικασίας, False: τα μοτέρ παραμένουν ανοιχτά (περίπτωση που ακολουθεί νέα ακολούθηση)
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
        return:
            Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        
        """
        sign=1
        if maxSpeed<0:
            sign=-1
        currentAngle=self.gyro.GetAngle()
        target=round(currentAngle / autoCorrectAngle)*autoCorrectAngle+angle
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s +="\nFollowWhiteLane="+str(target)+",kp="+str(kp)
        s +=",ki="+str(ki)+",kd="+str(kd)+",dt="+str(dt)+",minDistance="+str(minDistance)+", maxDegrees="+str(maxDistance)
        s +=",minspeed=" + str(minSpeed) + ",maxPower=" + str(maxSpeed) +  ",stop_action='" + stop_action + "')"
        self.gyro.timer.dt=dt
        self.maxSpeed=maxSpeed
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees=self.DistanceToDegrees(maxDistance)    
        
        self.gyro.pid=myPID(name=self.name+'.FollowAngle (pid)',kp=kp,ki=ki,kd=kd,loggingStats=True,direction=1)
        
        i=0
        # action=None
        self.action=None
        currPower=minSpeed
        powerStep=currPower/abs(currPower)
        self.markDegrees=minDegrees
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        # διαβάζεις νέα είσοδο. 
        input=self.gyro.GetAngle()
        # Υπολογίζεις διόρθωση
        # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
        target1=target
        cor=self.gyro.pid.Step(inputRead=input,target=target)
        
        self.timer.Start()
        start=time.perf_counter()
        c2=6
        c3=6
        isLine=False
        
        while  True:            
            i+=1
            # Ευρεση απόστασης
            distB=self.motorB.GetDegrees()
            distC=self.motorC.GetDegrees()
            self.soDegrees = abs(distB + distC) / 2
            # έλεγχος για υπέρβαση μέγιστης απόστασης
            if self.soDegrees>=maxDistance:
                break
            # αν έχει υλοποηθεί η ελάχιστη απόσταση ελέγχονται τα waits
            if self.soDegrees>=minDegrees:
                # εκτελούνται τα waits. Aν δεν υπάρχει action στα waits ολοκληρώνουμε 
                if not isLine:
                    if stopAtLineSensor==2 and c2==1:
                        self.markDegrees=abs(self.motorB.GetDegrees())*0.5+abs(self.motorC.GetDegrees())*0.5                
                        isLine=True
                        # self.Stop()
                        # break
                    elif stopAtLineSensor==3 and c3==1:
                        self.markDegrees=abs(self.motorB.GetDegrees())*0.5+abs(self.motorC.GetDegrees())*0.5                
                        # self.Stop()
                        isLine=True
                        # break
                if isLine:
                    curDegrees=abs(self.motorB.GetDegrees())*0.5+abs(self.motorC.GetDegrees())*0.5                
                    if curDegrees-self.markDegrees>=distanceAfterLine:
                        self.Stop()
                        break


            # self.Print("target=",target," input=",input," corr=",cor)
            # time.sleep(1)
            self.motorB.SetSpeed(speed=(currPower+cor),accelerationTime=accelerationTime)
            self.motorC.SetSpeed(speed=(currPower-cor),accelerationTime=accelerationTime)
            if abs(currPower)<abs(self.maxSpeed):
                currPower += powerStep
            if abs(currPower)>abs(self.maxSpeed):
                currPower -= powerStep*2.5
            # Περίμενε τουλάχιστον για χρόνο dt
            self.timer.Step()
            # διαβάζεις νέα είσοδο. 
            input=self.gyro.GetAngle()
            # Υπολογίζεις διόρθωση
            # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
            # self.Print('input:',input,'target=',target,'lastangle+target=',self.lastAngle+target,'cor=',cor)
            cor=self.gyro.pid.Step(inputRead=input,target=target)
            cor2=0
            if not isLine:
                c2=self.color2.GetColor(method=3)
                c3=self.color3.GetColor(method=3)
            else:
                c2=6
                c3=6
            if leftRight==-1:
                if c3==1 and c2!=1:
                    cor2=cor2-15*kp2
                elif c2==1 and c3!=1:
                    cor2=cor2-30*kp2
                elif  c3==1 and c2==1:
                    cor2=cor2
                elif c2==6 and c3==6:
                    cor2=cor2
                elif c2 in (2,3) :
                    cor2=cor2+10*kp2
                
            else:
                if c2==1 and c3!=1:
                    cor2=cor2+15*kp2
                elif c3==1 and c2!=1:
                    cor2=cor2+30*kp2
                elif  c3==1 and c2==1:
                    cor2=cor2
                elif c2==6 and c3==6:
                    cor2=cor2
                elif c3 in (2,3) :
                    cor2=cor2-10*kp2

            
            cor=cor+cor2*sign

            if self.gyro.logger.level<2:
                self.Print('input:',input,'target=',target,'cor=',cor,' b=',currPower+cor,' c=',currPower-cor)
    
        totalTime=time.perf_counter()-start
        if stopAtEnd:
            self.Stop()

        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        self.soDegrees = abs(distB + distC) / 2
        self.extraDegrees= self.soDegrees - self.markDegrees
                
        if self.logger.level<=2:
            s += self.gyro.pid.GetStatistics()        
            s +="\n| dt (real)="+str(totalTime/i)
            s += '\n| degrees/dt:'+str((distB+distC)/2/i)+' ('+str(self.DegreesToDistance(distB+distC)/2/i)+' mm/dt)'
            s += '\n| Gyro Drift (degrees/sec):'+str(self.gyro.driftError)
            s += '\n| motorB:'+str(distB)+" ("+str(self.DegreesToDistance(distB))+" mm)"
            s += '\n| motorC:'+str(distC)+" ("+str(self.DegreesToDistance(distC))+" mm)"
            s += "\n| topSpeed= "+str((abs(self.motorB.topSpeed)+abs(self.motorC.topSpeed))/2.0)
            s += "\n| topPower= "+str((abs(self.motorB.topPower)+abs(self.motorC.topPower))/2.0)
            s += "\n| soDistance= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
            s += "\n| markDistance=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
            s += "\n| extraDistance= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
            s += "\n=========================================================="
            self.logger.Debug(s)
            #self.logger.Save()
        self.Print("FollowWhiteLane dt (real)=",str(totalTime/i))
        return(self.extraDegrees)
 


    def FollowWhiteLane2(self, angle=0, kp=1, ki=0.0, kd=0, dt=0.01, minDistance=0, maxDistance=9999999, minSpeed=20, maxSpeed=50, stop_action='brake',stopAtLineSensor=2, stopAtEnd=True, comments='',accelerationTime=0,autoCorrectAngle=90):
        """
        ...............
        
        Parameters:
            port: Ή πόρτα που είναι συνδεμένο το Γυροσκόπειο που χρησιμοποιείται στην ακολούθηση
            target: Η τιμή της γωνίας που δεν χρειάζεται διόρθωση της πορείας
            kp: Ο αναλογικός συντελεστής διόρθωσης της πορείας
            ki: Ο ολοκληρωτικός συντελεστής διόρθωσης της πορείας
            kd: Ο διαφορικός συντελεστής διόρθωσης της πορείας
            dt: Ο χρόνος που διαρκεί κάθε  επανάληψη του αλγορίθμου
            minDegrees: Η ελάχιστη απόσταση που θα διαρκέσει η διαδικασία ακολούθησης της γραμμής
            maxDegrees: Η μέγιστη απόσταση ακολούθησης της γραμμής
            minSpeed: Η αρχική ισχύς των μοτέρ κατα την εκκίνηση της διαδικασίας ακολούθησης
            maxSpeed: Η μέγιστη ισχύς των μοτέρ κατα την διαδικασία ακολούθησης
            stop_action: Ο τρόπος με τον οποίο σταματάνε τα μοτέρ στο τέλος της διαδικασίας ('brake','coast','hold')
            stopAtEnd: True: σβήσιμο των μοτέρ στο τέλος της διαδικασίας, False: τα μοτέρ παραμένουν ανοιχτά (περίπτωση που ακολουθεί νέα ακολούθηση)
            waits: Μια λίστα με τις ενέργεις που εφαρμόζονται στο τέλος της κίνησης.
                   b1 σημαίνει color1.WaitForBlack
                   w1 σημαίνει color1.WaitForWhite
                   200 σημαίνει περίμενε γιαr 200 μοίρες
                   για παράδειγμα  waits=['w2','b2',120] σημαίνει color2.WaitForWhite(). color2.WaitForBlack, waitForDegrees(200)
        return:
            Οι μοίρες που κινήθηκε το μοτέρ (λόγω αδράνειας) μετά την εντολή τερματισμού της διαδικασίας.

        
        """
        currentAngle=self.gyro.GetAngle()
        target=round(currentAngle / autoCorrectAngle)*autoCorrectAngle+angle
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s +="\nFollowAngle(target="+str(target)+",kp="+str(kp)
        s +=",ki="+str(ki)+",kd="+str(kd)+",dt="+str(dt)+",minDistance="+str(minDistance)+", maxDegrees="+str(maxDistance)
        s +=",minspeed=" + str(minSpeed) + ",maxPower=" + str(maxSpeed) +  ",stop_action='" + stop_action + "')"
        self.gyro.timer.dt=dt
        self.maxSpeed=maxSpeed
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees=self.DistanceToDegrees(maxDistance)    
        
        self.gyro.pid=myPID(name=self.name+'.FollowAngle (pid)',kp=kp,ki=ki,kd=kd,loggingStats=True,direction=1)
        
        i=0
        # action=None
        self.action=None
        currPower=minSpeed
        powerStep=currPower/abs(currPower)
        self.markDegrees=minDegrees
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        # διαβάζεις νέα είσοδο. 
        input=self.gyro.GetAngle()
        # Υπολογίζεις διόρθωση
        # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
        target1=target
        cor=self.gyro.pid.Step(inputRead=input,target=target)
        
        self.timer.Start()
        start=time.perf_counter()
        iswhite=False
        while  True:            
            i+=1
            # Ευρεση απόστασης
            distB=self.motorB.GetDegrees()
            distC=self.motorC.GetDegrees()
            self.soDegrees = abs(distB + distC) / 2
            # έλεγχος για υπέρβαση μέγιστης απόστασης
            if self.soDegrees>=maxDistance:
                break
            # αν έχει υλοποηθεί η ελάχιστη απόσταση ελέγχονται τα waits
            # if self.soDegrees>=minDegrees:
            #     if stopAtLineSensor==2:
            #         sensor=self.color2
            #     elif stopAtLineSensor==3:
            #         sensor=self.color3
            #     elif stopAtLineSensor==1:
            #         sensor=self.color1
            #     else:
            #         sensor=self.color4
            #     colorx=sensor.GetColor(method=3)
            #     if colorx==6:
            #         iswhite=True
            #     if iswhite:
            #         if colorx==1:
            #             self.Stop()
            #             break
                        
            # self.Print("target=",target," input=",input," corr=",cor)
            # time.sleep(1)
            self.motorB.SetSpeed(speed=(currPower+cor),accelerationTime=accelerationTime)
            self.motorC.SetSpeed(speed=(currPower-cor),accelerationTime=accelerationTime)
            if abs(currPower)<abs(self.maxSpeed):
                currPower += powerStep
            if abs(currPower)>abs(self.maxSpeed):
                currPower -= powerStep*2.5
            # Περίμενε τουλάχιστον για χρόνο dt
            self.timer.Step()
            # διαβάζεις νέα είσοδο. 
            input=self.gyro.GetAngle()
            # Υπολογίζεις διόρθωση
            # cor=self.gyro.pid.Step(inputRead=input,target=self.lastAngle+target)
            # self.Print('input:',input,'target=',target,'lastangle+target=',self.lastAngle+target,'cor=',cor)
            cor=self.gyro.pid.Step(inputRead=input,target=target)
            cor2=0
            if self.color2.GetColor(method=3)==6:
                cor2=cor2
            elif self.color2.GetColor(method=3)==1:
                cor2=cor2+20
            else:
                cor2=cor+10
            if self.color3.GetColor(method=3)==6:
                cor2=cor2
            elif self.color3.GetColor(method=3)==1:
                cor2=cor2-20
            else:
                cor2=cor-10
            cor=cor+cor2

            if self.gyro.logger.level<2:
                self.Print('input:',input,'target=',target,'cor=',cor,' b=',currPower+cor,' c=',currPower-cor)
    
        totalTime=time.perf_counter()-start
        if stopAtEnd:
            self.Stop()

        distB=self.motorB.GetDegrees()
        distC=self.motorC.GetDegrees()
        self.soDegrees = abs(distB + distC) / 2
        self.extraDegrees= self.soDegrees - self.markDegrees
                
        if self.logger.level<=2:
            s += self.gyro.pid.GetStatistics()        
            s +="\n| dt (real)="+str(totalTime/i)
            s += '\n| degrees/dt:'+str((distB+distC)/2/i)+' ('+str(self.DegreesToDistance(distB+distC)/2/i)+' mm/dt)'
            s += '\n| Gyro Drift (degrees/sec):'+str(self.gyro.driftError)
            s += '\n| motorB:'+str(distB)+" ("+str(self.DegreesToDistance(distB))+" mm)"
            s += '\n| motorC:'+str(distC)+" ("+str(self.DegreesToDistance(distC))+" mm)"
            s += "\n| topSpeed= "+str((abs(self.motorB.topSpeed)+abs(self.motorC.topSpeed))/2.0)
            s += "\n| topPower= "+str((abs(self.motorB.topPower)+abs(self.motorC.topPower))/2.0)
            s += "\n| soDistance= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
            s += "\n| markDistance=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
            s += "\n| extraDistance= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
            s += "\n=========================================================="
            self.logger.Debug(s)
            #self.logger.Save()
        return(self.extraDegrees)
 



    def MoveTank(self,speedB=30,speedC=30,minDistance=100,maxDistance=9999999999999,waits=[],accelerationTime=0,comments="",stopAtEnd=True):            
        """[summary]

        Args:
            speedB (int, optional): [description]. Defaults to 30.
            speedC (int, optional): [description]. Defaults to 30.
            minDistance (int, optional): [description]. Defaults to 100.
            maxDistance (int, optional): [description]. Defaults to 9999999999999.
            waits (list, optional): [description]. Defaults to [].
            accelerationTime (int, optional): [description]. Defaults to 0.
            comments (str, optional): [description]. Defaults to "".
            stopAtEnd (bool, optional): [description]. Defaults to True.
        """
        # δημιουργία κειμένου για log
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s+='\n| MoveTank(speedB='+str(speedB)+', speedC='+str(speedC)+', minDistance='+str(minDistance)+', maxDistance='+str(maxDistance)
        s +=', waits=' + self.ListToStr(waits)+', accelerationTime='+str(accelerationTime) + ",stopAtEnd=" +self.ToString(stopAtEnd) + ")"
        s += '\n------------------------------------------------------------------'
        # αν το επίπεδο καταγραφής είναι info ή debug
        if self.logger.level<=2:
            self.logger.Debug(s)
            s=''
        # αρχικοποίησε το action σε None
        self.action=None
        # όρισε τα waits του αντικειμένου όπως την σχετική παράμετρο 
        self.waits=waits
        # αρχικοποίησε τις μοίρες των μοτέρ
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        # για το έλεγχο ταχύτητας μέσα απο τα waits
        self.maxSpeed=max(speedB,speedC)
        firstMax=self.maxSpeed
        # μετατροπή τηε απόστασης σε μοίρες
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees = self.DistanceToDegrees(maxDistance)

        # αρχικοποίησε το markdistance στην ελάχιστη απόσταση που εκτελεστεί η διαδικασία
        self.markDegrees=minDegrees
        
        i=0
        # Καταγραφή της έναρξης της διαδικασίας για τον υπόλογισμό του μέσου dt
        start=time.perf_counter()
        while True:
            i+=1
            # Ευρεση απόστασης
            degrB=self.motorB.GetDegrees()
            degrC=self.motorC.GetDegrees()
            self.soDegrees = abs(degrB + degrC) / 2
            # έλεγχος για υπέρβαση μέγιστης απόστασης
            if self.soDegrees>=maxDegrees:
                break
            # αν έχει υλοποηθεί η ελάχιστη απόσταση ελέγχονται τα waits
            if self.soDegrees>=minDegrees:
                # εκτελούνται τα waits. Aν δεν υπάρχει action στα waits ολοκληρώνουμε 
                if self.actionsDone():
                    break
            # Περίμενε τουλάχιστον για χρόνο dt
            self.timer.Step()
            self.motorB.SetSpeed(speed=speedB*self.maxSpeed/firstMax,accelerationTime=accelerationTime)
            self.motorC.SetSpeed(speed=speedC*self.maxSpeed/firstMax,accelerationTime=accelerationTime)
        if stopAtEnd:
            self.Stop()
        # Καταγραφή του τέλους της διαδικασίας για τον υπόλογισμό του μέσου dt
        totalTime=time.perf_counter()-start
        degrB=self.motorB.GetDegrees()
        degrC=self.motorC.GetDegrees()
        self.soDegrees = abs(degrB + degrC) / 2
        self.extraDegrees= self.soDegrees - self.markDegrees
        
        if self.logger.level<=2:
            #if self.logger.level<=1:
            s +="\n| dt="+str(totalTime/i)
            s += '\n| degrees/dt:' + str((degrB + degrC) / 2 / i) + ' (' + str(self.DegreesToDistance(degrB + degrC) / 2 / i) + ' mm/dt)'
            s += '\n| motorB:' + str(degrB) + " (" + str(self.DegreesToDistance(degrB)) + " mm)"
            s += '\n| motorC:' + str(degrC) + " (" + str(self.DegreesToDistance(degrC)) + " mm)"
            s += "\n| soDegrees= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
            s += "\n| markDegrees=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
            s += "\n| topSpeeds b:"+str(self.motorB.topSpeed)+' c:'+str(self.motorC.topSpeed)
            s += "\n| topPowers b:"+str(self.motorB.topPower)+' c:'+str(self.motorC.topPower)
            s += "\n| extraDegrees= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
            if self.gyro.initialized:
                s += "\n| angle= " + str(self.gyro.GetAngle())  + " dgs"
            s += "\n=========================================================="
            self.logger.Debug(s)

    def MoveSteering(self,steering=0,speed=30,minDistance=100,maxDistance=9999999999999,waits=[],accelerationTime=0,comments="",stopAtEnd=True):       
        """[summary]

        Args:
            steering (int, optional): [description]. Defaults to 0.
            speed (int, optional): [description]. Defaults to 30.
            minDistance (int, optional): [description]. Defaults to 100.
            maxDistance (int, optional): [description]. Defaults to 9999999999999.
            waits (list, optional): [description]. Defaults to [].
            accelerationTime (int, optional): [description]. Defaults to 0.
            comments (str, optional): [description]. Defaults to "".
            stopAtEnd (bool, optional): [description]. Defaults to True.
        """

        # Εύρεση ταχυτήτων μοτέρ από ταχύτητα steering 
        steer=self.Steering(steering=steering,power=speed) 
        # Υλοποίηση του MoveSteering μέσω MoveTank 
        self.MoveTank(speedB=steer[0],speedC=steer[1],minDistance=minDistance,maxDistance=maxDistance,waits=waits,accelerationTime=accelerationTime,comments=comments)          

    def MoveStraight(self,speed=40,kp=1,minSpeed=0,maxSpeed=80,minDistance=100,maxDistance=99999999,waits=[],accelerationTime=0,comments='',stopAtEnd=True):
        # δημιουργία κειμένου για log
        s = "\n=================================================================="
        if comments!="":
            s+="\n| --> "+comments+" <--"
            s+="\n=================================================================="
        s+='\n| MoveStraight(minDistance='+str(minDistance)+', maxDistance='+str(maxDistance)
        s +=', waits=' + self.ListToStr(waits)+', accelerationTime='+str(accelerationTime) + ",stopAtEnd=" +self.ToString(stopAtEnd) + ")"
        s += '\n------------------------------------------------------------------'
        # αν το επίπεδο καταγραφής είναι info ή debug
        if self.logger.level<=2:
            self.logger.Debug(s)
            s=''
        # αρχικοποίησε το action σε None
        self.action=None
        # όρισε τα waits του αντικειμένου όπως την σχετική παράμετρο 
        self.waits=waits
        # αρχικοποίησε τις μοίρες των μοτέρ
        self.motorB.ResetDegrees()
        self.motorC.ResetDegrees()
        # για το έλεγχο ταχύτητας μέσα απο τα waits
        self.maxSpeed=speed
        # firstMax=self.maxSpeed
        # μετατροπή τηε απόστασης σε μοίρες
        minDegrees=self.DistanceToDegrees(minDistance)
        maxDegrees = self.DistanceToDegrees(maxDistance)

        # αρχικοποίησε το markdistance στην ελάχιστη απόσταση που εκτελεστεί η διαδικασία
        self.markDegrees=minDegrees
        
        i=0
        # Καταγραφή της έναρξης της διαδικασίας για τον υπόλογισμό του μέσου dt
        start=time.perf_counter()
        while True:
            i+=1
            error=self.motorB.GetDegrees()-self.motorC.GetDegrees()
            corr=error*kp
            # Ευρεση απόστασης
            degrB=self.motorB.GetDegrees()
            degrC=self.motorC.GetDegrees()
            self.soDegrees = abs(degrB + degrC) / 2
            # έλεγχος για υπέρβαση μέγιστης απόστασης
            if self.soDegrees>=maxDegrees:
                break
            # αν έχει υλοποηθεί η ελάχιστη απόσταση ελέγχονται τα waits
            if self.soDegrees>=minDegrees:
                # εκτελούνται τα waits. Aν δεν υπάρχει action στα waits ολοκληρώνουμε 
                if self.actionsDone():
                    break

            self.timer.Step()
            self.motorB.SetSpeed(speed=speed-corr)
            self.motorC.SetSpeed(speed=speed+corr)
        if stopAtEnd:
            self.Stop()
        # Καταγραφή του τέλους της διαδικασίας για τον υπόλογισμό του μέσου dt
        totalTime=time.perf_counter()-start
        degrB=self.motorB.GetDegrees()
        degrC=self.motorC.GetDegrees()
        self.soDegrees = abs(degrB + degrC) / 2
        self.extraDegrees= self.soDegrees - self.markDegrees
        
        if self.logger.level<=2:
            #if self.logger.level<=1:
            s +="\n| dt="+str(totalTime/i)
            s += '\n| degrees/dt:' + str((degrB + degrC) / 2 / i) + ' (' + str(self.DegreesToDistance(degrB + degrC) / 2 / i) + ' mm/dt)'
            s += '\n| motorB:' + str(degrB) + " (" + str(self.DegreesToDistance(degrB)) + " mm)"
            s += '\n| motorC:' + str(degrC) + " (" + str(self.DegreesToDistance(degrC)) + " mm)"
            s += "\n| soDegrees= " + str(self.soDegrees) + " (" + str(self.DegreesToDistance(self.soDegrees)) + " mm)"
            s += "\n| markDegrees=" + str(self.markDegrees) + " (" + str(self.DegreesToDistance(self.markDegrees)) + " mm)"
            s += "\n| topSpeeds b:"+str(self.motorB.topSpeed)+' c:'+str(self.motorC.topSpeed)
            s += "\n| topPowers b:"+str(self.motorB.topPower)+' c:'+str(self.motorC.topPower)
            s += "\n| extraDegrees= " + str(self.extraDegrees) + " (" + str(self.DegreesToDistance(self.extraDegrees)) + " mm)"
            s += "\n=========================================================="
            self.logger.Debug(s)

    def GetStatus(self):
        s=' Start Robot Report ===================================='
        s+='| lastMessage:'+self.lastMessage
        s+='\n End   Robot Report ===================================='
        return(s)

    def CheckStatus(self,step=1):
        self.Print(step,') Start Sensor Check ====================================')
        if self.motorA.initialized:
            self.Print('| A:',self.motorA.GetDegrees())
        if self.motorB.initialized:
            self.Print('| B:',self.motorB.GetDegrees())
        if self.motorC.initialized:
            self.Print('| C:',self.motorC.GetDegrees())
        if self.motorD.initialized:
            self.Print('| D:',self.motorD.GetDegrees())
        if self.color1.initialized:
            self.Print('| 1.White:,',self.color1.GetLight(),' (',self.color1.GetLight(adjusted=False),') ','rgb:',self.color1.GetRGB(),' colors:',self.color1.GetColor(method=1),'-',self.color1.GetColor(method=2),'-',self.color1.GetColor(method=3))
        if self.color2.initialized:
            self.Print('| 2.White:,',self.color2.GetLight(),' (',self.color2.GetLight(adjusted=False),') ','rgb:',self.color2.GetRGB(),' colors:',self.color2.GetColor(method=1),'-',self.color2.GetColor(method=2),'-',self.color2.GetColor(method=3))
        if self.color3.initialized:
            self.Print('| 3.White:,',self.color3.GetLight(),' (',self.color3.GetLight(adjusted=False),') ','rgb:',self.color3.GetRGB(),' colors:',self.color3.GetColor(method=1),'-',self.color3.GetColor(method=2),'-',self.color3.GetColor(method=3))
        if self.color4.initialized:
            self.Print('| 4.White:,',self.color4.GetLight(),' (',self.color4.GetLight(adjusted=False),') ','rgb:',self.color4.GetRGB(),' colors:',self.color4.GetColor(method=1),'-',self.color4.GetColor(method=2),'-',self.color4.GetColor(method=3))
        if self.gyro.initialized:
            self.Print('| gyro:',self.gyro.GetAngle())
        if self.ultraSonic.initialized:
            self.Print('| ultrasonic:',self.ultraSonic.GetDistance())
        self.Print('| lastMessage:',self.lastMessage)
        self.Print(step,') End   Sensor Report ====================================')


    def PrintStatus(self,SaveToLog=True):
        s=self.GetStatus()
        self.logger.Out(s)

class myCar(legoPart):
    """Η βασική κλάση διαχείρισης του Αυτοκινήτου
    """
    display=brick.display
    motorA = myMotor()
    motorB = myMotor()
    motorC = myMotor()
    buttons=brick.buttons
    gyro = myGyro()
    # gyro1 = myGyro()
    # gyro2 = myGyro()
    # pi=math.pi()
    cm2dgrs=0.08036417322834644
    ultraSonic1 = myUltraSonic()
    ultraSonic2 = myUltraSonic()
    ultraSonic3 = myUltraSonic()
    # color1  = myColorSensor()
    lastMessage=''
    # time mark
    started=0
    duration=0
    degreesToDistanceRatio=1
    steeringGearRatio=5
    maxSteeringAngle=50
    speed=10
    angle=0
    angleCorrection=0
    angleCorrectionSum=0
    angleCorrectionI=0
    
    angleCorrection_d0=0
    angleCorrection_sx=0
    angleCorrection_sy=0
    angleCorrectionState=0
    angleCorrectionMarkDistance=0
    angleCorrectionAngle0=0
    
    lastFullDistance=0
    lastLeftDistance=0
    lastRightDistance=0
    lastAngle=0



    # angle1=0
    # angle2=0
    # color=0
    steering=0
    steeringCorrection=0
    steeringCorrectionState=0
    steeringCorrectionS1=0
    steeringCorrectionS2=0
    steeringCorrectionI=0
    

    sonarAngle=0
    sensor_i=0
    frontDistance=300
    leftDistance=300
    rightDistance=300
    distance=0
    fullDistance=0
    markDistance=0
    logcsv=''
    data=[]
    clockWise=False
    mins=[]
    maxs=[]
    # use to calc leftDistance
    lastLeftDistance=0
    lastCarDistance=0

    def __init__(self, loggingLevel=3):
        """[Αρχικοποιεί το αντικείμενο Car.]

        Keyword Arguments:
            loggingLevel {int} -- [Το επίπεδο καταγραφής] (default: {3})
        """
        self.name = 'myCar'
        self.lastMessage=''
        self.timer=myTimer(dt=0.01)
        self.logger = myLogger(name=self.name)
        self.logger.level=loggingLevel
        self.loggingLevel=loggingLevel

    def Beep(self):
        """[summary]
        """
        thread1 = threading.Thread(target=brick.sound.beep,kwargs={})
        thread1.start()

    def Display(self,text='',coordinate=None):
        """Εμφανίζει κείμενο στην οθόνη του Ρομπότ

        Args:
            text (str, optional): [description]. Defaults to ''.
            coordinate ([type], optional): [description]. Defaults to None.
        """
        self.display.text(text,coordinate)
    
    def ButtonPressed(self):
        """Επιστρέφει αν πατήθηκε κάποιο πλήκτρο στο Robot

        Returns:
            [bool]: [αν πατήθηκε πλήκτρο]
        """
        return(brick.buttons())

    def Run(self,speed=20,steering=0):
        """[summary]

        Args:
            speed (int, optional): [description]. Defaults to 20.
            steering (int, optional): [description]. Defaults to 0.
        """
        self.motorB.SetSpeed(speed=speed)
        if steering>0:
            if steering>self.motorA.GetDegrees():
                self.motorA.SetSpeed(speed==20)
            else:
                self.motorA.Off()
        else:
            if steering<self.motorA.GetDegrees():
                self.motorA.SetSpeed(speed=-20)
            else:
                self.motorA.Off()

    def ResetDistance(self):
        self.motorB.ResetDegrees()

    def GetCarDistance(self):
        self.fullDistance=abs(self.motorB.GetDegrees()*self.degreesToDistanceRatio*self.cm2dgrs)
        
        return(self.fullDistance)

    def Start(self,waitForButton=False,beep=True):  
        """Έναρξη της λειτουργίας του Ρομποτ. Ενεγοποίηση του εσωτερικού χρονομέτρου.
        1) Ενημερώνεται στα μέρη του ρομπότ η πατρική κλάση.
        2) Υπάρχει η δυνατότητα να περιμένει για πάτημα κουμπιού για έναρξη της λειτουργίας.

        Args:
            waitForButton (bool): [Να περιμένει για πάτημα πλήκτρου του Ρομπότ.]. Defaults to 'False'.
        """     


        if self.motorA.initialized or self.motorA.errors>0:
            self.motorA.SetParent(self)
        if self.motorB.initialized or self.motorB.errors>0:
            self.motorB.SetParent(self)
        if self.motorC.initialized or self.motorC.errors>0:
            self.motorC.SetParent(self)
        # if self.color1.initialized or self.color1.errors>0:
        #     self.color1.SetParent(self)
        if self.gyro.initialized or self.gyro.errors>0:
            self.gyro.SetParent(self)
        # if self.gyro1.initialized or self.gyro1.errors>0:
        #     self.gyro1.SetParent(self)
        # if self.gyro2.initialized or self.gyro2.errors>0:
        #     self.gyro2.SetParent(self)

        if self.ultraSonic1.initialized or self.ultraSonic1.errors>0:
            self.ultraSonic1.SetParent(self)
        if self.ultraSonic2.initialized or self.ultraSonic2.errors>0:
            self.ultraSonic2.SetParent(self)
        if self.ultraSonic3.initialized or self.ultraSonic2.errors>0:
            self.ultraSonic3.SetParent(self)
        self.Check()
        

        if beep:
            self.Beep()
        if waitForButton:
            # self.Display(text='Duration: '+str(self.duration))
            self.Display(text='Press any key to Start')
            self.WaitForButton()
        brick.display.image(ImageFile.AWAKE)
        self.started=time.perf_counter()

    def Finish(self,waitForButton=False,printStatus=False):
        """Όλοκλήρωση της αποστολής του Ρομπότ. Καταγραφή του χρόνου λειτουργίας του Ρομπότ. 
        1) Εμφάνιση της κατάσταση κάθε αισθητήρα και μοτέρ. Αποθήκευση του αρχείου με τις καταγραφές.
        22) Υπάρχει η δυνατότητα να περιμένουμε για πάτημα κουμπιού πρίν την ολοκλήρωση του προγράμματος.

        Args:
            waitForButton (bool, optional): [Αν επιθυμούμε την αναμονή για πλήκτρο στο τέλος της Αποστολής]. Defaults to False.
        """
        parts=[self.ultraSonic2,self.ultraSonic3,self.gyro,self.motorA,self.motorB,self.motorC]
        self.duration=-self.started+time.perf_counter()
        self.logger.Out('Mission Duration: '+str(self.duration))
        if printStatus:
            for part in parts:
                if part.initialized:
                    self.logger.Out(part.GetStatus())

        # if self.color1.initialized:
        #     self.logger.Warning(self.color1.GetStatus())
        # if self.color2.initialized:
        #     self.logger.Warning(self.color2.GetStatus())
        # if self.color3.initialized:
        #     self.logger.Warning(self.color3.GetStatus())
        # if self.color4.initialized:
        #     self.logger.Warning(self.color4.GetStatus())
        # if self.gyro.initialized:
        #     self.logger.Warning(self.gyro.GetStatus())
        # if self.ultraSonic.initialized:
        #     self.logger.Warning(self.ultraSonic.GetStatus())
        self.logger.Save()
        
        if waitForButton:
            self.Display(text='Duration: '+str(self.duration))
            self.Display(text='Press a Robot button to continue')
            self.logger.Warning('Press a Robot button to continue')
            self.WaitForButton()
        

    def WaitForButton(self,delayTimeBefore=0.3,dt=0.01):
        """Περιμένει μέχρι να πατηθεί κάποιο πλήκτρο στο Brick.

        Parameters:
            delayTimeBefore (float): Η καθυστέρηση πρίν ξεκινήσει η αναμονή για πλήκτρο
            dt (float): Η καθυστέρηση ανάμεσα στους ελέγχους για πάτημα πλήκτρου  
    
        return:
            None
        """
        if delayTimeBefore>0:
            time.sleep(delayTimeBefore)
        while not self.ButtonPressed():
            time.sleep(dt)
        self.Beep()

    def Check(self):  
        """
        Έλεγχει τα μέρη του Ρομπότ για σφάλματα αρχικοποίησης
        """     
        self.logger.Save()

        if not self.gyro.initialized:
            self.Print('gyro not initialized')
        # if not self.gyro1.initialized:
        #     self.gyro=self.gyro2
        # elif not self.gyro2.initialized:
        #     self.gyro=self.gyro1
        # elif abs(self.gyro1.driftError)>abs(self.gyro2.driftError):
        #     self.gyro=self.gyro2
        # else:
        #     self.gyro=self.gyro1

        imgs=[]
        
        if self.motorA.errors>0:
            imgs.append("./images/a.png")
        if self.motorB.errors>0:
            imgs.append("./images/b.png")
        if self.motorC.errors>0:
            imgs.append("./images/c.png")

        # if self.gyro1.errors>0:
        #     imgs.append("./images/1.png")
        if self.gyro.errors>0:
            imgs.append("./images/4.png")
        # if self.color1.errors>0:
        #     imgs.append("./images/2.png")
        # if self.ultraSonic1.errors>0:
        #     imgs.append("./images/1.png")
        if self.ultraSonic1.errors>0:
            imgs.append("./images/1.png")
        if self.ultraSonic2.errors>0:
            imgs.append("./images/2.png")
        if self.ultraSonic3.errors>0:
            imgs.append("./images/3.png")

        # if an ultrasonic not initialized use the other sensor
        # if not self.ultraSonic1.initialized:
        #     self.ultraSonic1=self.ultraSonic2
        # if not self.ultraSonic2.initialized:
        #     self.ultraSonic2=self.ultraSonic1
        if len(imgs)>0:
            try:
                brick.display.image(imgs[0])                
            except Exception as e:
                brick.display.image(ImageFile.WARNING)
                self.logger.Error('error in display image '+imgs[0]+' (in robot.check)')
                
        else:
            img1="./images/tap.png"
            try:
                brick.display.image(img1)
            except Exception as e:
                brick.display.image(ImageFile.UP)
                self.logger.Error('error in display image '+img1+' (in robot.check)')

    def ScanObjects(self,scanAngle=50,scanSpeed=30):
        t0=time.perf_counter()
        i=0
        mind1=300
        # turn right
        while True:
            self.UpdateSendorData(log=True)
            g1=self.sonarAngle
            d1=self.frontDistance
            if d1<mind1:
                mind1=d1
                ming1=g1
            i+=1
            x=self.SetSonarAngle(angle=scanAngle,speed=scanSpeed,maxAngle=70)
            if abs(x)<5:
                self.motorC.Off()
                break
                
        
        # turn left
        while True:
            self.UpdateSendorData(log=True)
            g1=self.sonarAngle
            d1=self.frontDistance
            if d1<mind1:
                mind1=d1
                ming1=g1
            i+=1
            x=self.SetSonarAngle(angle=-scanAngle,speed=scanSpeed,maxAngle=70)
            if abs(x)<5:
                self.motorC.Off()
                break
        
        # look ahead
        while True:
            self.UpdateSendorData(log=True)
            g1=self.sonarAngle
            d1=self.frontDistance
            if d1<mind1:
                mind1=d1
                ming1=g1
            i+=1
            x=self.SetSonarAngle(angle=0,speed=scanSpeed,maxAngle=70)
            if abs(x)<5:
                self.motorC.Off()
                break
        

        t1=time.perf_counter()
        self.Print('dt:',(t1-t0)/i,' i:',i)
        self.Print('object is at (r,a):',mind1,ming1,')')
        return((mind1,ming1))

    def SetSpeed(self,speed=50,minSpeed=10,maxSpeed=90):
        self.motorB.SetSpeed(speed=speed,minSpeed=minSpeed,maxSpeed=maxSpeed)
        return(self.motorB.GetSpeed())
    
    def Stop(self):
        self.motorB.Off()

    

    def StopAllMotors(self):
        self.motorA.Off()
        self.motorB.Off()
        self.motorC.Off()

    def GetAngle(self):
        self.angle=self.gyro.GetAngle()+self.angleCorrection
        # if self.gyro1.initialized:
        #     self.angle1=self.gyro1.GetAngle()
        # if self.gyro2.initialized :
        #     self.angle2=self.gyro2.GetAngle()

        return(self.angle)
        

    def GetSteering(self):
        # self.steering=self.motorA.GetDegrees()/self.steeringGearRatio
        self.steering=self.motorA.GetDegrees()/self.steeringGearRatio+self.steeringCorrection
        return(self.steering)

    def GetSonarAngle(self):
        self.sonarAngle=self.motorC.GetDegrees()/self.steeringGearRatio
        return(self.sonarAngle)


    def SetSteering(self,angle=0):        
        x=self.motorA.SetAngle(angle=angle*self.steeringGearRatio+self.steeringCorrection*self.steeringGearRatio,minSpeed=10,maxSpeed=60,maxAngle=self.maxSteeringAngle*self.steeringGearRatio,plusMinus=1.5,stopAtEnd=True) 
        error=x/self.steeringGearRatio
        return(error)

    def SetSonarAngle(self,angle=0,speed=30,maxAngle=50):        
        if angle>maxAngle:
            angle=maxAngle
        elif angle< -maxAngle:
            angle=-maxAngle
        x=self.motorC.SetAngle2(angle=angle*self.steeringGearRatio,speed=speed,plusMinus=1.5,stopAtEnd=True) 
        error=x/self.steeringGearRatio
        return(error)


    def GetFrontDistance(self):
        self.frontDistance=self.ultraSonic2.GetDistance()
        return(self.frontDistance)

    def GetLeftDistance(self):
        x=self.ultraSonic1.GetDistance()
        if x<100 and x>10:
            self.leftDistance=x
            self.lastLeftDistance=x
            self.lastCarDistance=self.fullDistance
        else:
            y=self.fullDistance-self.lastCarDistance
            self.leftDistance=-(self.lastLeftDistance+y*math.sin(self.angle*math.pi/180))

        return(self.leftDistance)

    def GetRightDistance(self):
        x=self.ultraSonic3.GetDistance()
        if x<100 and x>10:
            self.rightDistance=x
            self.lastRightDistance=x
            self.lastCarDistance=self.fullDistance
        else:
            y=self.fullDistance-self.lastCarDistance
            self.rightDistance=-(self.lastRightDistance-y*math.sin(self.angle*math.pi/180))

        return(self.rightDistance)


    def GetAngleCorrection(self,target=0,kp=1):
        # self.angle=self.gyro.GetAngle()
        error=target-self.angle
        correction=error*kp
        return(correction)
        
    def CalcTargetAngleFromWallDistance0(self,targetLeftDistance=40,maxCorrection=15,kp=1.5,calcAngleError=False):
        c=self
        
        if c.leftDistance<targetLeftDistance-maxCorrection/2:
            target=maxCorrection
        elif c.leftDistance>targetLeftDistance+maxCorrection/2:
            target=-maxCorrection
        else:
            
            error2=targetLeftDistance-c.leftDistance
            error2=error2*kp
            target=error2
            if calcAngleError:
                c.angleCorrectionI=c.angleCorrectionI+1
                c.angleCorrectionSum=c.angleCorrectionSum+target
        return(target)


    
    def CalcTargetAngleFromLeftWallDistance(self,targetLeftDistance=40,acceptedError=5,maxCorrection=12):
        c=self
        # if c.leftDistance<0:
        #     leftDistance=sumDistance-c.rightDistance
        #     rightDistance=c.rightDistance
        # elif c.rightDistance<0:
        #     leftDistance=c.leftDistance
        #     rightDistance=sumDistance-leftDistance
        # elif c.leftDistance<c.rightDistance:
        #     leftDistance=c.leftDistance
        #     rightDistance=sumDistance-leftDistance
        # else:
        #     leftDistance=sumDistance-c.rightDistance
        #     rightDistance=c.rightDistance

        leftDistance=abs(c.leftDistance)
        rightDistance=abs(c.rightDistance)
        error=leftDistance-targetLeftDistance
        if error<-abs(acceptedError):
            f=maxCorrection
        elif error>abs(acceptedError):
            f=-maxCorrection
        else:
            dy=abs(acceptedError)/math.tan(abs(maxCorrection)*math.pi/180)
            f=-math.atan(error/dy)*180/math.pi
        return(f)

    def CalcTargetAngleFromRightWallDistance(self,targetRightDistance=40,acceptedError=5,maxCorrection=12):
        c=self

        leftDistance=abs(c.leftDistance)
        rightDistance=abs(c.rightDistance)
        error=-rightDistance+targetRightDistance
        if error<-abs(acceptedError):
            f=maxCorrection
        elif error>abs(acceptedError):
            f=-maxCorrection
        else:
            dy=abs(acceptedError)/math.tan(abs(maxCorrection)*math.pi/180)
            f=-math.atan(error/dy)*180/math.pi
        return(f)



    def CalcTargetAngleFromWallDistance2(self,acceptedError=5,maxCorrection=12,sumDistance=65):
        c=self


        error=abs(leftDistance)-targetLeftDistance
        if error<-abs(acceptedError):
            f=maxCorrection
        elif error>abs(acceptedError):
            f=-maxCorrection
        else:
            dy=abs(acceptedError)/math.tan(abs(maxCorrection)*math.pi/180)
            f=-math.atan(error/dy)*180/math.pi
        return(f)


    def calcGyroAngleCorrection(self,init=False,forDistance=40,maxLeftDistance=70,log=False):
        c=self        
        if c.angleCorrectionState==0:
            if c.leftDistance<maxLeftDistance:
                c.angleCorrection_d0=c.leftDistance
                c.angleCorrection_sx=0
                c.angleCorrection_sy=0            
                c.angleCorrectionState=1
                c.angleCorrectionMarkDistance=c.fullDistance
                c.angleCorrectionAngle0=round(c.angle/90)*90
        elif c.angleCorrectionState==1:
            if abs(c.leftDistance-c.lastLeftDistance)>3:
                c.Print('ubnormal left distance change')
            dxi=(c.fullDistance-c.lastFullDistance)*math.sin((c.angle-c.angleCorrectionAngle0)*math.pi/180)
            dyi=(c.fullDistance-c.lastFullDistance)*math.cos((c.angle-c.angleCorrectionAngle0)*math.pi/180)
            c.angleCorrection_sx=c.angleCorrection_sx+dxi
            c.angleCorrection_sy=c.angleCorrection_sy+dyi
            if c.fullDistance>=c.angleCorrectionMarkDistance+forDistance:
                c.angleCorrectionState=2
        elif c.angleCorrectionState==2:
            d1=c.leftDistance
            errorf=math.atan((d1-c.angleCorrection_d0-c.angleCorrection_sx)/c.angleCorrection_sy)*180/math.pi
            c.angleCorrection=c.angleCorrection+errorf
            c.angleCorrectionState=3
            if log:
                # c.Print('d0:',c.angleCorrection_d0)
                # c.Print('d1:',c.leftDistance)
                # c.Print('d1-d0:',self.leftDistance-c.angleCorrection_d0)
                # c.Print('sx:',c.angleCorrection_sx)
                # c.Print('sy:',c.angleCorrection_sy)
                c.Print('errorf:',errorf,'total correction:',c.angleCorrection)

        if init:
            c.angleCorrectionState=0
        # to init angle correction mast set c.anglecorrectionstate to 0

    
    # is not ok 
    def GetDistanceCorrection(self,target=10,kp=0.1):
        error=0
        self.leftDistance=self.GetLeftDistance()
        if self.LeftDistance<90:
            error=self.leftDistance-target
        correction=error*kp
        return(correction)
        
  
    def GoToFrontWall(self,minSpeed=30,maxSpeed=50,minDistance=100,distanceFromFrontWall=50,leftLaneDistance=35,rightLaneDistance=55,detectObjects=False,maxSonarAngle=15,log=False):
        c=self
        c.markDistance=c.fullDistance+minDistance
        speed=minSpeed
        sonarTarget=maxSonarAngle
        wallDistanceTarget1=leftLaneDistance
        wallDistanceTarget2=rightLaneDistance
        wallDistanceTarget=wallDistanceTarget1
        kpAngle=1.5
        kpFix=3.5
        
        currentAngle=self.gyro.GetAngle()
        angle0=round(currentAngle / 90)*90

        i=0
        fix=0
        speedStep=1
        while True:
            # if c.frontDistance>100:
            #     if speed<maxSpeed:
            #         speed=speed+speedStep
            # else:
            #     if speed>minSpeed:
            #         speed=speed-speedStep
                
            c.SetSpeed(speed=speed,minSpeed=minSpeed,maxSpeed=minSpeed)
            c.UpdateSendorData(log=False)
            # calc gyro errpr
            error2=angle0-c.angle
    
            # calc distance from wall correction
            error3=0
            if c.fullDistance>=c.markDistance:
                if c.frontDistance<=distanceFromFrontWall:
                    c.StopAllMotors()
                    break
                    
            if detectObjects and c.frontDistance<25:

                if log:
                    c.LogSensorData()
                c.Beep()
                # change Lane to avoid object
                if wallDistanceTarget!=wallDistanceTarget1:
                    c.RightToLeftLane()
                    wallDistanceTarget=wallDistanceTarget1
                else:
                    c.LeftToRightLane()
                    wallDistanceTarget=wallDistanceTarget2
                # c.markDistance=c.fullDistance+40

            # error distance from wall
            error3=0
            # if wallDistanceTarget!=wallDistanceTarget2:
            #     if c.leftDistance>5 and c.leftDistance<100:
            #         error3=(wallDistanceTarget-c.leftDistance)*2.5
            #         if error3>10:
            #             error3=10
            #         if error3<-10:
            #             error3=-10

            correction=error2*kpAngle+error3*kpFix
            c.SetSteering(correction)

            c.SetSonarAngle(angle=0,speed=20)


    def Turn(self,angle=-90,speed=30,stopAtEnd=False):
        c=self
        c.UpdateSendorData()
        laneAngle=round(self.angle/90)*90
        while True:
            c.UpdateSendorData()
            c.MoveToAngle(targetAngle=laneAngle+angle,speed=speed)
            c.SetSonarAngle(angle=laneAngle+angle-c.angle,speed=30)
            if abs(laneAngle+angle-c.angle)<5:
                # markos=c.fullDistance+20
                break
        if stopAtEnd:
            c.StopAllMotors()

    def LogSensorData(self):
        self.dataRow=(self.angle,self.steering,self.sonarAngle,self.frontDistance,self.leftDistance,self.rightDistance,self.fullDistance-self.markDistance,self.fullDistance)   
        self.data.append(self.dataRow)

    def UpdateSendorData(self,print=False,log=False):
        # store previous values
        self.lastFullDistance=self.fullDistance        
        self.lastLeftDistance=self.leftDistance        
        self.lastRightDistance=self.rightDistance        
        
        self.lastAngle=self.angle        

        # Get new Values
        self.sensor_i=self.sensor_i+1
        self.angle=self.GetAngle()
        self.steering=self.GetSteering()
        self.sonarAngle=self.GetSonarAngle()
        self.fullDistance=self.GetCarDistance()
        if self.sensor_i % 3 ==0:
            self.frontDistance=self.GetFrontDistance()
        elif self.sensor_i % 3 ==1:
            self.leftDistance= self.GetLeftDistance()
        else:    
            self.rightDistance= self.GetRightDistance()
        
        if print:
            s= "\n         angle:"+str(self.angle) 
            s+="\n      steering:"+str(self.steering)
            s+="\n    sonarAngle:"+str(self.sonarAngle)
            s+="\n frontDistance:"+str(self.frontDistance)
            s+="\n  leftDistance:"+str(self.leftDistance)
            s+="\n  rightDistance:"+str(self.rightDistance)
            s+="\n  fullDistance:"+str(self.fullDistance)
            self.Print(s)
        if log:
            self.LogSensorData()

    def UpdateSendorDataLimited(self,print=False,log=False):
        # store previous values
        self.lastFullDistance=self.fullDistance        
        self.lastLeftDistance=self.leftDistance        
        self.lastRightDistance=self.rightDistance        
        
        self.lastAngle=self.angle        

        # Get new Values
        self.angle=self.GetAngle()
        self.steering=self.GetSteering()
        self.sonarAngle=self.GetSonarAngle()
        self.frontDistance=self.GetFrontDistance()
        self.fullDistance=self.GetCarDistance()
        self.leftDistance= 300
        self.rightDistance= 300
        
        if print:
            s= "\n         angle:"+str(self.angle) 
            s+="\n      steering:"+str(self.steering)
            s+="\n    sonarAngle:"+str(self.sonarAngle)
            s+="\n frontDistance:"+str(self.frontDistance)
            s+="\n  leftDistance:"+str(self.leftDistance)
            s+="\n  rightDistance:"+str(self.rightDistance)
            s+="\n  fullDistance:"+str(self.fullDistance)
            self.Print(s)
        if log:
            self.LogSensorData()

    def ResetSonarMinMax(self):
        self.mins=[]
        self.maxs=[]

    def UpdateSonarMinMax(self):
        
        if len(self.mins)==0:
            min=301
        else:
            min=self.mins[0]

        if len(self.maxs)==0:
            max=0
        else:
            max=self.maxs[0]

        if self.frontDistance<min:
            self.mins=[self.frontDistance,self.steering+self.sonarAngle,self.steering+self.sonarAngle]
        elif self.frontDistance==min:
            self.mins[2]=self.steering+self.sonarAngle
            
        if self.frontDistance>max:
            self.maxs=[self.frontDistance,self.steering+self.sonarAngle,self.steering+self.sonarAngle]
        elif self.frontDistance==max:
            self.maxs[2]=self.steering+self.sonarAngle

    def GetStatus(self,refresh=True,print=True,log=False):
        if refresh:
            self.UpdateSendorData()
        if print:
            s= '\n=========== Car Status ==========='
            s+="\n|"+'         angle:'+str(self.angle)            
            s+="\n|"+'         angle1:'+str(self.angle1)            
            s+="\n|"+'         angle2:'+str(self.angle2)            
            s+="\n|"+'      Steering:'+str(self.steering)            
            s+="\n|"+'    SonarAngle:'+str(self.steering)            
            s+="\n|"+' frontDistance:'+str(self.frontDistance)
            s+="\n|"+'  leftDistance:'+str(self.leftDistance)
            # s+="\n|"+'         color:'+str(self.color)                        
            s+="\n|"+'      distance:'+str(self.fullDistance-self.markDistance)                        
            s+="\n|"+'  fullDistance:'+str(self.fullDistance)                        
            s+='\n=================================='
            self.Print(s)
        if log:
            self.dataRow=(self.angle,self.steering,self.sonarAngle,self.frontDistance,self.leftDistance,self.fullDistance-self.markDistance,self.fullDistance,self.angle1,self.angle2)   
            self.data.append(self.dataRow)

    def calcObjectLocationInTurns(self,rowData=[],startingDistanceFromWall=71,print=False):
        carAngle=rowData[0]
        carSteering=rowData[1]
        frontDistance=rowData[2]
        leftDistance=rowData[3]
        carDistance=rowData[4]
        r=carDistance*180/(carAngle*math.pi)
        dy=r*math.sin(carAngle*math.pi/180)
        dx=r-r*math.cos(carAngle*math.pi/180)
        totalAngle=carAngle+carSteering
        sensorToObjectY=frontDistance*math.cos(totalAngle*math.pi/180)
        sensorToObjectX=frontDistance*math.sin(totalAngle*math.pi/180)
        distanceFromWall=startingDistanceFromWall-dy-sensorToObjectY
        if print:
            self.Print("carAngle=",carAngle)
            self.Print("carSteering",carSteering)
            self.Print("frontDistance",frontDistance)
            self.Print("leftDistance",leftDistance)
            self.Print("carDistance",carDistance)
            self.Print("r=",r)
            self.Print("dy=",dy)
            self.Print("dx=",dx)
            self.Print("totalAngle=",totalAngle)
            self.Print("sensorToObjectY",sensorToObjectY)
            self.Print("sensorToObjectX",sensorToObjectX)
            self.Print("distanceFromWall",distanceFromWall)
        return(distanceFromWall)
    
    def CalcObjectLocation(self,distance=40,angle=30,debug=False):
        # απόσταση οχήματος απο τοίχο
        selfdcx=self.leftDistance
        dox=round(distance*math.sin(angle*math.pi/180),2)
        doy=round(distance*math.cos(angle*math.pi/180),2)
        dow=selfdcx+dox
        if debug:
            print('Απόσταση αυτοκινήτου αντικειμένου:',distance)
            print('Γωνία οχήματος αντικειμένου:',angle)
            print('Απόσταση Οχήματος απο τον τοίχο:',selfdcx)
            print('Οριζόντια απόσταση οχήματος - αντικειμένου',dox)
            print('Κάθετη απόσταση οχήματος - αντικειμένου',doy)
            print('Απόσταση τοίχου - αντικειμένου',dow)
        return(dox,doy)
    
    def CalcNextPosition(self,object_dx=40,object_dy=10,safeDistance=20,debug=True):
        target1x=object_dx+safeDistance
        target2x=object_dx-safeDistance
        target1f=round(math.atan((target1x)/object_dy)*180/math.pi,2)
        target2f=round(math.atan((target2x)/object_dy)*180/math.pi,2)
        target1Distance=round(object_dy/math.cos(target1f*math.pi/180),2)
        target2Distance=round(object_dy/math.cos(target2f*math.pi/180),2)
        if debug:
            print('Οριζόντια απόσταση οχήματος - αντικειμένου',object_dx)
            print('Κάθετη απόσταση οχήματος - αντικειμένου',object_dy)
            print('Γωνία πορείας για νέα θέση 1 (διαφυγής)',target1f)
            print('Απόσταση πορείας για νέα θέση 1 (διαφυγής)',target1Distance)
            print('Γωνία πορείας για νέα θέση 2 (διαφυγής)',target2f)
            print('Απόσταση πορείας για νέα θέση 2 (διαφυγής)',target2Distance)
        return((target1Distance,target1f),(target2Distance,target2f))
    
        
        def CalcObjectPosition(self,distance=10,gonia=30):
            # in a lane
            selfdcx=50
            # if gonia<0:
            gonia=90-gonia
            dox=selfdcx+distance*math.cos(gonia*math.pi/180)
            doy=distance*math.sin(gonia*math.pi/180)
            if dox<50:
                targetx=70
            else:
                targetx=30
            if targetx!=selfdcx:
                targetf=math.atan(doy/(targetx-selfdcx))
                targetDistance=(100-selfdcx)/math.cos(targetf*math.pi/180)
                if targetf>90:
                    targetf=-(targetf-90)
            else:
                targetf=0
                targetDistance=doy

            return(targetDistance,targetf)

    def ExportDataToCSV(self,filename='data.csv'):
        s='angle;steering;sonarAngle;fronDistance;leftDistance;rightDistance,distance;fullDistance'
        for item in self.data:
            s+="\r\n"+str(item[0])            
            s+=";"+str(item[1])            
            s+=";"+str(item[2])
            s+=";"+str(item[3])
            s+=";"+str(item[4])                        
            s+=";"+str(item[5])                        
            s+=";"+str(item[6])    
            s+=";"+str(item[7])                                   
                                  
        s=s.replace('.',',')
        # print(s)
        file = open(filename, 'w')
        file.write(s)
        file.close()
        self.data=[]

    # def LeftToRightLane(self,speed=30,angleToTurn=45,distanceFromWall=32):
    #     c=self
        
    #     c.motorC.Off()
    #     # step1: change orientation to +45
    #     c.UpdateSendorData()
    #     currentAngle=self.angle
    #     angle0=round(currentAngle / 90)*90
    #     # first step turn right
    #     while True:
    #         c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed+3)
    #         c.SetSonarAngle(angle=angle0+90-c.angle)
    #         if c.angle-angle0<angleToTurn:
    #             c.SetSteering(angle=angleToTurn)
    #         else:
    #             c.motorC.Off()
    #             break
    #         c.UpdateSendorData()
    #     c.StopAllMotors()
    #     time.sleep(10)
    #     # step 2 got to wall
    #     c.UpdateSendorData()
    #     while True:
    #         c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed+3)
    #         c.SetSonarAngle(angle=angle0+90-c.angle)
    #         # if c.fullDistance<=markDistance:
    #         if c.frontDistance>=distanceFromWall:
    #             c.SetSteering(angle=angleToTurn+angle0-c.angle)
    #             # c.motorC.Off()
    #         else:
    #             break
    #         c.UpdateSendorData()

    #     # step3: Return to angle 0
    #     c.StopAllMotors()
    #     time.sleep(10)
    #     c.UpdateSendorData()
    #     while True:
    #         c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed+3)
    #         c.SetSonarAngle(angle=angle0-c.angle)
    #         if c.angle-angle0>0:
    #             c.SetSteering(angle=angle0-c.angle)
    #         else:
    #             c.motorC.Off()
    #             break
    #         c.UpdateSendorData()
    #     c.StopAllMotors()
    #     time.sleep(10)


    # def RightToLeftLane(self,speed=30,angleToTurn=45,distanceFromWall=32):
    #     c=self
        
    #     c.motorC.Off()
    #     # step1: change orientation to +45
    #     c.UpdateSendorData()
    #     currentAngle=self.angle
    #     angle0=round(currentAngle / 90)*90
    #     while True:
    #         c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed+3)
    #         c.SetSonarAngle(angle=-90+angle0-c.angle)
    #         if c.angle-angle0>-angleToTurn:
    #             c.SetSteering(angle=-angleToTurn)
    #         else:
    #             c.motorC.Off()
    #             break
    #         c.UpdateSendorData()
       
    #     # step 2 Go twards Left
    #     c.UpdateSendorData()
    #     while True:
    #         c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed+3)
    #         c.SetSonarAngle(angle=-90+angle0-c.angle)
    #         # if c.fullDistance<=markDistance:
    #         if c.frontDistance>=distanceFromWall:
    #             c.SetSteering(angle=-angleToTurn+angle0-c.angle)
    #         else:
    #             c.motorC.Off()
    #             break
    #         c.UpdateSendorData()
        

    #     # step3: Return to angle 0
        
    #     c.UpdateSendorData()
    #     while True:
    #         c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed+3)
    #         c.SetSonarAngle(angle=angle0-c.angle)
    #         if c.angle-angle0< -5:
    #             c.SetSteering(angle=angle0-c.angle)
    #         else:
    #             c.motorC.Off()
    #             break
    #         c.UpdateSendorData()
        
 

    def LeftToRightLane(self,speed=40,angleToTurn=45,distanceFromWall=31):
        c=self
        laneAngle=round(c.angle/90)*90
        markos = c.fullDistance + 12
        markos2 = c.fullDistance + 32
        
        # c.LogSensorData()
        while True:
            c.UpdateSendorData()
            c.MoveToAngle(targetAngle=laneAngle+angleToTurn,speed=speed)
            c.SetSonarAngle(angle=laneAngle+90 - c.angle ,speed=40)
    
            if  c.frontDistance <= distanceFromWall and c.fullDistance > markos  :
                # c.LogSensorData()
                break
            if c.fullDistance>=markos2:
                break
        
        markos = c.fullDistance + 20
        while True:
            c.UpdateSendorData()
            c.MoveToAngle(targetAngle=laneAngle,speed=40)
            c.SetSonarAngle(angle=laneAngle-c.angle,speed=40)
            if c.fullDistance > markos :  
                # c.LogSensorData()          
                break   


    def RightToLeftLane(self,speed=40,angleToTurn=-45,distanceFromWall=31):
        c=self
        laneAngle=round(c.angle/90)*90
        markos = c.fullDistance + 12
        markos2 = c.fullDistance + 32
        
        # c.LogSensorData()
        while True:
            c.UpdateSendorData()
            c.MoveToAngle(targetAngle=laneAngle+angleToTurn,speed=speed)
            c.SetSonarAngle(angle=laneAngle-90 - c.angle ,speed=40)
    
            if  c.frontDistance <= distanceFromWall and c.fullDistance > markos  :
                # c.LogSensorData()
                break
            if c.fullDistance>=markos2:
                break
        
        markos = c.fullDistance + 20
        while True:
            c.UpdateSendorData()
            c.MoveToAngle(targetAngle=laneAngle,speed=40)
            c.SetSonarAngle(angle=laneAngle-c.angle,speed=40)
            if c.fullDistance > markos :  
                # c.LogSensorData()          
                break   


    def GoRound1(self,distanceFromWall=40,numberOfTurns=11):
        c=self


    def CalcSteeringError0(self):
        c=self
        i=0
        s1=0
        s2=0
        while True:
            i=i+1
            c.UpdateSendorData()
            s1=s1+c.steering
            if c.fullDistance>60:
                c.LogSensorData()
                c.StopAllMotors()
                break
            correction=c.MoveToAngle(speed=40,targetAngle=0,angleKp=2)
            s2=s2+correction
            c.SetSteering(angle=correction)

            
        c.StopAllMotors()
        c.steeringCorrection=0.5*s1/i+0.5*s2/i

        c.Print('fixst1:',s1/i)
        c.Print('fixst2:',s2/i)
        c.Print('angle1:',c.angle1)
        c.Print('angle2:',c.angle2)
        c.Print('steering correction:',c.steeringCorrection)
        c.Print('i:',i)


    def MoveSteering(self,speed=40,steering=0):
        c=self
        c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed)
        error=c.SetSteering(angle=steering)
        return(error)


    def MoveToAngle(self,speed=40,targetAngle=0,angleKp=2,findSteeringCorrection=False):
        c=self

        c.SetSpeed(speed=speed,minSpeed=speed,maxSpeed=speed)
        correction=c.GetAngleCorrection(target=targetAngle,kp=angleKp)
        c.SetSteering(angle=correction)
        if findSteeringCorrection:
            c.steeringCorrectionI=c.steeringCorrectionI+1
            c.steeringCorrectionS1=c.steeringCorrectionS1+c.steering
            c.steeringCorrectionS2=c.steeringCorrectionS2+correction
        return(correction)


    def SetSteeringCorrection(self):
        c=self
        c.steeringCorrection=0.5*c.steeringCorrectionS1/c.steeringCorrectionI+0.5*c.steeringCorrectionS2/c.steeringCorrectionI

    def SetAngleCorrection(self):
        c=self
        if c.angleCorrectionI>0:
            c.angleCorrection=c.angleCorrection+c.angleCorrectionSum/c.angleCorrectionI
            c.angleCorrectionSum=0
            c.angleCorrectionI=0


    def Turn3(self,speed=50,turn=1,lane=1,log=False):
        # cannot understand lane
        c=self
        if lane==1:
            marko1=20
        else:
            marko1=20
            
        # ang=η γvνια που προχωρά 
        ang0=round(c.angle/90)*90
        ang1=ang0+90*turn
        ang=ang0+40*turn

        lane2ob=False
        marko=c.fullDistance+ marko1
        # Scan for object for specific distance
        while True:
            c.UpdateSendorDataLimited()
            c.MoveToAngle(targetAngle=ang,speed=speed)
            c.SetSonarAngle(angle=ang1-c.angle,speed=40)
            if c.fullDistance>marko:
                c.StopAllMotors()
                break
            if c.frontDistance<60:
                if log and not lane2ob:
                    c.LogSensorData()
                if abs(ang1-c.angle-c.sonarAngle)<10:
                    lane2ob=True
        
        if lane2ob:
            c.Beep()
            c.Print('Found object in turn')
            mariposa = c.fullDistance + 25
            while True:
                c.UpdateSendorDataLimited()
                c.SetSonarAngle(angle=ang0-c.angle,speed=40)
                c.MoveToAngle(targetAngle=ang,speed=speed)
                if c.frontDistance<30 and mariposa < c.fullDistance :
                    c.StopAllMotors()
                    break
            
            mariposa=c.fullDistance+20
            while True:
                c.UpdateSendorDataLimited()
                c.MoveToAngle(targetAngle=ang1,speed=50)
                c.SetSonarAngle(angle=ang1-c.angle,speed=40)
                if c.fullDistance>mariposa:
                    c.StopAllMotors()
                    break
        else:            
            
            if lane==2:
                mariposa=c.fullDistance+30
            else:
                mariposa=c.fullDistance+30
            while True:
                c.UpdateSendorDataLimited()
                c.MoveToAngle(targetAngle=ang1,speed=50)
                c.SetSonarAngle(angle=ang1-c.angle,speed=40)
                if c.fullDistance>mariposa:
                    c.StopAllMotors()
                    break
        if lane2ob:
            return(1)
        else:
            return(2)