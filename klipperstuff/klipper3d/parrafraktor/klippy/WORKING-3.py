
import threading
from threading import Thread

import RPi.GPIO as GPIO
import time
import pyartnet
import asyncio
import socket
import math
from stupidArtnet import StupidArtnetServer

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


# create a callback to handle data when received
def test_callback(data):
    """Test function to receive callback data."""
    # the received data is an array
    # of the channels value (no headers)
    pass


# a Server object initializes with the following data
universe             = 0
subnet               = 0
net                  = 0
#setSimplified     = True
#callback_function =  None


# You can use universe only
#universe = 1
a = StupidArtnetServer()

# For every universe we would like to receive,
# add a new listener with a optional callback
# the return is an id for the listener
u1_listener = a.register_listener(
    universe, callback_function=test_callback)


# or disable simplified mode to use nets and subnets as per spec
# subnet = 1 (would have been universe 17 in simplified mode)
# net = 0
# a.register_listener(universe, sub=subnet, net=net,
#                    setSimplified=False, callback_function=test_callback)


# giving it some time for the demo
time.sleep(1)
# if you prefer not using callbacks, the channel data
# is also available in the method get_buffer()
# use the given id to access it

#speed
minSpeed = 0.00068
#steps
steps = 400

class Stepper():
    
    def __init__(self, name,  direction, pulse ,  enable , position , minSpeed , steps, channelA, channelB, channelC):

        #threading.Thread.__init__(self)
        
        self.name = name
        self.pulse = pulse  # Stepper Drive Pulses
        self.direction = direction  # Controller Direction Bit (High for Controller default / LOW to Force a Direction Change).
        self.enable = enable  # Controller Enable Bit (High to Enable / LOW to Disable).
        self.position = position
        self.speed = 0
        self.steps = steps
        self.target = 0
        self.minSpeed = minSpeed
        
        #dmx channels and values
        self.channelA = channelA
        self.channelB = channelB
        self.channelC = channelC
        self.valA = 0
        self.valB = 0
        self.valC = 0
        self.dmxCounter = 10
        
        
        self.isRunning = False
        self.DIR_Left = GPIO.HIGH
        self.DIR_Right = GPIO.LOW
        
        
        
        
        
        self.lock = threading.Lock()
        
               
        print("Stepper: " + self.name)

        GPIO.setup(self.direction, GPIO.OUT)
        GPIO.setup(self.pulse, GPIO.OUT)
        #GPIO.setup(self.enable, GPIO.OUT)
        
        #GPIO.output(self.enable, GPIO.LOW)
   
        
        print(self.name + " started")
   
    def moveStep(self, di, velocity):
    #        #direction ?
        if(di == -1):
            GPIO.output(self.direction, self.DIR_Left)
                                
        if(di == 1):               
            GPIO.output(self.direction, self.DIR_Right)                            
                 
             
            #pulse the motor with 
            GPIO.output(self.pulse, GPIO.HIGH)
            time.sleep(velocity)

            GPIO.output(self.pulse, GPIO.LOW)
            time.sleep(velocity)          
            
            #update the postion
            self.setPosition(self.getPosition() + di)
            
        else:
            self.isRunnig = False
            
        #if the motor made a 180° turn, reset the pos to zero    
        #self.check180()  
        
        
        
        
              
    def getPosition(self): 
        return self.position
    
    def setPosition(self, step):
        self.position = step
        
        
        #set the motor on 0 after a 360° Turn
    def check180(self):
        if(self.getPosition() >= self.steps or self.getPosition() <= -self.steps):
            self.setPosition(0)
             
    
    def getSpeed(self):
        return self.speed 
    
    def setSpeed(self, speed):
        minPercentage = 1  # Minimum percentage (1%)
        maxPercentage = 100  # Maximum percentage (100%)
    
    # Calculate the percentage based on the input value
        percentage = maxPercentage - (speed / 255.0) * (maxPercentage - minPercentage)
    # Calculate the speed delay based on the percentage        
        self.speed = self.minSpeed * (percentage / 100.0)
    
    def getDirection(self):
        return self.direction
    
    def setDirection(self):       
        self.direction        
        
        
    def setTarget(self, t):
        self.target = t             
    def getTarget(self):
        return self.target
    
    def programRotate(self):
        self.getDir()

    # Determine the range of the input value (0-255)
    # Determine the range of the output value (x to y)
         
    def dmxToVal(self , value, input_min  , input_max , output_min , output_max):       
    # Scale the value to the new range
        scaled_value = ((value - input_min) / (input_max - input_min)) * (output_max - output_min) + output_min
        return int(scaled_value)

    
    def readDmx(self):
        buffer = a.get_buffer(u1_listener)
        print(self.channelA)
        self.valA = buffer[(self.channelA - 1)]
        self.valB = int((buffer[self.channelB - 1 ] ))
        self.valC = int(buffer[self.channelC -1])
        
   
        if(self.valC > 254):
            
            print("Name: " , self.name , " CH1: " , self.valA , " CH2: " , self.valB , " Pos: " ,  self.getPosition())
        
  
        
    # MAIN LOOP FOR STEPPERS
    def run(self):
            
        
        while(True):
            
            #read outdmx values every 10th loop
            if(self.dmxCounter == 10):
                self.readDmx()
                self.dmxCounter = 0         
            self.dmxCounter += 1   
            
            
            #calculate the target
            self.setTarget(int(self.dmxToVal(self.valB , 0 , 255 , -self.steps , self.steps)))
            
            
            #if dmx channel val 1 is not 0 calc a new speed
            if (self.valA > 0):
                
                self.setSpeed(self.valA)
                #self.setSpeed(self.minSpeed)                
                #self.setSpeed(float(self.dmxToVal(self.valA , 0 , 255, 0.0 , 6000.0)))
                if(self.target  > self.getPosition()):
                    self.isRunning = True
                    self.moveStep(1 , self.speed)
                
                #step backwards
                if(self.target  < self.getPosition()):
                    self.isRunning = True
                    self.moveStep(-1 , self.speed)
         
            #if velocity is 0 or position is reached
                if(self.getTarget() == self.getPosition()):
                    print(self.name + " at target:" + self.getTarget())
                    self.isRunning = False
            #step forwards
            
                
            #print("Name: " ,self.name , " Pos: " , self.getPosition())
    
    
def main():

    

    #create Stepper Motor NAME PIN_S PIN_DIR ; EN ; 0 , minspeed, stepmode , dmx chan 1 2 3
    S1 =  Stepper("S1"  , 11,  7 , 3 , 0 , minSpeed , steps , 1 ,   2 ,   3)
    S2 =  Stepper("S2"  , 15, 13 , 3 , 0 , minSpeed , steps , 4 ,   5 ,   6)
    S3 =  Stepper("S3"  , 21, 19 , 3 , 0 , minSpeed , steps , 7 ,   8 ,   9)
    S4 =  Stepper("S4"  , 29, 23 , 3 , 0 , minSpeed , steps , 10 , 11 ,  12)
    S5 =  Stepper("S5"  , 33, 31 , 3 , 0 , minSpeed , steps , 13 , 14 ,  15)
    S6 =  Stepper("S6"  , 37, 35 , 3 , 0 , minSpeed , steps , 16 , 17 ,  18)
    
    # ~ S7  =  Stepper("S7"   , 0, 32 , 3 , 0 , 0 , 0 , 0 , 0)
    # ~ S8  =  Stepper("S8"   , 0, 0 , 3 , 0 , 0 , 0 , 0 , 0)
    # ~ S9  =  Stepper("S9"   , 0, 0 , 3 , 0 , 0 , 0 , 0 , 0)
    # ~ S10 =  Stepper("S10"  , 0, 0 , 3 , 0 , 0 , 0 , 0 , 0)
    # ~ S11 =  Stepper("S11"  , 31, 32 , 3 , 0 , 0 , 0 , 0 , 0)
    # ~ S12 =  Stepper("S12"  , 31, 32 , 3 , 0 , 0 , 0 , 0 , 0)
    
    
    steppers = [ S1 , S2 , S3 , S4 , S5 , S6 ]    
    threads = []
    
    
    try:
        #thread = threading.Thread(target=b.run)
        #thread.start()
        #threads.append(thread)
        for i in steppers:
            thread = threading.Thread(target=i.run)
            thread.start()
            threads.append(thread)
            
    except:
        print ("Error: unable to start thread")  
   
    
 
    print("finish")
if __name__ == "__main__":
    
   
    main()
    #loop = asyncio.get_event_loop()
    #loop.run_until_complete(start_receiver())
    #GPIO.cleanup()

