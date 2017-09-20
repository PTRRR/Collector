import time, math
import threading
from RPi import GPIO

# Step thread to send pulses to the stepper asyncronousely an order to make smooth accelerations

class StepThread(threading.Thread):
    def __init__(self):
        super(StepThread, self).__init__()
        self._stop_event = threading.Event()
    
    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
    
    def setStepFunc(self, _func):
        self.stepFunc = _func
    
    def run(self):
        try:
            while self.stopped() is not True:
                self.stepFunc()
                
            print("Stepper stopped")
                
        except KeyboardInterrupt:
            print("Stepper stopped")

# Main stepper class

class Stepper():
    
    def __init__(self, stepPin = 17, directionPin = 4, enablePin = 25, microStepping = 4, stepDelay = 0.001):
        
        self.DEFAULT_FULL_TURN = 200
        self.GEAR_RATIO = 1
        self.MICROSTEPPING = microStepping
        
        self.MIN_DELAY = 0.0001
        self.MAX_DELAY = 0.05
        self.DELAY = self.MIN_DELAY + (self.MAX_DELAY - self.MIN_DELAY) * 0.1
        
        self.AUTO_DISABLE = True
        self.DISABLE_THRESHOLD = 0.05
        self.ENABLED = True
        
        self.STEP_INDEX = 0
        
        self.DIRECTION = 1
        self.PINS = {
        
            "step": stepPin,
            "dir": directionPin,
            "enable": enablePin,
            "ms1": 24,
            "ms2": 23,
            "ms3": 22
        
        }
        
        self.step_thread = StepThread()
        self.step_thread.setStepFunc(self.moveOneStep)
        
        self.accelerationSteps = 0
        
        # Setup PINS
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        for pin in self.PINS:
            GPIO.setup(self.PINS[pin], GPIO.OUT)
            
    # Utils functions
    def setMicrosteppingPins(self, _ms1, _ms2, _ms3):
        self.PINS["ms1"] = _ms1
        self.PINS["ms2"] = _ms2
        self.PINS["ms3"] = _ms3
        
    def setMicrostepping(self, _microstepping):
        if _microstepping == 1:
            GPIO.output(self.PINS["ms1"], GPIO.LOW)
            GPIO.output(self.PINS["ms2"], GPIO.LOW)
            GPIO.output(self.PINS["ms3"], GPIO.LOW)
        elif _microstepping == 4:
            GPIO.output(self.PINS["ms1"], GPIO.HIGH)
            GPIO.output(self.PINS["ms2"], GPIO.LOW)
            GPIO.output(self.PINS["ms3"], GPIO.LOW)
        elif _microstepping == 8:
            GPIO.output(self.PINS["ms1"], GPIO.HIGH)
            GPIO.output(self.PINS["ms2"], GPIO.HIGH)
            GPIO.output(self.PINS["ms3"], GPIO.LOW)
        elif _microstepping == 16:
            GPIO.output(self.PINS["ms1"], GPIO.HIGH)
            GPIO.output(self.PINS["ms2"], GPIO.HIGH)
            GPIO.output(self.PINS["ms3"], GPIO.HIGH)
            
        self.MICROSTEPPING = _microstepping
          
    def setGearRatio(self, _gearRatio):
        self.GEAR_RATIO = _gearRatio
        
    def getFullTurn(self):
        return self.MICROSTEPPING * self.GEAR_RATIO * self.DEFAULT_FULL_TURN
    
    def getStepIndex(self):
        return self.STEP_INDEX
    
    def getAngle(self):
        one_step_in_deg = 360.0 / self.getFullTurn()
        return self.getStepIndex() * one_step_in_deg
    
    def resetCount(self):
        self.STEP_INDEX = 0
        
    def setAutoDisable(self, _auto_disable):
        self.AUTO_DISABLE = _auto_disable
    
    # This functions triggers the stepping thread
    def start(self):
        self.step_thread.start()
        
    # This function stops the stepping thread
    def stop(self):
        self.step_thread.stop()
        
    def setDelayClampValues(self, _min_delay, _max_delay):
        self.MIN_DELAY = _min_delay
        self.MAX_DELAY = _max_delay
        
    def isEnabled(self):
        return self.ENABLED
        
    # Beta
    def setAccelerationSteps(self, _accelerationSteps):
        self.accelerationSteps = _accelerationSteps
        
    def waitDelay(self):
        time.sleep(self.DELAY)
        
    # Always use this function to move one step
    # This will keep track of the step count
    def moveOneStep(self):
        GPIO.output(self.PINS["step"], GPIO.LOW)
        self.waitDelay()
        GPIO.output(self.PINS["step"], GPIO.HIGH)
        self.waitDelay()
        
        self.STEP_INDEX += self.DIRECTION
        
    def setDirection(self, _direction):
        if math.copysign(1, _direction) >= 0:
            self.DIRECTION = 1
            GPIO.output(self.PINS["dir"], GPIO.HIGH)
        else:
            self.DIRECTION = -1
            GPIO.output(self.PINS["dir"], GPIO.LOW)
        
    def setVelocity(self, _velocity):
        if math.copysign(1, _velocity) >= 0:
            self.setDirection(1)
        else:
            self.setDirection(-1)
            
        _velocity = max(-1, min(1, _velocity))
        
        if abs(_velocity) < self.DISABLE_THRESHOLD and self.AUTO_DISABLE:
            self.enable(False)
        else:
            self.enable(True)
            
        self.DELAY = self.MIN_DELAY + (self.MAX_DELAY - self.MIN_DELAY) * (1 - abs(_velocity))
            
    def enable(self, _enable):
        if _enable:
            self.ENABLED = True
            GPIO.output(self.PINS["enable"], GPIO.LOW)
        else:
            self.ENABLED = False
            GPIO.output(self.PINS["enable"], GPIO.HIGH)