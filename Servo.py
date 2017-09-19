import math
from Adafruit_PWM_Servo_Driver import PWM

class Servo():
    def __init__(self):
        self.ADDRESSE = 0x40
        self.MIN_PULSE_LENGTH = 150
        self.MAX_PULSE_LENGTH = 600
        self.FREQUENCY = 60
        self.CLAMP_VALUES = [[self.MIN_PULSE_LENGTH, self.MAX_PULSE_LENGTH]] * 15
        self.pwm = PWM(self.ADDRESSE)
        
        print(self.CLAMP_VALUES)
        
    def setAddresse(self, _addresse):
        self.ADDRESSE = _addresse
        self.pwm = PWM(self.ADDRESSE)
    
    def setFrequency(self, _frequency):
        self.FREQUENCY = _frequency
        
    def setClampValuesOnChanel(self, _channel, _min_pulse_length, _max_pulse_length):
        self.CLAMP_VALUES[_channel] = [_min_pulse_length, _max_pulse_length]
        
    def set(self, _channel, _normalized_value):
        if _normalized_value > 1:
            _normalized_value = 1
        elif _normalized_value < 0:
            _normalized_value = 0
            
        self.pwm.setPWM(_channel, 0, int(round(self.CLAMP_VALUES[_channel][0] + (self.CLAMP_VALUES[_channel][1] - self.CLAMP_VALUES[_channel][0]) * _normalized_value)))
        