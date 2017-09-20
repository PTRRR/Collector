# Collector script
# This script controls the robots
# To run -> cmd: source ~/.profile, workon cv

import time, math, os, glob
import threading
import cv2
import sys
import imutils
import random
import numpy
from Thread import Thread
from picamera import PiCamera
from picamera.array import PiRGBArray
from Stepper import Stepper
from Servo import Servo
from FaceDetector import FaceDetector

# Init HACK
#!/usr/bin/python
from Adafruit_PWM_Servo_Driver import PWM

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz
for i in range(0, 2):
  # Change speed of continuous servo on channel O
  pwm.setPWM(0, 0, servoMin)
  time.sleep(1)
  pwm.setPWM(0, 0, servoMax)
  time.sleep(1)

# Tail (stepper motor)
STEPPER = Stepper()
STEPPER.setAutoDisable(False)
STEPPER.setMicrostepping(16)
STEPPER.setGearRatio(5.18)
stepper_index = 0

# Head (servo motors)
SERVO = Servo()
#SERVO.setClampPulseLength(145, 490)
servo_offset_x = random.random() * 1000
servo_index_x = 0.1
stepper_target = 0

# Face detection
# Get cascades pathes
basePath = "/home/pi/Desktop/FaceTracking/FaceTracking_test_1/"
#cascadeFilesPaths = ["HS.xml", "haarcascade_frontalface_default.xml", "haarcascade_profileface.xml", "haarcascade_eye.xml", "Mouth.xml"]
cascadeFilesPaths = ["HS.xml", "haarcascade_frontalface_default.xml"]

# Create multiple haar cascade to detect a maximum of facial features
# We want to bo sure that the script detects a person if there is one
CASCADES = [None] * len(cascadeFilesPaths)

fileIndex = 0;
for file in cascadeFilesPaths:
    CASCADES[fileIndex] = cv2.CascadeClassifier(basePath + file)
    fileIndex += 1

FACES = None
CURRENT_FACE = None
FACE_DETECTOR = None

def face_detection():
    global FACES, CASCADES
    
    detected_faces = []
    for cascade in CASCADES:
        features = cascade.detectMultiScale(
            gray,
            scaleFactor = 1.1,
            minNeighbors = 5,
            minSize = (30, 30),
            flags = cv2.CASCADE_SCALE_IMAGE
        )
        
        # Reset faces
        num_faces = len(features)
        
        if num_faces > 0:
            for (x, y, w, h) in features:
                detected_faces.append([x, y, w, h])
            break
        
    # Override faces
    FACES = detected_faces
    
def photo_thread():
    global CAMERA, SYNC_FOLDER_PATH
    CAMERA.capture(SYNC_FOLDER_PATH + str(int(math.floor(time.time() * 1000))) + '.jpg')

# Motors control loop
def control_loop():
    global STEPPER, SERVO, CURRENT_FACE
    global servo_index_x, servo_offset_x, stepper_target
    index = 0
    stepper_index = 0
    servo_target = 0
    
    try:
        while CONTROL_THREAD.stopped() is not True:
            
            #SERVO.set(0, 0.5 - math.sin(index) * 0.1)
            #SERVO.set(0, 0.5)
            stepper_index += 0.001
            
            # Check if there is a current face to track
            if CURRENT_FACE is not None:
                face_pos_x = CURRENT_FACE[0] + CURRENT_FACE[2] / 2
                delta_x = (face_pos_x - TRACKING_SCREEN[0] * 0.5) / (TRACKING_SCREEN[0] * 0.5)
                servo_index_x += delta_x * -0.002
        
                STEPPER.enable(True)
                face_pos_y = CURRENT_FACE[1] + CURRENT_FACE[3] / 2
                delta_y = (face_pos_y - TRACKING_SCREEN[1] * 0.5) / (TRACKING_SCREEN[1] * 0.5)
                STEPPER.setVelocity(delta_y * 3)
            else:
                STEPPER.enable(False)
                index += 0.001
                servo_target = 0.5 - math.sin(index) * 0.3
                servo_index_x += (servo_target - servo_index_x) * 0.005
                
            
            SERVO.set(0, servo_index_x)
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Control thread ended!")
    
# Init script            
STEPPER.start()
STEPPER.enable(False)

CONTROL_THREAD = Thread()
CONTROL_THREAD.setRunFunction(control_loop)
CONTROL_THREAD.start()

# Global variables
CURRENT_TIME = time.time()
DELAY_IMAGES = 5
NEXT_TIME = CURRENT_TIME + DELAY_IMAGES

# Camera
IMAGE = None
DEFINITION_MULTIPLIER = 1
CAMERA = PiCamera()
CAMERA.resolution = (int(1024 * 1.5), int(768 * 1.5))
CAMERA.framerate = 32
TRACKING_SCREEN = [int(160 * DEFINITION_MULTIPLIER), int(120 * DEFINITION_MULTIPLIER)]
CAPTURE = PiRGBArray(CAMERA, size = (int(160 * DEFINITION_MULTIPLIER), int(120 * DEFINITION_MULTIPLIER)))
DELAY_DELETE_IMAGES = 60 * 2 # seconds -> 2min
LOW_RES_STREAM = CAMERA.capture_continuous(CAPTURE, format = "bgr", use_video_port = True, splitter_port = 2, resize = (int(160 * DEFINITION_MULTIPLIER), int(120 * DEFINITION_MULTIPLIER)))

time.sleep(2.0)
print("done warming up")

# Remove all old images
SYNC_FOLDER_PATH = "/home/pi/Desktop/RPI_3_sync/"
file_list = glob.glob(SYNC_FOLDER_PATH + "*.jpg")
for image in file_list:
    os.remove(image)

# Run the min loop
run = True
try:
    while run:
        # Update current time
        CURRENT_TIME = time.time()
      
        # Take pictures every DELAY_IMAGES
        if CURRENT_TIME >= NEXT_TIME:
            file_list = glob.glob(SYNC_FOLDER_PATH + "*.jpg")
            #print("Image taken: " + str(len(file_list)))
            for image in file_list:
                name = float(image.split("/")[len(image.split("/")) - 1].replace(' ', '')[:-4].upper())
                # Delete images that are older than the DELAY_DELETE_IMAGES
                if name < CURRENT_TIME - DELAY_DELETE_IMAGES:
                    os.remove(image)
            
            NEXT_TIME += DELAY_IMAGES
            if FACES is not None:
                if len(FACES) > 0:
                    #print("photo")
                    CAMERA.capture(SYNC_FOLDER_PATH + str(int(math.floor(time.time() * 1000))) + '.jpg')
        
        low_res = LOW_RES_STREAM.next()
        IMAGE = low_res.array
        gray = cv2.cvtColor(IMAGE, cv2.COLOR_BGR2GRAY)
        
        # Run face detection as soon as one has ended
        if FACE_DETECTOR is None:
            FACE_DETECTOR = Thread()
            FACE_DETECTOR.setRunFunction(face_detection)
            FACE_DETECTOR.start()
        else:
            if FACE_DETECTOR.isAlive() is not True:
                FACE_DETECTOR = None
                
        if FACES is not None:
            if len(FACES) > 0:
                if CURRENT_FACE is None:
                    CURRENT_FACE = FACES[0]
                    
                # Always check for the closer face to the last one
                # in order not to jump from a face to another
                min_distance = float("inf")
                for face in FACES:
                    distance = math.sqrt(math.pow(CURRENT_FACE[0] - face[0], 2) + math.pow(CURRENT_FACE[1] - face[1], 2))
                    if distance < min_distance:
                        min_distance = distance
                        CURRENT_FACE = face
                        
                #print(CURRENT_FACE)
                cv2.circle(IMAGE, (CURRENT_FACE[0] + CURRENT_FACE[2] / 2, CURRENT_FACE[1] + CURRENT_FACE[3] / 2), int(CURRENT_FACE[2] / 3), (255, 255, 255), 1)
            else:
                CURRENT_FACE = None
        
        
        CAPTURE.truncate(0)
        #cv2.imshow("Frame", IMAGE)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            STEPPER.stop()
            STEPPER.enable(False)
            CONTROL_THREAD.stop()
            run = False
            break
        elif key == ord("r"):
            servo_index_x = 0.45

except KeyboardInterrupt:
    STEPPER.stop()
    STEPPER.enable(False)
    CONTROL_THREAD.stop()
    run = False