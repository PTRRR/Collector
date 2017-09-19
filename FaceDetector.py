import time
import cv2
import threading

class FaceDetector(threading.Thread):
    def __init__(self):
        super(FaceDetector, self).__init__()
        self._stop_event = threading.Event()
        
    def stop(self):
        self._stop_event.set()
        
    def stopped(self):
        return self._stop_event.is_set()
    
    def run(self):
        print("run")

    


     
        