import threading

class Thread(threading.Thread):
    def __init__(self):
        super(Thread, self).__init__()
        self._stop_event = threading.Event()
        self.RUN_FUNCTION = None
        
    def stop(self):
        self._stop_event.set()
        
    def stopped(self):
        return self._stop_event.is_set()
    
    def setRunFunction(self, _run_function):
        self.RUN_FUNCTION = _run_function
    
    def run(self):
        if self.RUN_FUNCTION is not None:
            self.RUN_FUNCTION()