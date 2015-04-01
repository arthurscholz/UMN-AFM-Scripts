import picoscript
from time import sleep
import threading
import atexit

print __name__

limit = 0.87
speed = 1e-6

ServoRange = picoscript.GetServoTopographyRange()

def StepOpen():
    picoscript.SetMotorSpeed(speed)
    picoscript.SetMotorStepDistance(ServoRange*limit) 
    picoscript.MotorStepOpen()
    print "Stepped Open"

def StepClose():
    picoscript.SetMotorSpeed(speed)
    picoscript.SetMotorStepDistance(ServoRange*limit) 
    picoscript.MotorStepClose()
    print "Stepped Close"

class Track(threading.Thread):
    def run(self):
        print "Starting tracker..."
        while True:
            if(picoscript.GetServoActive() and picoscript.GetStatusApproachState() is 0):
                #print picoscript.GetServoZDirect()/ServoRange * 2
                if picoscript.GetServoZDirect() > limit * ServoRange / 2:
                    StepOpen()
                elif picoscript.GetServoZDirect() < -limit * ServoRange / 2:
                    StepClose()
            sleep(.1)
        
if __name__ == "__main__":
    atexit.register(picoscript.Disconnect)
    tracker = Track()
    tracker.start()
    tracker.join()
    