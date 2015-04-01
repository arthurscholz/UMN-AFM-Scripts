import numpy as np
import picoscript
import cv2
import HeightTracker
import atexit

print "Setting Parameters..."

zDacRange = 0.215 # Sensor specific number

windowSize = 3e-6 # window size in meters
windowBLHCX = 3.5e-6 # window bottom left hand corner X-axis in meters
windowBLHCY = 3.5e-6 # window bottom left hand corner Y-axis in meters
imageBuffer = 0 # buffer for tracking image (0-7)
binary = True

servoRange = picoscript.GetServoTopographyRange()
imageRange = servoRange * zDacRange

MAX_SHORT = 2**15

def PlaneLevel(im):
    x, y = np.indices(im.shape)
    A = np.vstack([x.flatten(), y.flatten(), np.ones(im.size)]).T
    z = im.flatten()
    a,b,c = np.linalg.lstsq(A, z)[0]
    plane = a*x + b*y + c
    return im - plane

def LineLevel(im):
    output = np.empty_like(im)
    for i in range(im.shape[0]):
        output[i] = im[i] - np.median(im[i])

    return output

def LoadBuffer(i):
    xSize = picoscript.GetScanXPixels()
    ySize = picoscript.GetScanYPixels()
    # Read in image data buffer
    im = np.asarray(picoscript.ReadImageDataBuffer(imageBuffer))
    im = im.reshape(ySize,xSize)
    im = im * imageRange / MAX_SHORT
    return im

# Calculates scan offset for new image. Takes an image, roi template and bottom
# left hand corner
def CalculateOffset(im,template,blhc):
    res = cv2.matchTemplate(im.astype('float32'),template.astype('float32'),cv2.TM_CCORR_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    
    # returns vector to feature
    return np.asarray(max_loc)-blhc
    
def Stabilize():
    position = picoscript.GetStatusApproachPosition()
    
    im = LoadBuffer(0)
    im = PlaneLevel(im)
    im = LineLevel(im)

    ref_image = im

    im_size = picoscript.GetScanSize()
    
    x0 = int(windowBLHCX / im_size * im.shape[1])
    y0 = int(windowBLHCY / im_size * im.shape[0])

    x1 = int((windowBLHCX + windowSize)/im_size * im.shape[1])
    y1 = int((windowBLHCY + windowSize)/im_size * im.shape[0])

    template = ref_image[y0:y1,x0:x1]

    while(position == picoscript.GetStatusApproachPosition()):
        picoscript.ScanStartDown()
        picoscript.WaitForStatusScanning(True)
        picoscript.WaitForStatusScanning(False)

        im = LoadBuffer(0)
        im = PlaneLevel(im)
        im = LineLevel(im)

        offset = CalculateOffset(im,template,np.array([x0,y0]))
        x_offset = picoscript.GetScanXOffset() + offset[0]*im_size/im.shape[1]
        y_offset = picoscript.GetScanYOffset() + offset[1]*im_size/im.shape[0]
        picoscript.SetScanXOffset(x_offset)
        picoscript.SetScanYOffset(y_offset)

        x0_new = offset[0] + x0
        x1_new = offset[0] + x1
        y0_new = offset[1] + y0
        y1_new = offset[1] + y1
        print '{0}, {1}'.format(x_offset, y_offset)

        template = im[y0_new:y1_new,x0_new:x1_new]
                             
if __name__  == "__main__":
    atexit.register(picoscript.Disconnect)
    heighttrack = HeightTracker.Track()
    heighttrack.start()
    RunStabilize = True
    
    print "Waiting for current scan to end..."
    picoscript.WaitForStatusScanning(False)
    
    print "Starting stabilization..."
    
    while True:
        if RunStabilize:
            Stabilize()
        position = picoscript.GetStatusApproachPosition()
        picoscript.ScanStartDown()
        picoscript.WaitForStatusScanning(True)
        picoscript.WaitForStatusScanning(False)
        RunStabilize = position == picoscript.GetStatusApproachPosition()
    