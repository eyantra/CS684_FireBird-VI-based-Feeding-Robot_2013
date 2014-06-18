import cv2, math
import numpy as np
import time

class ColourTracker:
    def __init__(self):
        cv2.namedWindow("ColourTrackerWindow", cv2.CV_WINDOW_AUTOSIZE)
        self.capture = cv2.VideoCapture(0)
        self.scale_down = 4
        
    def match(self,templet ):
        template = cv2.imread(templet )
        trows,tcols = template.shape[:2]
        f, img = self.capture.read()
        img2 = img.copy()
        matchvalue = 2
        res = {}
        meanx = 0
        meany = 0
        for matchvalue in range(6):
            result = cv2.matchTemplate(img,template,matchvalue)
            cv2.normalize(result,result,0,255,cv2.NORM_MINMAX)
            mini,maxi,(mx,my),(Mx,My) = cv2.minMaxLoc(result) # We find minimum and maximum value locations in result
            if matchvalue in [0,1]: # For SQDIFF and SQDIFF_NORMED, the best matches are lower values.
                MPx,MPy = mx,my
            else: # Other cases, best matches are higher values.
                MPx,MPy = Mx,My
            res[matchvalue] = (MPx,MPy)
            meanx = meanx + MPx
            meany = meany + MPy
        #print res
        sumsqurex = 0
        sumsqurey = 0
        for i in range(6):
            sumsqurex   = sumsqurex + (meanx - res[i][0]) * (meanx - res[i][0])
            sumsqurey   = sumsqurey + (meany - res[i][0]) * (meany - res[i][0])
        #print math.sqrt(sumsqurex /6),  math.sqrt(sumsqurey/6)
        if math.sqrt(sumsqurex /6) < 650 and math.sqrt(sumsqurey/6) < 650:
            return True
        else:
            return False
## Normed methods give better results, ie matchvalue = [1,3,5], others sometimes shows errors
#        cv2.rectangle(img2, (MPx,MPy),(MPx+tcols,MPy+trows),(0,0,255),2)
#        cv2.imshow('input',img2)
#        cv2.imshow('output',result)
#        if cv2.waitKey(20) == 27:
#            cv2.destroyWindow("ColourTrackerWindow")
#            self.capture.release()

if __name__ == "__main__":
    colour_tracker = ColourTracker()
    while True:
        colour_tracker.match('./template/t0.jpg')
        time.sleep(0.05)

