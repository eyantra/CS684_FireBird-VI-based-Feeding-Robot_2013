'''This file contains all fuctions related to image porcessing operations'''

import cv2, math
import numpy as np
import time
class ColourTracker:
    def __init__(self, cameraPort):
        cv2.namedWindow("ColourTrackerWindow", cv2.CV_WINDOW_AUTOSIZE)
        self.capture = cv2.VideoCapture(cameraPort)
        self.scale_down = 4

    def run(self):
        while True:
            f, orig_img = self.capture.read()
            orig_img = cv2.flip(orig_img, 1)
            img = cv2.GaussianBlur(orig_img, (5,5), 0)
            img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2HSV)
            img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
            red_lower = np.array([0, 150, 0],np.uint8)
            red_upper = np.array([5, 255, 255],np.uint8)
            red_binary = cv2.inRange(img, red_lower, red_upper)
            #red_lower = np.array([130, 150, 0],np.uint8)
            #red_upper = np.array([135   , 255, 255],np.uint8)
            #red_binary = cv2.inRange(img, red_lower, red_upper)            
            dilation = np.ones((15, 15), "uint8")
            red_binary = cv2.dilate(red_binary, dilation)
            contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            largest_contour = None
            for idx, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    largest_contour = contour
                if not largest_contour == None:
                    moment = cv2.moments(largest_contour)
                    if moment["m00"] > 1000 / self.scale_down:
                        rect = cv2.minAreaRect(largest_contour)
                        rect = ((rect[0][0] * self.scale_down, rect[0][1] * self.scale_down), (rect[1][0] * self.scale_down, rect[1][1] * self.scale_down), rect[2])
                        box = cv2.cv.BoxPoints(rect)
                        box = np.int0(box)
                        cv2.drawContours(orig_img,[box], 0, (0, 0, 255), 2)
                        cv2.imshow("ColourTrackerWindow", orig_img)
                        if cv2.waitKey(20) == 27:
                            cv2.destroyWindow("ColourTrackerWindow")
                            self.capture.release()
                            break

    def show(self): #show a live window 
        box = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]
        f, orig_img = self.capture.read()
        orig_img = cv2.flip(orig_img, 1)
        img = cv2.GaussianBlur(orig_img, (5,5), 0)
        img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2HSV)
        img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
        red_lower = np.array([0, 150, 0],np.uint8)
        red_upper = np.array([10, 255, 255],np.uint8)
        red_binary = cv2.inRange(img, red_lower, red_upper)
        #red_lower = np.array([130, 150, 0],np.uint8)
        #red_upper = np.array([135   , 255, 255],np.uint8)
        #red_binary = cv2.inRange(img, red_lower, red_upper)            
        dilation = np.ones((15, 15), "uint8")
        red_binary = cv2.dilate(red_binary, dilation)
        contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        largest_contour = None
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                largest_contour = contour
            if not largest_contour == None:
                moment = cv2.moments(largest_contour)
                if moment["m00"] > 1000 / self.scale_down:
                    rect = cv2.minAreaRect(largest_contour)
                    rect = ((rect[0][0] * self.scale_down, rect[0][1] * self.scale_down), (rect[1][0] * self.scale_down, rect[1][1] * self.scale_down), rect[2])
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(orig_img,[box], 0, (0, 0, 255), 2)
                    cv2.imshow("ColourTrackerWindow", orig_img)
                    if cv2.waitKey(20) == 27:
                        cv2.destroyWindow("ColourTrackerWindow")
                        self.capture.release()
                        break
        if largest_contour == None:
            cv2.imshow("ColourTrackerWindow", orig_img)
            if cv2.waitKey(20) == 27:
                cv2.destroyWindow("ColourTrackerWindow")
                self.capture.release()
                return (box[0][0] + box[2][0] )/2
        return (box[0][0] + box[2][0] )/2

    def show2(self):
        box = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]
        f, orig_img = self.capture.read()
        orig_img = cv2.flip(orig_img, 1)
        img = cv2.GaussianBlur(orig_img, (5,5), 0)
        img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2HSV)
        img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
        red_lower = np.array([335, 150, 70],np.uint8)
        red_upper = np.array([360, 255, 255],np.uint8)
        red_binary = cv2.inRange(img, red_lower, red_upper)
        #red_lower = np.array([130, 150, 0],np.uint8)
        #red_upper = np.array([135   , 255, 255],np.uint8)
        #red_binary = cv2.inRange(img, red_lower, red_upper)            
        dilation = np.ones((15, 15), "uint8")
        red_binary = cv2.dilate(red_binary, dilation)
        contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        largest_contour = None
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                largest_contour = contour
            if not largest_contour == None:
                moment = cv2.moments(largest_contour)
                if moment["m00"] > 1000 / self.scale_down:
                    rect = cv2.minAreaRect(largest_contour)
                    rect = ((rect[0][0] * self.scale_down, rect[0][1] * self.scale_down), (rect[1][0] * self.scale_down, rect[1][1] * self.scale_down), rect[2])
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(orig_img,[box], 0, (0, 0, 255), 2)
                    cv2.imshow("ColourTrackerWindow", orig_img)
                    if cv2.waitKey(20) == 27:
                        cv2.destroyWindow("ColourTrackerWindow")
                        self.capture.release()
                        break
        if largest_contour == None:
            cv2.imshow("ColourTrackerWindow", orig_img)
            if cv2.waitKey(20) == 27:
                cv2.destroyWindow("ColourTrackerWindow")
                self.capture.release()
                return (box[0][0] + box[2][0] )/2
        return (box[0][0] + box[2][0] )/2

#---
    def show3(self):
        box = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]
        f, orig_img = self.capture.read()
        orig_img = cv2.flip(orig_img, 1)
        img = cv2.GaussianBlur(orig_img, (5,5), 0)
        img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2HSV)
        img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
        red_lower = np.array([100, 100, 0],np.uint8)
        red_upper = np.array([200, 255, 255],np.uint8)
        red_binary = cv2.inRange(img, red_lower, red_upper)
        #red_lower = np.array([130, 150, 0],np.uint8)
        #red_upper = np.array([135   , 255, 255],np.uint8)
        #red_binary = cv2.inRange(img, red_lower, red_upper)            
        dilation = np.ones((15, 15), "uint8")
        red_binary = cv2.dilate(red_binary, dilation)
        contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        largest_contour = None
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                largest_contour = contour
            if not largest_contour == None:
                moment = cv2.moments(largest_contour)
                if moment["m00"] > 1000 / self.scale_down:
                    rect = cv2.minAreaRect(largest_contour)
                    rect = ((rect[0][0] * self.scale_down, rect[0][1] * self.scale_down), (rect[1][0] * self.scale_down, rect[1][1] * self.scale_down), rect[2])
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(orig_img,[box], 0, (0, 0, 255), 2)
                    cv2.imshow("ColourTrackerWindow", orig_img)
                    if cv2.waitKey(20) == 27:
                        cv2.destroyWindow("ColourTrackerWindow")
                        self.capture.release()
                        break
        if largest_contour == None:
            cv2.imshow("ColourTrackerWindow", orig_img)
            if cv2.waitKey(20) == 27:
                cv2.destroyWindow("ColourTrackerWindow")
                self.capture.release()
                return (box[0][0] + box[2][0] )/2
        return (box[0][0] + box[2][0] )/2

#---
    def findColor(self):
        box = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]
        f, orig_img = self.capture.read()
        orig_img = cv2.flip(orig_img, 1)
        img = cv2.GaussianBlur(orig_img, (5,5), 0)
        img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2HSV)
        img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
        red_lower = np.array([0, 150, 0],np.uint8)
        red_upper = np.array([10, 255, 255],np.uint8)
        red_binary = cv2.inRange(img, red_lower, red_upper)
        dilation = np.ones((15, 15), "uint8")
        red_binary = cv2.dilate(red_binary, dilation)
        contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        largest_contour = None
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                largest_contour = contour
            if not largest_contour == None:
                moment = cv2.moments(largest_contour)
                if moment["m00"] > 1000 / self.scale_down:
                    rect = cv2.minAreaRect(largest_contour)
                    rect = ((rect[0][0] * self.scale_down, rect[0][1] * self.scale_down), (rect[1][0] * self.scale_down, rect[1][1] * self.scale_down), rect[2])
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
        if largest_contour == None:
                return (box[0][0] + box[2][0] )/2
        return (box[0][0] + box[2][0] )/2

    def findColor2(self):
        box = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]
        f, orig_img = self.capture.read()
        orig_img = cv2.flip(orig_img, 1)
        img = cv2.GaussianBlur(orig_img, (5,5), 0)
        img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2HSV)
        img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
        red_lower = np.array([330, 150, 0],np.uint8)
        red_upper = np.array([350, 255, 255],np.uint8)
        red_binary = cv2.inRange(img, red_lower, red_upper)
        dilation = np.ones((15, 15), "uint8")
        red_binary = cv2.dilate(red_binary, dilation)
        contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        largest_contour = None
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                largest_contour = contour
            if not largest_contour == None:
                moment = cv2.moments(largest_contour)
                if moment["m00"] > 1000 / self.scale_down:
                    rect = cv2.minAreaRect(largest_contour)
                    rect = ((rect[0][0] * self.scale_down, rect[0][1] * self.scale_down), (rect[1][0] * self.scale_down, rect[1][1] * self.scale_down), rect[2])
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
        if largest_contour == None:
                return (box[0][0] + box[2][0] )/2
        return (box[0][0] + box[2][0] )/2

      
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
    
    def saveImage(self, filename):
        ramp_frames = 30
        for i in xrange(ramp_frames):
            f, orig_img = self.capture.read()
#        file = "/home/rohit/python/store/test1_image.png"
        cv2.imwrite(filename, orig_img)

if __name__ == "__main__":
    colour_tracker = ColourTracker()
    while True:
        print (colour_tracker.show2())
        time.sleep(0.05)

	

