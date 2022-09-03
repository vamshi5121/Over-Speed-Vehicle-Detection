import cv2
import dlib
import time
import threading
import math
from flask import Flask,render_template,request
import easyocr

#Initialization
site = Flask(__name__)

@site.route('/')
def homepage():
    return render_template('homepage.html')

@site.route('/', methods = ['POST'])
def getvalue():
	video = request.form['video']
	spd = request.form['speedlimit']
	platelist = trackMultipleObjects(video,spd)
	return render_template('pass.html',list = platelist,s = spd)

def estimateSpeed(location1, location2):
	d_pixels = math.sqrt(math.pow(location2[0] - location1[0], 2) + math.pow(location2[1] - location1[1], 2))
	ppm = 8.8
	d_meters = d_pixels / ppm
	fps = 18
	speed = d_meters * fps * 3.6
	return speed

def getID(carLocation1,xp,yp):
    for key,value in carLocation1.items():
        if value[0] < xp and value[0]+value[2] > xp:
            return key

def trackMultipleObjects(video,speedLimit):
    carCascade = cv2.CascadeClassifier('myhaar.xml')

    video = cv2.VideoCapture(str(video))

    carPlatesCascade = cv2.CascadeClassifier('licenseplate.xml')

    #WIDTH = 620
    #HEIGHT = 820

    WIDTH = 720
    HEIGHT = 1240
    
    rectangleColor = (0, 255, 0)
    frameCounter = 0
    currentCarID = 0
    carp = 0
    carCount=0
    fps = 0
    speedLimit=int(speedLimit)
    carTracker = {}
    carplatesTracker = {}
    carNumbers = {}
    carplateloc = {}
    platelist = []
    carLocation1 = {}
    carLocation2 = {}
    speed = [None] * 1000
    spdcar = [None] * 1000
    text = [None] * 1000

    while True:
        start_time = time.time()
        rc, image = video.read()
        if type(image) == type(None):
            break
        image = cv2.resize(image, (WIDTH, HEIGHT))
        resultImage = image.copy()
        frameCounter = frameCounter + 1
        carIDtoDelete = []
        carplatetoDelete = []
        for plateID in carplatesTracker.keys():
            tQuality = carplatesTracker[plateID].update(image)
            if tQuality < 7:
                carplatetoDelete.append(plateID)
        for carID in carTracker.keys():
            trackingQuality = carTracker[carID].update(image)
            if trackingQuality < 7:
                carIDtoDelete.append(carID)

        for plateID in carplatetoDelete:
            carplatesTracker.pop(plateID,None)
            carp = carp-1;
        for carID in carIDtoDelete:
            carTracker.pop(carID, None)
            carCount = carCount-1
            carLocation1.pop(carID, None)
            carLocation2.pop(carID, None)

        if not (frameCounter % 10):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cars = carCascade.detectMultiScale(gray, 1.1, 13, 18, (24, 24))

            car_plates = carPlatesCascade.detectMultiScale(gray,scaleFactor=1.2,minNeighbors = 5, minSize=(25,25))

            for (xp,yp,wp,hp) in car_plates:
                xp_bar = xp + 0.5 * wp
                yp_bar = yp + 0.5 * hp
                cv2.rectangle(resultImage,(xp,yp),(xp+wp,yp+hp),(0,0,255),2)
                plate = image[yp: yp+hp, xp:xp+wp]

                match = None
                for plateID in carplatesTracker.keys():
                    trackedP = carplatesTracker[plateID].get_position()
                    t_xp = int(trackedP.left())
                    t_yp = int(trackedP.top())
                    t_wp = int(trackedP.width())
                    t_hp = int(trackedP.height())
                    t_xp_bar = t_xp + 0.5 * t_wp
                    t_yp_bar = t_yp + 0.5 * t_hp

                    if ((t_xp <= xp_bar <= (t_xp + t_wp)) and (t_yp <= yp_bar <= (t_yp + t_hp)) and (xp <= t_xp_bar <= (xp + wp)) and (yp <= t_yp_bar <= (yp + hp))):
                        match = plateID

                if match is None:
                    trackp = dlib.correlation_tracker()
                    trackp.start_track(image, dlib.rectangle(xp, yp, xp + wp, yp + hp))
                    currentplateID =  getID(carLocation1,xp,yp)
                    carplatesTracker[currentplateID] = trackp
                    carp = carp + 1
                    carplateloc[currentplateID] = [xp, yp, wp, hp]

            for (_x, _y, _w, _h) in cars:
                x = int(_x)
                y = int(_y)
                w = int(_w)
                h = int(_h)
                x_bar = x + 0.5 * w
                y_bar = y + 0.5 * h

                matchCarID = None

                for carID in carTracker.keys():
                    trackedPosition = carTracker[carID].get_position()
                    t_x = int(trackedPosition.left())
                    t_y = int(trackedPosition.top())
                    t_w = int(trackedPosition.width())
                    t_h = int(trackedPosition.height())
                    t_x_bar = t_x + 0.5 * t_w
                    t_y_bar = t_y + 0.5 * t_h

                    if ((t_x <= x_bar <= (t_x + t_w)) and (t_y <= y_bar <= (t_y + t_h)) and (x <= t_x_bar <= (x + w)) and (y <= t_y_bar <= (y + h))):
                        matchCarID = carID

                if matchCarID is None:
                    tracker = dlib.correlation_tracker()
                    tracker.start_track(image, dlib.rectangle(x, y, x + w, y + h))
                    carTracker[currentCarID] = tracker
                    carCount = carCount + 1
                    carLocation1[currentCarID] = [x, y, w, h]
                    currentCarID = currentCarID + 1

        for carID in carTracker.keys():
            trackedPosition = carTracker[carID].get_position()
            t_x = int(trackedPosition.left())
            t_y = int(trackedPosition.top())
            t_w = int(trackedPosition.width())
            t_h = int(trackedPosition.height())
            cv2.rectangle(resultImage, (t_x, t_y), (t_x + t_w, t_y + t_h), rectangleColor, 4)
            carLocation2[carID] = [t_x, t_y, t_w, t_h]

        for plateID in carplatesTracker.keys():
            trackedP = carplatesTracker[plateID].get_position()
            t_xp = int(trackedP.left())
            t_yp = int(trackedP.top())
            t_wp = int(trackedP.width())
            t_hp = int(trackedP.height())
            cv2.rectangle(resultImage, (t_xp, t_yp), (t_xp + t_wp, t_yp + t_hp), rectangleColor, 4)

        end_time = time.time()

        if not (end_time == start_time):
            fps = 1.0/(end_time - start_time)

        for i in carplateloc.keys():
            if frameCounter % 1 == 0:
                [xp1,yp1,wp1,hp1] = carplateloc[i]
                if (text[i] == None or text[i] == 0) and yp1>=270:
                    reader = easyocr.Reader(['en'])
                    text[i] = reader.readtext(plate)

                    if len(text[i]) ==0:
                        continue
                    print(text[i][0][1][1:])
                    if speed[i]> speedLimit:
                        platelist.append(text[i][0][1][1:])


        for i in carLocation1.keys():
            if frameCounter % 1 == 0:
                [x1, y1, w1, h1] = carLocation1[i]
                [x2, y2, w2, h2] = carLocation2[i]
                carLocation1[i] = [x2, y2, w2, h2]
                if [x1, y1, w1, h1] != [x2, y2, w2, h2]:
                    if (speed[i] == None or speed[i] == 0) and y1 >= 0 and y1 <= 285:
                        speed[i] = estimateSpeed([x1, y1, w1, h1], [x2, y2, w2, h2])

                    if speed[i] != None and y1 >= 180:
                        cv2.putText(resultImage, str(int(speed[i])) + " km/hr", (int(x1 + w1/2), int(y1-5)),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
                        if speed[i]>speedLimit:
                            spdcar[i]=1
                            cv2.rectangle(resultImage, (x2,y2), (x2 + w1,y2 + h2),(0,0,255), 4)

                spdcount = 0
                for i in range(len(spdcar)):
                    if(spdcar[i] == 1):
                        spdcount = spdcount + 1
                cv2.putText(resultImage,"Total Number of Cars : " + str(int(carCount)),(int(50),int(50)),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
                cv2.putText(resultImage,"Total Number of OverSpeed Vehicles : " + str(int(spdcount)),(int(50),int(75)),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
                cv2.putText(resultImage,"SPEEDLIMIT : " + str(int(speedLimit)) + "km/hr",(int(50),int(100)),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

        cv2.imshow('result', resultImage)

        if cv2.waitKey(33) == 27:
            break

    cv2.destroyAllWindows()
    return platelist






if __name__ == '__main__':
    site.run(debug=True)
