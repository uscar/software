import numpy as np
import cv2
import cv2.cv as cv
import scipy as sp
from matplotlib import pyplot as plt
import time
import math
from operator import itemgetter, attrgetter, methodcaller
from numpy.linalg import inv

#sensor data
height = 1
fieldOfView = 5*math.pi/18

#creating hsv mask for original image
lower_white = np.array([0,0,50])
lower_white2 = np.array([0,0,100])
higher_white = np.array([179,50,255])

#loading the image
cap = cv2.VideoCapture(0)
lower_green = (55,245,245)
upper_green = (65,255,255)

#intersection function
def intersect(p1,p2):
  mat = np.matrix(((math.cos(p1[1]),-math.sin(p1[1])),(math.cos(p2[1]),-math.sin(p2[1]))))
  mat = inv(mat)
  dis = np.matrix((p1[0],p2[0]))
  dis = dis.transpose()
  result = np.dot(mat,dis)
  return result

#distance between p1 and p2, p2[1] is a negative number
def distance(p1,p2):
  dist = math.sqrt(pow((p1[0]-p2[0]),2)+pow(p1[1]+p2[1],2))
  return dist
index = 0;
while(True):
    #calculate range of circle
    radius = int(((0.53*800)/(2*height*math.tan(fieldOfView/2)))/2)
    # Capture frame-by-frame
    ret, frame = cap.read()
    #frame = cv2.imread('roombafar.jpg')
    if ret == True:

      frame = cv2.resize(frame,(800,600))
      #masking
      gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
      gray = cv2.GaussianBlur(gray,(9,9),0)
      circles = cv2.HoughCircles(gray,cv.CV_HOUGH_GRADIENT,1,20,
                             param1=120,param2=50,minRadius=int(radius*0.85),maxRadius=int(radius*1.15))

      #h2, w2 = frame.shape[:2]
      view = frame.copy()
      temp = frame.copy()
      if circles != None:
          circles = np.uint16(np.around(circles))
          count = 0;
          #print circles
          for i in circles[0,:]:
             # draw the outer circle
             #cv2.circle(temp,(i[0],i[1]),i[2],(0,255,0),-1)
              up = i[1]-i[2]
              if up > 10000:
                up = 0;

              cv2.circle(view,(i[0],i[1]),i[2],(0,255,0),3)
              cv2.rectangle(temp,(i[0]-i[2],up),(i[0]+i[2],i[1]+i[2]),(0,255,0),-1)
              #cv2.imshow('rect',temp)
              count = count + 1
              if count == 10:
                 break
            # draw the center of the circle
          cv2.circle(view,(i[0],i[1]),2,(0,0,255),3)
          hsv = cv2.cvtColor(temp,cv2.COLOR_BGR2HSV)
          mask = cv2.inRange(hsv,lower_green,upper_green)
          result = cv2.bitwise_and(frame,frame,mask= mask)
          
          #detecting rotation of the roomba
          #cv2.imshow('result',result)
          result = cv2.medianBlur(result,5)
          gray = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
          edges = cv2.Canny(gray,50,150,apertureSize = 3)
          #cv2.imshow("canny",edges)
          lines = cv2.HoughLines(edges,1,np.pi/180,80)
          good = []
          temp = []
          arr1 = []
          arr2 = []
          hight, width, channel = result.shape
          if lines != None:
            for m,n in lines[0]:
                temp.append((m,n))
          # sort with distance
            temp.sort(key=lambda elem: elem[0])
            length = len(temp)
            goodlen = 0
            
          # delete multiple overlapping lines
            for x in range(0,length):
              if x != length-1:
                if abs(temp[x+1][0]-temp[x][0]) > 0.05*width:
                      good.append([temp[x][0],temp[x][1]])
                      goodlen = goodlen + 1
                elif abs(temp[x+1][1]-temp[x][1]) > 0.35:
                      good.append([temp[x][0],temp[x][1]])
                      goodlen = goodlen + 1
              elif abs(temp[x-1][0]-temp[x][0]) < 0.05*width:
                    good.append([temp[x][0],temp[x][1]])
                    goodlen = goodlen + 1
          #calculate angle
            
            A = []
            B = []
            #print goodlen
            Count = 0;
            Count = 0;
            #filter lines with certain angle
            for x in range(0,goodlen):
              for y in range(x,goodlen):
                  if abs(good[x][1]-good[y][1])>0.7 and abs(good[x][1]-good[y][1])<0.9:
                      
                        A.append(good[x])
                        B.append(good[y])
                        break
            
            if A is not []:
              #print A
              A_length = len(A)
              intesecList = []
              twoPointsList = []
              for x in range(0,A_length):
                intesec = intersect(A[x],B[x])
                #calculate distance between center and intersection
                
                for i in circles[0,:]:
                  center = []
                  center.append(i[0])
                  center.append(i[1])
                  dist = distance(center,intesec)
                  if dist < 0.8*i[2]:
                    twoPointsList.append([center,intesec])
                    intesecList.append(intesec)

              #draw line between center and intersection
              for x,y in twoPointsList:
                cv2.line(view,(x[0],x[1]),(y[0],-y[1]),(0,0,255),2)

              #print intersection from list
              for x in intesecList:
                cv2.circle(view,(int(x[0,0]),-int(x[1,0])),10,255,-1)

                #print intesec[0,0]
                #cv2.circle(view,(int(intesec[0,0]),-int(intesec[1,0])),10,255,-1)
                

                
            for rho,theta in good:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                    
                cv2.line(view,(x1,y1),(x2,y2),(0,0,255),2)
                


    cv2.imshow('view',view)
    
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break
    if k == 99:
          cv2.imwrite("frames/"+str(index)+".jpg",view)
          cv2.imwrite("frames/edges_"+str(index)+".jpg",edges)
          index = index+1
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
