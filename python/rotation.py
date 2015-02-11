import cv2
import numpy as np
from operator import itemgetter, attrgetter, methodcaller
from numpy.linalg import inv
import math

img = cv2.imread('roombatest.jpg')
#img = cv2.resize(img,(60,60))
img = cv2.medianBlur(img,5)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)
lines = cv2.HoughLines(edges,1,np.pi/180,80)
good = []
temp = []
arr1 = []
arr2 = []
hight, width, channel = img.shape
if lines != None:
  for m,n in lines[0]:
      temp.append((m,n))
# sort with distance
  temp.sort(key=lambda elem: elem[0])
  len = len(temp)
  goodlen = 0
# delete multiple overlapping lines
  for x in range(0,len):
    if x != len-1:
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
  print goodlen

  for x in range(0,goodlen):
    for y in range(x,goodlen):
        if abs(good[x][1]-good[y][1])>0.7 and abs(good[x][1]-good[y][1])<0.9:
            
              A = good[x]
              B = good[y]
              break

  def intersect(p1,p2):
        mat = np.matrix(((math.cos(p1[1]),-math.sin(p1[1])),(math.cos(p2[1]),-math.sin(p2[1]))))
        mat = inv(mat)
        dis = np.matrix((p1[0],p2[0]))
        dis = dis.transpose()
        result = np.dot(mat,dis)
        return result
  intesec = intersect(A,B)
  print intesec[0,0]
  cv2.circle(img,(int(intesec[0,0]),-int(intesec[1,0])),10,255,-1)
  for rho,theta in good:
      a = np.cos(theta)
      b = np.sin(theta)
      x0 = a*rho
      y0 = b*rho
      x1 = int(x0 + 1000*(-b))
      y1 = int(y0 + 1000*(a))
      x2 = int(x0 - 1000*(-b))
      y2 = int(y0 - 1000*(a))
    
      cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
cv2.imshow('grid',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
