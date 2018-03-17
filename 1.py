import numpy as np
import cv2
import serial


newFile = open("3.raw", "rb")
b = newFile.read(80000)

#img = cv2.CreateMat(320,240,CV_8UC1,b)
img = cv2.imdecode(b,cv2.CV_8UC1)
cv2.imshow('image',img)




k = cv2.waitKey(0)
if k == 27:         # wait for ESC key to exit
    cv2.destroyAllWindows()
elif k == ord('s'): # wait for 's' key to save and exit
    cv2.imwrite('messigray.png',img)
    cv2.destroyAllWindows()
