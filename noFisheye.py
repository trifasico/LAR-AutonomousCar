import numpy as np
import cv2
import sys
import os

# You should replace these 3 lines with the output in calibration step
DIM=(1920, 1080)
K=np.array([[1414.7164794319501, 0.0, 1077.8708554901095], [0.0, 1413.8651544790787, 577.421948183488], [0.0, 0.0, 1.0]])
D=np.array([[-0.016987095322426125], [-0.1476976019970751], [0.2034444046929388], [-0.17014911185457315]])

def undistort(img):
    #img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # cv2.imshow("undistorted", undistorted_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    #cv2.imwrite('out_' + os.path.basename(img_path), undistorted_img)
    return undistorted_img


video_input = cv2.VideoCapture(1)
video_input.set(3, 1920)
video_input.set(4, 1080)
while(cv2.waitKey(1) != 27):
    ret, frame = video_input.read()
    cv2.imshow("original", frame)
    #frame = undistort(frame)
    #cv2.imshow("new", frame)
    
    
