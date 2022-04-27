import numpy as np
import cv2
import tkinter
import tkinter.messagebox

square_side_length = 100.0 #The size of one side of a square in a chess board(mm)
grid_intersection_size = (10, 10) #Number of grids in the chess board

pattern_points = np.zeros( (np.prod(grid_intersection_size), 3), np.float32 )
pattern_points[:,:2] = np.indices(grid_intersection_size).T.reshape(-1, 2)
pattern_points *= square_side_length
object_points = []
image_points = []

root = tkinter.Tk()
root.withdraw()

video_input = cv2.VideoCapture(1)
if (video_input.isOpened() == False):
    exit()

camera_mat, dist_coef = [], []

if tkinter.messagebox.askyesno('askyesno','Calibration data(K.csv, d.csv)Do you want to read?'):
    #Reading calibration data
    camera_mat = np.loadtxt('K.csv', delimiter=',')
    dist_coef = np.loadtxt('d.csv', delimiter=',')
    print("K = \n", camera_mat)
    print("d = ", dist_coef.ravel())
else:
    #Shooting chess board
    capture_count = 0
    while(True):
        ret, frame = video_input.read()

        #Convert to grayscale image for chessboard detection
        #grayscale_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        #Detects chess board corners
        #found, corner = cv2.findChessboardCorners(grayscale_image, grid_intersection_size)
        found, corner = cv2.findChessboardCorners(frame, grid_intersection_size)

        if found == True:
            print('findChessboardCorners : True')

            #Find Chessboard Corners in current OpenCV()Within, cornerSubPix()Is considerable processing being carried out? Confirmation required
            #term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            #cv2.cornerSubPix(grayscale_image, corner, (5,5), (-1,-1), term)
            #cv2.drawChessboardCorners(grayscale_image, grid_intersection_size, corner, found)

            cv2.drawChessboardCorners(frame, grid_intersection_size, corner, found)
        if found == False:
            print('findChessboardCorners : False')

        cv2.putText(frame, "Enter:Capture Chessboard(" + str(capture_count) + ")", (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
        cv2.putText(frame, "N    :Completes Calibration Photographing", (100, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
        cv2.putText(frame, "ESC  :terminate program", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
        #cv2.putText(grayscale_image, "Enter:Capture Chessboard(" + str(capture_count) + ")", (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
        #cv2.putText(grayscale_image, "ESC  :Completes Calibration Photographing.", (100, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
        cv2.imshow('original', frame)
        #cv2.imshow('findChessboardCorners', grayscale_image)

        c = cv2.waitKey(50) & 0xFF
        if c == 13 and found == True: # Enter
            #Added chess board corner detection information
            image_points.append(corner)
            object_points.append(pattern_points)
            capture_count += 1
        if c == 110: # N
            if tkinter.messagebox.askyesno('askyesno','Do you want to finish chessboard shooting and find the internal parameters of the camera?'):
                cv2.destroyAllWindows()
                break
        if c == 27: # ESC
            if tkinter.messagebox.askyesno('askyesno','Do you want to exit the program?'):
                video_input.release()
                cv2.destroyAllWindows()
                exit()

    if len(image_points) > 0:
        #Calculate camera internal parameters
        print('calibrateCamera() start')
        rms, K, d, r, t = cv2.calibrateCamera(object_points,image_points,(frame.shape[1],frame.shape[0]),None,None)
        print("RMS = ", rms)
        print("K = \n", K)
        print("d = ", d.ravel())
        np.savetxt("K.csv", K, delimiter =',',fmt="%0.14f") #Save camera matrix
        np.savetxt("d.csv", d, delimiter =',',fmt="%0.14f") #Preservation of strain coefficient

        camera_mat = K
        dist_coef = d

        #Evaluation by reprojection error
        mean_error = 0
        for i in xrange(len(object_points)):
            image_points2, _ = cv2.projectPoints(object_points[i], r[i], t[i], camera_mat, dist_coef)
            error = cv2.norm(image_points[i], image_points2, cv2.NORM_L2) / len(image_points2)
            mean_error += error
        print("total error: ", mean_error/len(object_points)) #A value close to 0 is desirable(Not suitable for evaluation of fisheye lenses?)
    else:
        print("findChessboardCorners() not be successful once")

#Distortion correction image display
if camera_mat != []:
    while(True):
        ret, frame = video_input.read()
        undistort_image = cv2.undistort(frame, camera_mat, dist_coef)

        cv2.imshow('original', frame)
        cv2.imshow('undistort', undistort_image)
        c = cv2.waitKey(50) & 0xFF
        if c==27: # ESC
            break

video_input.release()
cv2.destroyAllWindows()