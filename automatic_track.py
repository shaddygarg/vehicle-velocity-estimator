import numpy as np
import cv2
import global_variables
import random

import multiprocessing

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

def findDistance(r1,c1,r2,c2):
    d = (r1-r2)**2 + (c1-c2)**2
    d = d**0.5
    return d

cv2.namedWindow('tracker')

cap = cv2.VideoCapture(global_variables.video_path)

fgbg = cv2.createBackgroundSubtractorKNN()

def calculate_optical_flow(cap, old_gray_frame, old_corners, centroid_old_row, centroid_old_col, c1, ans):
    while True:
            #Now we have oldFrame, we can get new_frame, we have old corners and we can get new corners and update accordingly
            flag=0

            #read new frame and cvt to gray
            ret,frame = cap.read()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #finding the new tracked points
            new_corners, st, err = cv2.calcOpticalFlowPyrLK(old_gray_frame, gray_frame, old_corners, None, **lk_params)

            # Finding Centroid of the points
            row_sum, column_sum = 0,0
            for corner in new_corners:
                    row_sum = row_sum + corner[0][1]
                    column_sum = column_sum + corner[0][0]
            centroid_row = int(1.0*row_sum/len(new_corners))
            centroid_col = int(1.0*column_sum/len(new_corners))

            # draw centroid
            cv2.circle(frame, (int(centroid_col), int(centroid_row)), 5, (255,0,0))

            # add only those corners to new_corners_updated which are at a distance of 30 or lesser
            new_corners_updated = new_corners.copy()
            to_be_deleted = []
            for index in range(len(new_corners)):
                    if findDistance(new_corners[index][0][1], new_corners[index][0][0], int(centroid_row), int(centroid_col)) > 30:
                            to_be_deleted.append(index)
            new_corners_updated = np.delete(new_corners_updated,to_be_deleted,0)

            # drawing the new points
            for corner in new_corners_updated:
                    cv2.circle(frame, (int(corner[0][0]),int(corner[0][1])), 5, (0, 255, 0))
                    if  int(corner[0][0])>319:
                            flag = 1
                            break

            c1 += 1
            if flag==1 or c1==25:
                if c1>1:
                    final_speed = (ans*20)/(c1-1)
                    print "The average speed of the car was {} pixels per second".format(final_speed)
                return

            ans += findDistance(centroid_old_col, centroid_old_row, centroid_col, centroid_row)

            # finding the min enclosing circle
            center, radius = cv2.minEnclosingCircle(new_corners_updated)

            cv2.circle(frame, (int(center[0]), int(center[1])), int(radius), (0, 0, 255), thickness = 5)

            # updating old_corners and old_gray_frame
            old_gray_frame = gray_frame.copy()
            old_corners = new_corners_updated.copy()
            centroid_old_row = centroid_row
            centroid_old_col = centroid_col


            cv2.imshow('tracker', frame)
            a = cv2.waitKey(50)
            if a== 27:
                    cv2.destroyAllWindows()
                    cap.release()
            elif a == 97:
                    return

while True:
    c = 0
    c1 = 0
    ans = 0
    c2 = 0
    while True:
        _,frame = cap.read()
        #-----Drawing Stuff on the Image
        cv2.rectangle(frame,(global_variables.top_left[1],global_variables.top_left[0]),(global_variables.bottom_right[1],global_variables.bottom_right[0]),color = (100,255,100),thickness = 4)
        cv2.rectangle(frame,(global_variables.top_left2[1],global_variables.top_left2[0]),(global_variables.bottom_right2[1],global_variables.bottom_right2[0]),color = (100,255,100),thickness = 4)
        fgmask = fgbg.apply(frame)

        cv2.imshow('frame',fgmask)

        if int(fgmask[40,80]) > 125:
            c+=1
            if c>4:
                calculate_optical_flow(cap, old_gray_frame, old_corners, centroid_old_row, centroid_old_col, c1, ans)
        elif int(fgmask[130,80]) > 125:
            c2+=1
            if c2>4:
                calculate_optical_flow(cap, old_gray_frame2, old_corners2, centroid_old_row2, centroid_old_col2, c1, ans) 

        #-----Finding ROI and extracting Corners
        gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        roi = gray_frame[ global_variables.top_left[0]:global_variables.bottom_right[0], global_variables.top_left[1]:global_variables.bottom_right[1] ] #selecting roi
        new_corners = cv2.goodFeaturesToTrack(roi, 50, 0.01, 10) #find corners

        roi2 = gray_frame[ global_variables.top_left2[0]:global_variables.bottom_right2[0], global_variables.top_left2[1]:global_variables.bottom_right2[1] ] #selecting roi
        new_corners2 = cv2.goodFeaturesToTrack(roi2, 50, 0.01, 10) #find corners

        #-----converting to complete image coordinates (new_corners)

        new_corners[:,0,0] = new_corners[:,0,0] + global_variables.top_left[1]
        new_corners[:,0,1] = new_corners[:,0,1] + global_variables.top_left[0]

        new_corners2[:,0,0] = new_corners2[:,0,0] + global_variables.top_left2[1]
        new_corners2[:,0,1] = new_corners2[:,0,1] + global_variables.top_left2[0]

        #-----drawing the corners in the original image
        row_sum, column_sum = 0,0
        for corner in new_corners:
            cv2.circle(frame, (int(corner[0][0]),int(corner[0][1])) ,5, (0,255,0))
            row_sum = row_sum + corner[0][1]
            column_sum = column_sum + corner[0][0]

        row_sum2, column_sum2 = 0,0
        for corner in new_corners2:
            cv2.circle(frame, (int(corner[0][0]),int(corner[0][1])) ,5, (0,255,0))
            row_sum2 = row_sum2 + corner[0][1]
            column_sum2 = column_sum2 + corner[0][0]

        #-----old_corners and oldFrame is updated
        old_gray_frame = gray_frame.copy()
        old_corners = new_corners.copy()
        centroid_old_row = int(1.0*row_sum/len(new_corners))
        centroid_old_col = int(1.0*column_sum/len(new_corners))

        old_gray_frame2 = gray_frame.copy()
        old_corners2 = new_corners2.copy()
        centroid_old_row2 = int(1.0*row_sum2/len(new_corners2))
        centroid_old_col2 = int(1.0*column_sum2/len(new_corners2))

        cv2.imshow('tracker',frame)

        a = cv2.waitKey(50)
        if a== 27:
            cv2.destroyAllWindows()
            cap.release()
        elif a == 97:
            break

cv2.destroyAllWindows()