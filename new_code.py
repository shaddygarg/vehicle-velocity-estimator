import numpy as np
import cv2
import global_variables as gV
import random


gV.selRoi = 0
gV.top_left= [20,40]
gV.bottom_right = [60,120]
gV.first_time = 1

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

def findDistance(r1,c1,r2,c2):
	d = (r1-r2)**2 + (c1-c2)**2
	d = d**0.5
	return d

cv2.namedWindow('tracker')

cap = cv2.VideoCapture('output.avi')

fgbg = cv2.createBackgroundSubtractorKNN()

_,frame = cap.read()
_,frame = cap.read()
_,frame = cap.read()
_,frame = cap.read()
_,frame = cap.read()
_,frame = cap.read()

while True:
        c = 0
        c1 = 0
        ans = 0
	while True:
		_,frame = cap.read()
		#-----Drawing Stuff on the Image
		cv2.rectangle(frame,(gV.top_left[1],gV.top_left[0]),(gV.bottom_right[1],gV.bottom_right[0]),color = (100,255,100),thickness = 4)

                fgmask = fgbg.apply(frame)

                cv2.imshow('frame',fgmask)
                if int(fgmask[40,80]) > 125:
                    c+=1
                    if c>4:
                        break

		#-----Finding ROI and extracting Corners
		frameGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		roi = frameGray[ gV.top_left[0]:gV.bottom_right[0], gV.top_left[1]:gV.bottom_right[1] ] #selecting roi
		new_corners = cv2.goodFeaturesToTrack(roi, 50, 0.01, 10) #find corners

		#-----converting to complete image coordinates (new_corners)

		new_corners[:,0,0] = new_corners[:,0,0] + gV.top_left[1]
		new_corners[:,0,1] = new_corners[:,0,1] + gV.top_left[0]

		#-----drawing the corners in the original image
		r_add, c_add = 0,0
		for corner in new_corners:
			cv2.circle(frame, (int(corner[0][0]),int(corner[0][1])) ,5, (0,255,0))
			r_add = r_add + corner[0][1]
			c_add = c_add + corner[0][0]


		#-----old_corners and oldFrame is updated
		oldFrameGray = frameGray.copy()
		old_corners = new_corners.copy()
		centroid_old_row = int(1.0*r_add/len(new_corners))
		centroid_old_col = int(1.0*c_add/len(new_corners))

		cv2.imshow('tracker',frame)

		a = cv2.waitKey(50)
		if a== 27:
			cv2.destroyAllWindows()
			cap.release()
		elif a == 97:
			break

	#Actual Tracking
	while True:
		#Now we have oldFrame, we can get new_frame, we have old corners and we can get new corners and update accordingly
		flag=0

        #read new frame and cvt to gray
		ret,frame = cap.read()
		frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #finding the new tracked points
		new_corners, st, err = cv2.calcOpticalFlowPyrLK(oldFrameGray, frameGray, old_corners, None, **lk_params)

		# ---pruning far away points:
		# first finding centroid
		r_add, c_add = 0,0
		for corner in new_corners:
			r_add = r_add + corner[0][1]
			c_add = c_add + corner[0][0]
		centroid_row = int(1.0*r_add/len(new_corners))
		centroid_col = int(1.0*c_add/len(new_corners))

        # draw centroid
		cv2.circle(frame, (int(centroid_col), int(centroid_row)), 5, (255,0,0))

        # add only those corners to new_corners_updated which are at a distance of 30 or lesser
		new_corners_updated = new_corners.copy()
		tobedel = []
		for index in range(len(new_corners)):
			if findDistance(new_corners[index][0][1], new_corners[index][0][0], int(centroid_row), int(centroid_col)) > 90:
				tobedel.append(index)
		new_corners_updated = np.delete(new_corners_updated,tobedel,0)

		# drawing the new points
		for corner in new_corners_updated:
			cv2.circle(frame, (int(corner[0][0]),int(corner[0][1])), 5, (0, 255, 0))
			if  int(corner[0][0])>319:
				flag = 1
				break

		c1 += 1
		if len(new_corners_updated)<10 or flag==1 or c1==25:
			break

		ans += findDistance(centroid_old_col, centroid_old_row, centroid_col, centroid_row)

		# finding the min enclosing circle
		ctr , rad = cv2.minEnclosingCircle(new_corners_updated)

		cv2.circle(frame, (int(ctr[0]), int(ctr[1])), int(rad), (0, 0, 255), thickness = 5)

		# updating old_corners and oldFrameGray
		oldFrameGray = frameGray.copy()
		old_corners = new_corners_updated.copy()
		centroid_old_row = centroid_row
		centroid_old_col = centroid_col


		cv2.imshow('tracker', frame)

		a = cv2.waitKey(50)
		if a== 27:
			cv2.destroyAllWindows()
			cap.release()
		elif a == 97:
			break
	print "The average speed of the car was {}".format(ans/c1-1)

cv2.destroyAllWindows()
