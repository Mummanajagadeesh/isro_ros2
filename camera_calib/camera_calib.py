import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('test_images/*.jpeg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (9,6), corners2, ret)
        height, width, channels = img.shape
        cv.imshow('img', cv.resize(img, (width//3, height//3)))
        cv.waitKey(500)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Extract individual parameters from the camera matrix
fx = mtx[0, 0]
fy = mtx[1, 1]
cx = mtx[0, 2]
cy = mtx[1, 2]

# Extract distortion coefficients
k1 = dist[0, 0]
k2 = dist[0, 1]
p1 = dist[0, 2]
p2 = dist[0, 3]

# Get image dimensions
width = gray.shape[1]
height = gray.shape[0]

# Print in the exact format requested
print("# Camera calibration and distortion parameters (OpenCV)")
print(f"Camera1.fx: {fx:.3f}")
print(f"Camera1.fy: {fy:.3f}")
print(f"Camera1.cx: {cx:.3f}")
print(f"Camera1.cy: {cy:.3f}")
print("")
print(f"Camera1.k1: {k1:.8f}")
print(f"Camera1.k2: {k2:.8f}")
print(f"Camera1.p1: {p1:.8f}")
print(f"Camera1.p2: {p2:.8f}")
print("")
print(f"Camera.width: {width}")
print(f"Camera.height: {height}")

"""
CHECKING UNDISTORTION ON A TEST IMAGE 

img = cv.imread('test_images/12.jpeg')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult.png', dst)
dst_h, dst_w = dst.shape[:2]
cv.imshow('calib result',cv.resize(dst, (dst_w//3, dst_h//3)))
cv.waitKey(0)
"""

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print("\nTotal reprojection error: {}".format(mean_error/len(objpoints)) )

cv.destroyAllWindows()

