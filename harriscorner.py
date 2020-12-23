#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Detect corners using C. Harris and M. Stephens 1988
# see https://medium.com/data-breach/introduction-to-harris-corner-detector-32a88850b3f6

# Pseudo code
# 1.Take the grayscale of the original image
# 2. Apply a Gaussian filter to smooth out any noise
# 3. Apply Sobel operator to find the x and y gradient values for every pixel in the grayscale image
# 4. For each pixel p in the grayscale image, consider a 3×3 window around it and compute the corner strength function. Call this its Harris value.
# 5. Find all pixels that exceed a certain threshold and are the local maxima within a certain window (to prevent redundant dupes of features)
# 6. For each pixel that meets the criteria in 5, compute a feature descriptor.

#import matplotlib.pyplot as plt
import numpy as np
import cv2

ver = (cv2.__version__)
print( "Version",ver )

# read image
img = cv2.imread( "Images/cu_baby_320.jpg" )
#print( "DEPTH",img.depth )
# copy image and change to RGB
img_cpy = np.copy( img )
img_cpy = cv2.cvtColor( img_cpy, cv2.COLOR_BGR2RGB )
cv2.imshow( "Image originelle", img_cpy )

# 1. grayscale
gray = cv2.cvtColor( img_cpy, cv2.COLOR_RGB2GRAY)
gray = np.float32(gray)

# *********************************************************************** HARRIS
harris_corner_win = "Harris Corners"
harris_dst_win = "Harris DST"
cv2.namedWindow( harris_corner_win )
cv2.namedWindow( harris_dst_win )
thres = 200
max_thres = 255
def corner_harris( val_tresh ):
    
    # 2. DetectCorners
    # dst =	cv.cornerHarris( src, blockSize, ksize, k[, dst[, borderType]]	)
    # dst = detM^(x,y) - k (trM^(x,y))^2
    dst = cv2.cornerHarris( src=gray,
                            blockSize=2,
                            ksize=3, #aperture, Sobel operator
                            k=0.04, # free parameter
    )
    cv2.imshow( harris_dst_win, dst )
    # dst = cv2.dilate( dst, None )

    # Normalizing
    dst_norm = np.empty(dst.shape, dtype=np.float32)
    cv2.normalize(dst, dst_norm, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    dst_norm_scaled = cv2.convertScaleAbs(dst_norm)
    
    thresh = val_tresh #val_thres * dst.max() 

    # Create an image copy to draw corners on
    corner_img = np.copy( img_cpy)
    # Iterate through all the corners and draw them on the image
    # (if they pass the threshold)
    for j in range(0, dst.shape[0]):
        for i in range(0, dst.shape[1]):
            if(dst_norm_scaled[j,i] > thresh):
                # image, center pt, radius, color, thickness
                cv2.circle( corner_img, (i, j), 3, (0,255,0), 1)
    cv2.imshow( harris_corner_win, corner_img )

cv2.createTrackbar( 'Thres: ', harris_corner_win, thres, max_thres, corner_harris )
cv2.imshow( harris_corner_win, img_cpy )
cv2.imshow( harris_dst_win, img_cpy )

# ***************************************************** Shi-Tomasi Good Features
shitomasi_corner_win = "ShiTomasi Corners"
cv2.namedWindow( shitomasi_corner_win )
quality = 10
quality_max = 100
quality_param = None
mindist = 5
mindist_max = 50
def corner_shitomasi( qual, min_dist ):
    corners = cv2.goodFeaturesToTrack(gray, 25, qual, min_dist)
    corners = np.int0(corners)

    shi_img = np.copy( img_cpy)
    for i in corners:
        x,y = i.ravel()
        cv2.circle( shi_img,(x,y), 3, 255,-1)
    cv2.imshow( shitomasi_corner_win, shi_img )

def shitomasi_quality( val ):
    global quality_param
    quality_param = float(val) / float(quality_max)
    corner_shitomasi( quality_param, mindist )
def shitomasi_mindist( val ):
    global mindist
    mindist = val
    corner_shitomasi( quality_param, mindist )

cv2.createTrackbar( 'Quality: ', shitomasi_corner_win, quality, quality_max, shitomasi_quality )
cv2.createTrackbar( 'MinDist: ', shitomasi_corner_win, mindist, mindist_max, shitomasi_quality )

cv2.imshow( shitomasi_corner_win, img_cpy )


# ************************************************************************ Canny
canny_line_win = "Canny Lines"
cv2.namedWindow( canny_line_win )
canny_det_win = "Cannu Det"
cv2.namedWindow( canny_det_win )
low_th = 0
low_th_max = 100
high_th = 10
high_th_max = 100
kernel_size = 3
def lines_canny( low, high ):
    
    img_blur = cv2.blur( gray, (3,3))
    img_blur = np.uint8(img_blur)
    # print( "GRAY",gray.depth )
    # print( "BLUR",img_blur.depth )
    detected_edges = cv2.Canny(img_blur,
                               low, high, kernel_size)
    cv2.imshow(canny_det_win, detected_edges)

    mask = detected_edges != 0
    dst_canny = img * (mask[:,:,None].astype(img.dtype))
    cv2.imshow(canny_line_win, dst_canny)

def canny_low( val ):
    global low_th
    low_th = min( val, high_th )
    lines_canny( low_th, high_th )
def canny_high( val ):
    global high_th
    high_th = max( val, low_th )
    lines_canny( low_th, high_th )

cv2.createTrackbar( 'Low: ', canny_line_win, low_th, low_th_max, canny_low )
cv2.createTrackbar( 'High: ', canny_line_win, high_th, high_th_max, canny_high )
cv2.imshow(canny_line_win, img_cpy)
cv2.imshow(canny_det_win, img_cpy)


# ************************************************************************* Blob
blob_win = "Blob Detector"
cv2.namedWindow( blob_win )
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
# img_gray = cv2.imread( "Images/cu_baby_320.jpg",  cv2.IMREAD_GRAYSCALE)
# # Change thresholds
params.minThreshold = 10
params.maxThreshold = 200


# # Filter by Area.
params.filterByArea = True
params.minArea = 100

# Filter by Color
params.filterByColor = True
params.blobColor = 0

# # Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# # Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.9

# # Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0.9

# # Create a detector with the parameters
blob = cv2.SimpleBlobDetector_create(params)
# # Detect blobs.
img_src = np.uint8( gray )
keypoints = blob.detect( img_src )
# keypoints = angle, class_id, octave (pyramid layer), pt2D, response, size
print(">*** {} points".format( len(keypoints)))
for k in keypoints:
    print( "s={}".format( k.size ))

# # Draw detected blobs as red circles.
# # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# # the size of the circle corresponds to the size of blob

blob_img = cv2.drawKeypoints( img_cpy, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# # Show blobs
cv2.imshow( blob_win, blob_img )

# ************************************************************************** ORB
# orb_corner_win = "ORB Corners"
# cv2.namedWindow( orb_corner_win )
# orb = cv2.ORB()

# def corner_orb():
#     img_gray = np.uint8( gray )
#     kp = orb.detect( img_gray)

#     orb_img = cv2.drawKeypoints(img_cpy,kp,color=(0,255,0), flags=0)
#     cv2.imshow( orb_corner_win, orb_img )

# cv2.imshow( orb_corner_win, img_cpy )
# corner_orb()

cv2.waitKey()
