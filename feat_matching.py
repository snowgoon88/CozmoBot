#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Use ORB to set keypoints, then features matching, then Homograhy,
# then matchlines
import sys
import numpy as np
import cv2

ver = (cv2.__version__)
print( "Open CV Version",ver )

GOOD_SUM_THRES = 100
GOOD_THRES = 3
orb_win = "ORB Matching"
cv2.namedWindow( orb_win )

cube_names = ["tromb", "lamp", "baby"]
cube_colors = [(255,0,0), (0,255,0), (0,0,255)]
cube_size = 124

## info on cubes
cube = {}
cube_gray = {}
cube_kp = {}
cube_desc={}
## info on unknown image
ukn_img = None
k, desc = None, None

# read images
for n in cube_names:
    cube[n] = cv2.imread( "Images/cu_{}_{}.png".format( n, cube_size ) )
    if( cube[n] is None):
        print( "Could not read Images/cu_{}_{}.png".format( n, cube_size ))
        sys.exit()
    cube_gray[n] = np.uint8(cv2.cvtColor( cube[n], cv2.COLOR_RGB2GRAY))
    
ukn_img = cv2.imread( "Images/cu_baby_320.jpg" )
if( ukn_img is None):
        print( "Could not read Images/cu_baby_320.jpg" )
        sys.exit()
ukn_gray = np.uint8(cv2.cvtColor( ukn_img, cv2.COLOR_RGB2GRAY))

# basic default image
# new image
height = max(3*cube_size, ukn_img.shape[0] )
width = 2*cube_size + ukn_img.shape[1]
base_img = np.zeros( (height, width, 3), np.uint8 )
base_img[:, 0:cube_size] = 255
# _idc = 0
# for n in cube_names:
#     base_img[_idc*cube_size:(_idc+1)*cube_size,0:cube_size] = cube[n]
#     _idc += 1
    
#ORB detector
orb = cv2.ORB_create()
#cur_patch_size = orb.getPatchSize()
cur_patch_size = 10
orb.setPatchSize( cur_patch_size )
orb.setEdgeThreshold( cur_patch_size )
max_patch_size = 100
#cur_fast_thres = orb.getFastThreshold()
cur_fast_thres = 90
orb.setFastThreshold( cur_fast_thres )
max_fast_thres = 100
cur_scale_factor = orb.getScaleFactor()
max_scale_factor = 10 # between 1 and 2
cur_nlevels = orb.getNLevels()
max_nlevels = 10
cur_first_lvl = orb.getFirstLevel()
max_first_level = 10

# display globals
nb_lines = 10
max_nb_lines = 100
id_cube = 4
max_id_cube = len(cube_names)+1
cv_font = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 0.5


def orb_and_match():
    # cubes
    cube_kp = {}
    cube_desc={}
    for n in cube_names:
        cube_kp[n] = orb.detect( cube_gray[n], None )
        cube_kp[n], cube_desc[n] = orb.compute( cube_gray[n], cube_kp[n] )
        print( "Found {} kp for cube {}".format( len(cube_kp[n]), n))

    # unknown
    kp = orb.detect( ukn_gray, None )
    kp, desc = orb.compute( ukn_gray, kp )
    print( "Found {} kp for cube {}".format( len(kp), "Images/cu_baby_320.jpg"))

    # Look for best Match Brute Force and FLANN
    bforce = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # FLANN_INDEX_KDTREE = 8
    # FLANN_INDEX_LSH = 6
    # index_params = dict( algorithm = FLANN_INDEX_LSH,
    #                      table_number = 6, #12,6
    #                      key_size = 12,    #20,12
    #                      multi_probe_level = 1 #2,1
    #                      )
    # #search_params = dict(checks = 50)
    # search_params = {}
    # flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches_bf = {}
    matches_flann = {}
    good_flann = {}
    best_match = None
    nb_match = 0
    for n in cube_names:
        print( "__MATCHES for {}".format(n))
        matches_bf[n] = bforce.match( cube_desc[n], desc)
        matches_bf[n] = sorted(matches_bf[n], key = lambda x:x.distance)
        print( "BF: Found {} matches".format( len(matches_bf[n] )))
        for m in matches_bf[n]:
            #print( "  {} of type={}".format( m, type(m)))
            print( "     d={} : {} -> {} ".format( m.distance, m.queryIdx, m.trainIdx))

        # matches_flann[n] = flann.knnMatch( cube_desc[n], desc, k=2)
        # print( "FLANN: Found {} matches".format( len(matches_flann[n] )))
        # good_flann[n] = []
        # for m1,m2 in matches_flann[n]:
        #     print( "     d={} : {} -> {} VS d={} : {} -> {} ".format(
        #         m1.distance, m1.queryIdx, m1.trainIdx,
        #         m2.distance, m2.queryIdx, m2.trainIdx
        #         ))

        #     if m1.distance < 0.7 * m2.distance:
        #         good_flann[n].append( m1 )
        # print( "FLANN: Found {} good for {}".format( len(good_flann[n]), n))


    # new image
    res_img = base_img.copy()
    
    idc = 0
    for n in cube_names:
        orb_img = cv2.drawKeypoints( cube[n], cube_kp[n], np.array([]),
                                     color=cube_colors[idc], flags=0)
        res_img[idc*cube_size:(idc+1)*cube_size,cube_size:2*cube_size] = orb_img

        idc += 1

    orb_img = cv2.drawKeypoints(ukn_img, kp, np.array([]), color=(0,255,0), flags=0)
    res_img[0:ukn_img.shape[0],2*cube_size:] = orb_img

    # draw lines to matches
    ukn_offset = np.array( (2*cube_size, 0) )
    nb_good = {}
    sum_good = {}
    idc = 0
    for n in cube_names:
        cube_offset = np.array( (cube_size, idc*cube_size) )
        count_line = 0
        min_dist = sys.float_info.max
        sum_dist = 0
        nb_good[n] = 0
        sum_good[n] = 0
        for m in matches_bf[n]:
            if count_line < nb_lines:
                pt_cube = cube_kp[n][m.queryIdx].pt
                pt_ukn  = kp[m.trainIdx].pt
                #print( "cube {} => ukn {}".format( pt_cube, pt_ukn ))
                pt_cube = np.array( pt_cube ) + cube_offset 
                pt_ukn = np.array( pt_ukn) + ukn_offset
                #print( "__line {} -> {}".format( pt_cube, pt_ukn))
                if id_cube == 4 or idc == (id_cube-1):
                    cv2.line( res_img,
                              tuple(pt_cube.astype(int)),
                              tuple(pt_ukn.astype(int)),
                              cube_colors[idc], 1 )
                sum_dist += m.distance
                if sum_dist < GOOD_SUM_THRES:
                    nb_good[n] += 1
                    sum_good[n] = sum_dist
                if m.distance < min_dist:
                    min_dist = m.distance
            count_line += 1
        min_txt = "m: {:6.2f}".format( min_dist )
        sum_txt = "s: {:6.2f}".format( sum_dist )
        tw, th = cv2.getTextSize( min_txt, cv_font, fontscale, 1)[0]
        cv2.putText( res_img, min_txt,
                     (10, 10+idc*cube_size+th),
                     cv_font, fontscale, cube_colors[idc], thickness = 1 )
        cv2.putText( res_img, sum_txt,
                     (10, 10+idc*cube_size+2*th+5),
                     cv_font, fontscale, cube_colors[idc], thickness = 1 )
        idc += 1

    best_cube = None
    best_nb_good = GOOD_THRES
    best_sum = sys.float_info.max
    for n in cube_names:
        if nb_good[n] >= best_nb_good and sum_good[n] < best_sum:
            best_cube = n
            best_nb_good = nb_good[n]
            best_sum = sum_good[n]

    best_txt = "None"
    if best_cube is not None:
        best_txt = "*{}*".format( best_cube )
    tw, th = cv2.getTextSize( best_txt, cv_font, fontscale, 2)[0]
    w = res_img.shape[1]
    print( "w={}, tw={}".format( w, tw ))
    cv2.putText( res_img, best_txt,
                     (int(w - tw - 10), 10+th),
                     cv_font, fontscale, (0,255,0), thickness = 2 )
        
    cv2.imshow( orb_win, res_img )

def update_patch_size( val ):
    global cur_patch_size
    cur_patch_size = val
    orb.setPatchSize( val )
    orb.setEdgeThreshold( val )
    orb_and_match()
def update_fast_treshold( val ):
    global cur_fast_thres
    cur_fast_thres = val
    orb.setFastThreshold( val )
    orb_and_match()
def update_scale_factor( val ):
    cur_scale_factor = 1.0 + val / 10.0
    orb.setScaleFactor( cur_scale_factor )
    orb_and_match()
def update_nlevels( val ):
    global cur_nlevels, cur_first_lvl
    cur_nlevels = val
    if cur_first_lvl >= cur_nlevels:
        cur_first_lvl = 0
        cv2.setTrackbarPos( first_track, orb_win, cur_first_lvl )
    orb.setNLevels( cur_nlevels )
    orb.setFirstLevel( cur_first_lvl )
    orb_and_match()
def update_firstlevel( val ):
    global cur_first_lvl
    cur_first_lvl = val
    if cur_first_lvl >= cur_nlevels:
        cur_first_lvl = cur_nlevels - 1
        cv2.setTrackbarPos( first_track, orb_win, cur_first_lvl )
    orb.setFirstLevel( cur_first_lvl )
    orb_and_match()

def update_nb_lines( val ):
    global nb_lines
    nb_lines = val
    orb_and_match()
def update_id( val ):
    global id_cube
    id_cube = val
    orb_and_match()
            
cv2.createTrackbar( "P.E.Size", orb_win, cur_patch_size, max_patch_size,
                    update_patch_size )
cv2.createTrackbar( "Fast Th", orb_win, cur_fast_thres, max_fast_thres,
                    update_fast_treshold )
cv2.createTrackbar( "Scale Fac", orb_win, int( 10*(cur_scale_factor - 1.0)),
                    max_scale_factor,
                    update_scale_factor )
cv2.createTrackbar( "N Levels", orb_win, cur_nlevels, max_nlevels,
                    update_nlevels )
first_track = cv2.createTrackbar( "First Lvl", orb_win, cur_first_lvl,
                                  max_first_level, update_firstlevel )

cv2.createTrackbar( "Nb Lines", orb_win, nb_lines, max_nb_lines,
                    update_nb_lines )
cv2.createTrackbar( "ID cube", orb_win, id_cube, max_id_cube,
                    update_id )

cv2.imshow( orb_win, base_img )
orb_and_match()
cv2.waitKey(-1)

