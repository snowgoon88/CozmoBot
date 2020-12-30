#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS node that tries to detect the cube of the Cozmo robot using
# OpenCV ORB features and BruteForce keypoint matching.
# see for example
# - https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_orb/py_orb.html#orb
# - https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html#matcher
#
# This is a work in progress and this node is more like a debugging tool.
# Cubes are NOT recognized yet. Far from it in fact...

# Some "Models" are loaded and their keypoint descriptors are looked for using ORB
# these models are images of the pattern on the cubes
#
# Subscribes to /in (likely remapped to /camera)
# Every ROS::Image is transformed to a cv2.image => Query
# then keypoint descriptor are extracted using ORB
# and a BruteForce search looks for the Model whith keypoints that matches
# best the keypoints of the query Image.
#
# A naive and quick and crude criteria is used to select the most likely cube.
# i.e. the model with the most keypoints that we can add so that their cumulative
# distance is inferior to '_GOOD_SUM_THRES'
#
# Trackbars/sliders allow to play with some parameters of the detector

import time
import sys
import rospy
import rospkg

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

# CV <-> ROS
bridge = CvBridge()

# OpenCV windows
_out_win = "Final ORB detection"
_orbmatcher_win = None
_default_img = None
_nb_lines_to_draw = 10
_max_nb_lines_to_draw = 50
_cube_to_draw = 0
_max_cube_to_draw = 10 # depends on the nb of models
_cv_font = cv2.FONT_HERSHEY_SIMPLEX
_fontscale = 0.5

# The URB detector
_orb = cv2.ORB_create()
# default param
_orb_param = {
    'patch_size': 10,
    'fast_thres': 90,
}
_orb.setPatchSize( _orb_param['patch_size'] )
_orb.setEdgeThreshold( _orb_param['patch_size'] )
_orb.setFastThreshold( _orb_param['fast_thres'] )
_max_patch_size = 100
_max_fast_thres = 100

_GOOD_SUM_THRES = 100
_GOOD_THRES = 3

# BruteForce matching
_bforce = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# the models to match
_models = []

class Model(object):
    """ Hold image of the cubes, their keypoints and matches distances to query
    """
    def __init__(self, pathname, id=-1, name=None, color=(0,255,0)):
        self.id = id
        self.name = name
        self.color = color
        self.read( pathname )
        self.kp = None
        self.desc = None

    def read(self, pathname):
        self.img = cv2.imread( pathname )
        if self.img is None:
            print( "Could not read image "+pathname )
            sys.exit()
        self.gray = np.uint8(cv2.cvtColor( self.img, cv2.COLOR_RGB2GRAY))
        self.width = self.img.shape[1]
        self.height = self.img.shape[0]

    def compute_kpdesc(self, orb ):
        # a kp will have pt (x,y)
        self.kp, self.desc = orb.detectAndCompute(self.gray, None)
        

    def compute_match(self, bforce, query ):
        # sorted list of matches as (distance, trainIdx, queryIdx)
        self.matches = None
        try:
            raw_matches = bforce.match( self.desc, query.desc )
            self.matches = sorted( raw_matches,
                                   key = lambda x:x.distance )
        except:
            print( "__compute_match: Strange error dumping" )
            now = time.time()
            cv2.imwrite( pkg_path+"/dump_query_{}.png".format( now ), query.img )

            with open( pkg_path+"/dump_desc_{}.txt".format( now ), 'w') as f:
                f.write( "dump by {}-{}\n".format( self.name, self.id ))
                f.write( "*****"+query.name+"*************\n" )
                f.write( "shape={}\n".format( query.img.shape ))
                for el in query.desc:
                    f.write( "{}\n".format( el ))
                for m in _models:
                    f.write( "*****"+m.name+"*************\n" )
                    f.write( "shape={}\n".format( m.img.shape ))
                    for el in m.desc:
                        f.write( "{}\n".format( el ))
        

    def compute_stats_drawlines(self, query, res_img ):
        if self.matches is None:
            return 0,_GOOD_SUM_THRES+1.0
        cube_offset = np.array( (100, self.id*self.height) )
        query_offset = np.array( (100 + self.width, 0) )
        self.nb_good = 0
        self.sum_good = 0
        min_dist = sys.float_info.max
        sum_dist = 0
        count_line = 0
        for m in self.matches:
            sum_dist += m.distance
            if sum_dist < _GOOD_SUM_THRES:
                self.nb_good += 1
                self.sum_good = sum_dist
            if m.distance < min_dist:
                min_dist = m.distance    
                
            # draw lines
            if (_cube_to_draw == 4 or self.id == (_cube_to_draw - 1)) \
               and count_line < _nb_lines_to_draw:
                pt_cube = self.kp[m.queryIdx].pt
                pt_query  = query.kp[m.trainIdx].pt
                #print( "cube {} => ukn {}".format( pt_cube, pt_ukn ))
                pt_cube = np.array( pt_cube ) + cube_offset 
                pt_query = np.array( pt_query) + query_offset
                cv2.line( res_img,
                          tuple(pt_cube.astype(int)),
                          tuple(pt_query.astype(int)),
                          self.color, 1 )
            count_line += 1

        # write stats
        min_txt = "m: {:6.2f}".format( min_dist )
        nb_txt = "nb: {}".format( self.nb_good )
        sum_txt = "s: {:6.2f}".format( self.sum_good )
        tw, thm = cv2.getTextSize( min_txt, _cv_font, _fontscale, 1)[0]
        cv2.putText( res_img, min_txt,
                     (10, 10+self.id*self.height+thm),
                     _cv_font, _fontscale, self.color, thickness = 1 )
        tw, thn = cv2.getTextSize( nb_txt, _cv_font, _fontscale, 1)[0]
        cv2.putText( res_img, nb_txt,
                     (10, 10+self.id*self.height+thm+5+thn),
                     _cv_font, _fontscale, self.color, thickness = 1 )
        tw, ths = cv2.getTextSize( sum_txt, _cv_font, _fontscale, 1)[0]
        cv2.putText( res_img, sum_txt,
                     (10, 10+self.id*self.height+thm+5+thn+5+ths),
                     _cv_font, _fontscale, self.color, thickness = 1 )

        return self.nb_good, self.sum_good

            
class QueryImage(object):
    """ Hold image of the new Image, its keypoints
    """
    def __init__(self, ROS_msg, id=-1, name=None ):
        self.id = id
        self.name = name
        self.img = bridge.imgmsg_to_cv2( ROS_msg )
        self.gray = np.uint8(cv2.cvtColor( self.img, cv2.COLOR_RGB2GRAY))
        self.width = self.img.shape[1]
        self.height = self.img.shape[0]
        self.kp = None
        self.desc = None

    def compute_kpdesc(self, orb ):
        self.kp = None
        self.desc = None
        self.kp, self.desc = orb.detectAndCompute(self.gray, None )

class OrbMatcherWindow:
    """Build and maintain a cv2 base image for displaing results.
    
    Must be updated if _orb_parameters of display parameters are changed.

    a white stripe of 100px on the left to write some stats
    --> 100px + _model.width + query.width(320)
    |
    | max( 3 * _model.height, query.height)
    V 
    """
    def __init__(self, models, query):
        self.models = models
        self.query = query

        self.base_img = self.build_base_image( query.width, query.height )
        # the self.models_img is the base of the final image.
        # lacks only the query image and the lines
        self.models_img = self.update_models_img()

    def build_base_image(self, query_width, query_height ):
        mod_w = _models[0].width
        mod_h = _models[0].height
        height = max( 3 * mod_h, query_height )
        width = 100 + mod_w + query_width
        # black image
        base_img = np.zeros( (height, width, 3), np.uint8 )
        # with white on left side, np image are [height,width,depth]
        base_img[:, 0:mod_w] = 255 
        return base_img

    def update_models_img(self):
        self.models_img = self.base_img.copy()
        # print( "Models", self.models_img.shape )
        # build an image with a circle on each kp for the _models
        # and paste to self.models_img
        for m in self.models:
            if m.kp is not None:
                # print( "m.img", m.width, m.height, m.img.shape, m.id )
                kp_img = cv2.drawKeypoints( m.img, m.kp, np.array([]),
                                            color=m.color, flags=0)
                # print( "kp_img", kp_img.shape )
                self.models_img[m.id*m.height:(m.id+1)*m.height,
                                100:100+m.width] = kp_img

def image_cbk( message ):
    #print( "New image" )
    query = QueryImage( message, id=-1, name="cozmo" )

    # Description points for the Query
    query.compute_kpdesc( _orb )
    if query.kp is None or query.desc is None:
        #print( "Could not get descriptors")
        return

    # Output image
    res_img = _orbmatcher_win.models_img.copy()
    cube_w = _models[0].width

    # add the query image and its keypoint to res_img
    # np image are [heigh,width,depth]
    query_kp_img = cv2.drawKeypoints( query.gray,
                                      query.kp,
                                      np.array([]),
                                      (255,255,0),
                                      cv2.DRAW_MATCHES_FLAGS_DEFAULT )
    res_img[0:query.height,100+cube_w:] = query_kp_img

    # compute best match
    best_cube = None
    best_nb_good = _GOOD_THRES
    best_sum = sys.float_info.max
    for cube in _models:
        cube.compute_match( _bforce, query )

        # compute statistics and draw lines (if needed) on res_img
        nb,sum = cube.compute_stats_drawlines( query, res_img )
        if nb > best_nb_good or (nb >= best_nb_good and sum < best_sum):
            best_cube = cube.name
            best_nb_good = nb
            best_sum = sum

    # write global result on res_img
    best_txt = "None"
    if best_cube is not None:
        best_txt = "*{}*".format( best_cube )
    tw, th = cv2.getTextSize( best_txt, _cv_font, _fontscale, 2)[0]
    w = res_img.shape[1]
    # print( "w={}, tw={}".format( w, tw ))
    cv2.putText( res_img, best_txt,
                     (int(w - tw - 10), 10+th), # pos of bottom left of text
                     _cv_font, _fontscale, (0,255,0), thickness = 2 )
    
    cv2.imshow( _out_win, res_img )

def start_node():
    rospy.init_node( 'cube_orb_detector' )
    rospy.Subscriber( 'in', Image, image_cbk )

    # main loop with watchdog
    try:
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            rospy.sleep( 0.1 )
            cv2.waitKey(1)
    finally:
        print( "This is the end...." )


def update_orb():
    """Compute kp for _models and update window"""
    print( "Update ORB => _models" )
    for m in _models:
        m.compute_kpdesc( _orb )
    _orbmatcher_win.update_models_img()

def update_patch_size( val ):
    """ Update PatchSize and EdgeThreshold ORB parameters"""
    _orb_param['patch_size'] = val
    _orb.setPatchSize( _orb_param['patch_size'] )
    _orb.setEdgeThreshold( _orb_param['patch_size'] )
    update_orb()

def update_fast_treshold( val ):
    """ Update FastThreshold ORB parameter"""
    _orb_param['fast_thrs'] = val
    _orb.setFastThreshold( _orb_param['fast_thrs'] )
    update_orb()


def update_nb_lines( val ):
    """Update some display parameters: how many lines (max) are drawn between
    a cube and the query image
    """
    global _nb_lines_to_draw
    _nb_lines_to_draw = val

def update_id( val ):
    """Update some display parameters: for which cube(s) (as _models) do we
    draw lines. 0 => no cube, x => cube.id, nbcube+1 => for all cubes
    """
    global _cube_to_draw
    _cube_to_draw = val
    

def build_GUI( win ):
    """Build Trackbars to the cv2 Window"""
    cv2.createTrackbar( "P.E.Size", win, _orb_param['patch_size'],
                        _max_patch_size,
                        update_patch_size )
    cv2.createTrackbar( "Fast Th", win, _orb_param['fast_thres'],
                        _max_fast_thres,
                        update_fast_treshold )

    cv2.createTrackbar( "Nb Lines", win, _nb_lines_to_draw , _max_nb_lines_to_draw,
                        update_nb_lines )
    cv2.createTrackbar( "ID cube", win, _cube_to_draw, _max_cube_to_draw,
                        update_id )
        
if __name__ == '__main__':
    # location of package
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('cozmo')
    print( "COZMO path="+pkg_path )

    # load models
    cube_names = ["tromb", "lamp", "baby"]
    cube_colors = [(255,0,0), (0,255,0), (0,0,255)]
    cube_size = 124

    for i in range(len(cube_names)):
        _models.append(
            Model( pkg_path+"/Ressources/cu_{}_{}.png".format( cube_names[i], cube_size),
                   id = i,
                   name = cube_names[i],
                   color = cube_colors[i] ))
    # load a default 320x240 image
    default_img = Model( pkg_path+"/Ressources/no_image.png" )
    _max_cube_to_draw = len(_models)+1
    # initialise the base_image for rendering results
    _orbmatcher_win = OrbMatcherWindow( _models, default_img )
    update_orb() # and kp for _models

    cv2.namedWindow( _out_win )
    build_GUI( _out_win )
    cv2.imshow( _out_win, _orbmatcher_win.models_img )
    
    start_node()
    

