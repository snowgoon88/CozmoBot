#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from opencv_node import IntParameter, ImageProcessor, get_default_img
import cv2
import numpy as np
import math

# ******************************************************************************
# ************************************************************* ThresoldDetector
# ******************************************************************************
class ThresoldDetector(ImageProcessor):
    """
    img to gray
    then thresold global, adaptive mean, adaptive gaussian
    then find contours
    """
    def __init__(self):
        super().__init__( get_default_img() )

        # Add parameters
        _param_thres = IntParameter( id='thres', title="Thres",
                                 init_val=127, max_val=255 )
        self.add_param( _param_thres )
        _param_blur = IntParameter( id='blur', title='Blur',
                                init_val=5, max_val=33,
                                update_func= lambda x : (x // 2)*2+1 )
        self.add_param( _param_blur )

    def run(self, img):
        h,w,d = img.shape
        # gray
        gray_img = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY)
        gray_img = np.uint8( gray_img )

        # blurring
        blur_img = cv2.medianBlur( gray_img, self.param['blur'].val )

        # global threshold
        ret, th1_img = cv2.threshold( blur_img, self.param['thres'].val,
                                      255, cv2.THRESH_BINARY )
        _,contours,_ = cv2.findContours(th1_img, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #print( "1 ",len(contours))
        ct1_img = cv2.drawContours(img.copy(), contours, -1, (0,255,0), 1)
        th2_img = cv2.adaptiveThreshold( blur_img, 255,
                                     cv2.ADAPTIVE_THRESH_MEAN_C,
                                     cv2.THRESH_BINARY, 11, 2)
        _,contours,_ = cv2.findContours(th2_img, cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        #print( "2 ",len(contours))
        ct2_img = cv2.drawContours(img.copy(), contours, -1, (0,255,0), 1)
        th3_img = cv2.adaptiveThreshold( blur_img, 255,
                                     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 11, 2)
        _,contours,_ = cv2.findContours(th3_img, cv2.RETR_LIST,
                                               cv2.CHAIN_APPROX_SIMPLE)
        #print( "3 ",len(contours))
        ct3_img = cv2.drawContours(img.copy(), contours, -1, (0,255,0), 1)


        # otsu global threshold
        ret, th4_img = cv2.threshold(blur_img,
                                     0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        _,contours,_ = cv2.findContours(th4_img, cv2.RETR_LIST,
                                               cv2.CHAIN_APPROX_SIMPLE)
        ct4_img = cv2.drawContours(img.copy(), contours, -1, (0,255,0), 1)
        #output image
        res_img = np.zeros( (3 * h, 4 * w, 3), dtype="uint8")
        res_img[0:h,0:w] = img
        res_img[h:2*h,0:w,0] = th1_img
        res_img[h:2*h,w:2*w,0] = th2_img
        res_img[h:2*h,2*w:3*w,0] = th3_img
        res_img[h:2*h,3*w:4*w,0] = th4_img
        res_img[2*h:3*h,0:w] = ct1_img
        res_img[2*h:3*h,w:2*w] = ct2_img
        res_img[2*h:3*h,2*w:3*w] = ct3_img        
        res_img[2*h:3*h,3*w:4*w] = ct4_img        
        
        return res_img

# ******************************************************************************
# ***************************************************************** LineDetector
# ******************************************************************************
class LineDetector(ImageProcessor):
    def __init__(self):
        super().__init__( get_default_img() )

        # Add parameters
        _param_blur = IntParameter( id='blur', title='Blur',
                                init_val=5, max_val=33,
                                update_func= lambda x : (x // 2)*2+1 )
        self.add_param( _param_blur )

        _param_low = IntParameter( id='low', title='CannyLow',
                                   init_val = 0, max_val=100,
                                   update_func=self.update_low_cbk )

        self.add_param( _param_low )
        _param_low = IntParameter( id='high', title='CannyHigh',
                                   init_val = 10, max_val=100,
                                   update_func=self.update_low_cbk )
        self.add_param( _param_low )
        _param_ks = IntParameter( id='kernel_size', title='CannyKernel',
                                  init_val = 3, max_val=15,
                                  update_func= lambda x: (x // 2)*2+1 )
        self.add_param( _param_ks )

        _param_inter = IntParameter( id='inter', title='LinesMinInter',
                                init_val=100, max_val=200 )
        self.add_param( _param_inter )
                                     
        
    def update_low_cbk(self, val):
        result = min( val, self.param['high'].val)
        return result
    def update_high_cbk(self, val):
        result = max( val, self.param['low'].val)
        return result

    def run(self, img):
        h,w,d = img.shape
    
        # gray
        gray_img = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY)
        gray_img = np.uint8( gray_img )

        # blurring
        blur_img = cv2.medianBlur( gray_img, self.param['blur'].val )

        detected_edges = cv2.Canny(blur_img,
                                   self.param['low'].val,
                                   self.param['high'].val,
                                   self.param['kernel_size'].val )

        mask = detected_edges != 0
        canny_img = img * (mask[:,:,None].astype(img.dtype))

        # now the lines
        lines = cv2.HoughLines( detected_edges,
                                1, #rho resolution in pixel
                                np.pi/180, #theta in rad
                                self.param['inter'].val, # thres of intersections
                                0, 0)

        # draw the lines
        line_img = img.copy()
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(line_img, pt1, pt2, (0,0,255), 2, cv2.LINE_AA)

        # final image
        res_img = np.zeros( (h, 2*w, 3), dtype="uint8" )
        res_img[:,0:w] = canny_img
        res_img[:,w:2*w] = line_img

        return res_img

# ******************************************************************************
# **************************************************************** VerticalLines
# ******************************************************************************
class VerticalLinesDetector(ImageProcessor):
    def __init__(self):
        super().__init__( get_default_img() )

    def run(self, img):
        h,w,d = img.shape

        # gray
        gray_img = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY)
        gray_img = np.uint8( gray_img )

        # Need a binary inverted image
        gray_img = cv2.bitwise_not(gray_img)
        bw_img = cv2.adaptiveThreshold(gray_img,
                                       255, cv2.ADAPTIVE_THRESH_MEAN_C, \
                                       cv2.THRESH_BINARY, 15, -2)

        vert_img = bw_img.copy()

        # prepare kernel for vertical
        vert_size = h // 10
        vert_kern = cv2.getStructuringElement(cv2.MORPH_RECT, (1, vert_size))

        # apply morphology operation
        vert_img = cv2.erode( vert_img, vert_kern )
        vert_img = cv2.dilate( vert_img, vert_kern )

        res_img = np.zeros( (2 * h, 2 * w), dtype="uint8")
        res_img[0:h,0:w] = gray_img
        res_img[0:h,w:2*w] = bw_img
        res_img[h:2*h,0:w] = vert_img

        return res_img
        

        
