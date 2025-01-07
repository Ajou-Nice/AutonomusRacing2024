import cv2
import numpy as np
from scipy.stats import binned_statistic_2d


# 1409.032384 0.000000 942.625540
# 0.000000 1412.254216 604.104805
# 0.000000 0.000000 1.000000

FX = 1409.032384  # pixle/mm
FY = 1412.254216   # pixle/mm
CX = 942.625540  # pixlel
CY = 604.104805  # pixlel

NEW_CX=862.0


TARGET_SIZE = 600

C2W = [
    [ 1, 0 , 0, -0.15],
    [ 0, 1, 0, 0],
    [ 0, 0, 1, 0.24],
    [ 0, 0, 0, 1]
]



def caculate_local_postion( box_msg ):

    center_x = box_msg.bbox.center.x
    size_y = box_msg.bbox.size_y

    pos_y = (TARGET_SIZE*FY)/size_y
    pos_x = ((center_x - CX)/FY)* pos_y
    # print ("size:",box_msg.bbox.size_x , box_msg.bbox.size_y)
    # print( "pixel pos :" ,box_msg.bbox.center.x , box_msg.bbox.center.y)
    # print("local pos  :" ,pos_y/1000, pos_x/1000)
    return (pos_y/1000)-0.24, (pos_x/1000)+0.15


