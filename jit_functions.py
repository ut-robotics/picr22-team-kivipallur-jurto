from numba import jit
import numpy as np

@jit
def black_check(x1,x2,y1,y2,fragments):
    black_count = 0
    for x in range(x1,x2):
        for y in range(y1,y2):
            if fragments[y][x] == 6: #for black
                black_count += 1

    return black_count

@jit
def median_depth(x,y,depth_frame):
    basket_area = []
    for i in range(x-5,x+5):
        for j in range (y-5,y+5):
            basket_area.append(depth_frame[j][i])
    
    return basket_area
