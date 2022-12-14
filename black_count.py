from numba import jit

@jit
def black_check(x1,x2,y1,y2,fragments):
    black_count = 0
    for x in range(x1,x2):
        for y in range(y1,y2):
            if fragments[y][x] == 6: #for black
                black_count += 1

    return black_count