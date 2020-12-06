import grid_generator as gg
import numpy as np
import random
#695.2036  2227.3872
#695.1987  1830.3673
#1100.9381  1830.4353
#1100.6505  2227.0947

def euclideanDistance(posA, posB):
    vector1 = np.array(posA)
    vector2 = np.array(posB)
    dis=np.linalg.norm(vector1-vector2)
    return dis

def mazeGridToPixel(x,y):
    return [x * 13.33 + 695, y*13.33+1830]

def pixelToMaze(x,y):
    return[int((x-695)/13.58), int((y-1830)/13.33)]
def mazeToPixels(maze):
    pixels = []
    for j in range(len(maze)):
        for i in range(len(maze[0])):
            pixels.append([j*13.33+695, i*13.33+1830])
    return pixels

def isValidPoint(candidatePoint, valid_points):
    for point in valid_points:
        if euclideanDistance(point,candidatePoint)<= 18:
            return True
    return False

def generateMazeData():
    grids = gg.generateGridPoints(13.33, ["shape_description.shape","shape_description1.shape"])
    valid_points = gg.quickDelete(grids, 5)
    #print(valid_points)
    debug = False
    if debug:
        import matplotlib.pyplot as plt
        for grid in valid_points:
            plt.scatter(grid[0], grid[1],color='blue')
        plt.show()  
        plt.clf()
    maze = []
    x_cursor = 695
    while x_cursor < 1105:
        bufferRaw = []
        y_cursor = 1830
        while y_cursor < 2227:
            if isValidPoint([x_cursor,y_cursor],valid_points):

                bufferRaw.append(0)
            else:
                bufferRaw.append(1)
            y_cursor+=13.33
        maze.append(bufferRaw)
        x_cursor+= 13.33
    return maze

if __name__=='__main__':
    debug = True
    grids = gg.generateGridPoints(13.33, ["shape_description.shape","shape_description1.shape"])
    valid_points = gg.quickDelete(grids, 5)
    #print(valid_points)
    if debug:
        import matplotlib.pyplot as plt
        for grid in valid_points:
            plt.scatter(grid[0], grid[1],color='blue')
        plt.show()  
        plt.clf()
    maze = []
    x_cursor = 695
    while x_cursor < 1105:
        bufferRaw = []
        y_cursor = 1830
        while y_cursor < 2227:
            if isValidPoint([x_cursor,y_cursor],valid_points):
                bufferRaw.append(0)
            else:
                bufferRaw.append(1)
            y_cursor+=13.58
        maze.append(bufferRaw)
        x_cursor+= 13.58

    pixels = mazeToPixels(maze)
    #print(pixels)
    if debug:
        import matplotlib.pyplot as plt
        for grid in pixels:
            if isValidPoint(grid,valid_points):
                plt.scatter(grid[0], grid[1],color='red')
            else:
                plt.scatter(grid[0], grid[1],color='blue')
        plt.show()  
