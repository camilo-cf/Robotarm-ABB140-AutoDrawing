"""
MIT License

Copyright (c) 2020 Camilo A. Cáceres

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import cv2
import numpy as np
import math
import time
import vrep
from tsp_solver.greedy import solve_tsp


################################################################################
#########################           Functions            #######################
################################################################################

def distance(x1, y1, x2, y2):
    '''
    This fucntion calculates the  Euclidian distance between 2 points

    Args:
        x1 (float): X value of the first point
        y1 (float): Y value of the first point
        x2 (float): X value of the second point
        y2 (float): Y value of the secons point

    Returns:
        dist (float): Euclidian distance between point 1 and 2
    '''
    dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    return dist

# 
def Start(IP='127.0.0.1', PORT=19999):# Local IP and API address (19999)
    '''
    This fucntion starts the communication with the simulator
    Important: 
        First click play on the simulator then run his program
        Verify the vrep requirements of the remote API library: 
            "remoteApi.dll" (Windows)   
            "remoteApi.dylib" (Mac) 
            "remoteApi.so" (Linux)
            In Windows:
            C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\lib
        Put the required requirement in the same directory of this app
    
    By default it should run with the default VREP configuration
    '''
    vrep.simxFinish(-1) # Finish all the connections
    clientID=vrep.simxStart(IP, PORT, True, True, 5000, 5) # Start a new connection in the VREP default port 19999 
    print('Connection stablished')
    return clientID


def locate_pen(x, y, z, target, plane, clientID):
    '''
    VREP locate pen function on the position x, y, z on the space
    taking as reference targer and plane
    '''
    vrep.simxSetObjectPosition(clientID, target, plane, [x,y,z], vrep.simx_opmode_oneshot_wait)

# Image processing (get edges) and path optimization 
def image_processing_optimization(img = 'Images/LAIR.jpg', factor = 1, size = 350):
    '''
    This function process an image obtaining the edges ussing canny.
    It also optimizes the path that will be draw using an ecternal library (tsp-solver).
    Args:
        img (str): 
            The desired image source location
        factor (int): 
            Reduction or scaling factor (1 is normal, less for reduce (ex 0.5), more increases (1.5))
        size (int): 
            The size of the draw (in the given simulation 200 is little and 350 maximum) - but depends of the draw size also

    Returns:
        factor(int): 
            Reduction or scaling factor (1 is normal, less for reduce, more increases)
        points (list): 
            List of the draw points lists of [x, y] positions
        path (list) : 
            Optimized sequence of path for the points list
        
    Example:
        image_processing_optimization('Images/LAIR.jpg', 1, 200)
    '''
    print("Program running")
    rgb_img = cv2.imread(img) 
    x, y, _ = rgb_img.shape
    max_value = max(x, y)

    desired_size = [size, size]
    desired =   [desired_size[0]*(y/max_value), 
                desired_size[1]*(x/max_value), 
                3]

    scale = [desired[0]/y,
            desired[1]/x]

    rgb_img = cv2.resize(rgb_img, 
                        None,
                        fx=scale[0], 
                        fy=scale[1], 
                        interpolation = cv2.INTER_CUBIC)

    gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray_img, 0, 255, apertureSize = 3)
    cv2.imshow('Image', edges)
    print("Image Processed")

    new_x, new_y, _ = rgb_img.shape
    edges = edges.tolist()

    point_x=[]
    point_y=[]

    for row in range(1, new_y):
        for column in range(1, new_x):
            if edges[column][row] == 255:
                point_x.append(row)
                point_y.append(column)

    points = list(zip(point_x, point_y))
    r  = [[0 for x in range(len(point_x))] for y in range(len(point_x))]

    print(str(len(point_x))+" Points Obtained")

    for p1 in range(1,len(point_x)):   
        for p2 in range(1,len(point_x)):
            x1, y1= points[p1]
            x2, y2= points[p2]
            r[p1][p2]=distance(x1,y1,x2,y2)

    print("Distances calculated")        

            
    #TSP - library
    print("Solving TSP")
    path = solve_tsp(r)
    print("TSP Done")

    np_points = np.array(points)
    np_points = np_points*factor
    points = np_points.tolist()

    return factor, path, points, new_x, new_y

def draw_VREP(factor, path, points, Z_draw, x, y):
    '''
    This function draws on VREP the specified draw

    Args:
        factor(int): 
            Reduction or scaling factor (1 is normal, less for reduce, more increases)
        points (list): 
            List of the draw points lists of [x, y] positions
        path (list) : 
            Optimized sequence of path for the points list
        Z_draw (float):
            Drawing plane Height Constant - According to the VREP model
    '''
    # Start VREP connection
    clientID= Start()

    # Identify the VREP environment objects
    _, plane = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_oneshot_wait) # Reference point on the drawing plane
    _, target = vrep.simxGetObjectHandle(clientID, 'IRB140_target', vrep.simx_opmode_oneshot_wait) # End effector joint Reference

    print("Drawing Optimizated image")
    locate_pen(0, 0, Z_draw+0.2, target, plane, clientID)
    cv2.waitKey(2)

    img_result = np.zeros((x*factor, y*factor, 3), np.uint8)
    for each in range(1,(len(path))-1):
        x1, y1 = points[path[each]]
        x2, y2 = points[path[each+1]]
        if distance(x1, y1, x2, y2) <= 2:
            locate_pen(y1/1000, x1/1000, Z_draw, target, plane, clientID)
            time.sleep(0.01)
            # Draw the progress on an external canvas 
            cv2.line(img_result, 
                    tuple(points[path[each]]), 
                    tuple(points[path[each+1]]), 
                    (255, 0, 0), 
                    1)
            cv2.imshow('Optimized', img_result)
            cv2.waitKey(1)
            locate_pen(y2/1000, x2/1000, Z_draw, target, plane, clientID)
            time.sleep(0.01)
            cv2.waitKey(1)
        else:
            time.sleep(0.01)
            locate_pen(y1/1000, x1/1000, Z_draw+0.001, target, plane, clientID)
            time.sleep(0.01)
            locate_pen(y2/1000, x2/1000, Z_draw+0.001, target, plane, clientID)
            time.sleep(0.05)

    # Go to the initial position
    locate_pen(0, 0, Z_draw+0.2, target, plane, clientID) 
    print("Draw finished")

def script_RAPID_draw(module_name, reference_point_name, tool_name, points, vel=5, factor=1):
    '''
    This function creates an ABB RAPID language program to draw the given path
    This doesnt work with VREP (CoppeliaSim)
    This is only to get the code and put it into the real ABB robot ABB 140
    Be careful because this script is adjusted to MY robot axis settings, calibration, tools and configurations
    USE IT CAREFULLY!!!!!!!!

    Args:
        module_name (str): Name of the program (how it will be saved)
        reference_point_name (str):  Name of the reference point in the ABB robot settings
        tool_name (str): Name of the tool in the ABB robot settings
        points (list): Path list
        vel (int): speed of the robot  (BE CAREFUL!!!!! start with a low number and the adjust)
        factor (int): factor to size of the draw 
    '''

    array_x=[]
    array_y=[]
    array_z=[]

    for point in range(0,len(points)-1):
        x1=points[path[point]][0]/2
        y1=points[path[point]][1]/2
        x2=points[path[point+1]][0]/2
        y2=points[path[point+1]][1]/2
        
        if distance(x1,y1,x2,y2)<= 1*factor:
            array_x.append(str(x1))
            array_y.append(str(y1))
            array_z.append(str(-15))
            
            array_x.append(str(x2))
            array_y.append(str(y2))
            array_z.append(str(-15))            
            
        else:
            array_x.append(str(x1))            
            array_y.append(str(y1))
            array_z.append(str(10))
            
            array_x.append(str(x2))
            array_y.append(str(y2))
            array_z.append(str(10))            

    try:
        file  = open(module_name+".prg", "r+")        
    except (FileNotFoundError):
        file  = open(module_name+".prg", "w")
        file  = open(module_name+".prg", "r+")        

    open(module_name+".prg", 'w').close()

    #HEADER
    file.write("%%%\n  VERSION:1\n  LANGUAGE:ENGLISH\n%%%\n")
    file.write("\nMODULE "+module_name+"\n")

    #CONSTANTS FOR MY ROBOT CONFIGURATION/ CALIBRATION
    file.write("\n! UPDATE reference point")
    ## Reference Point
    ## file.write("\nCONST robtarget P1:=[[572.06,42.23,447.88],[0.355512,-0.378941,0.762498,-0.385504],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];")
    file.write("\nCONST robtarget P1:=[[639.4,-119.9,378.9],[0.13334, -0.23621, 0.96143, -0.04554],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];")
    file.write("\n! UPDATE tool point")
    file.write("\nPERS tooldata "+tool_name+":=[TRUE,[[0.500839,-0.574904,226.276],[1,0,0,0]],[0.25,[85,0,65],[1,0,0,0],0.01,0.01,0.01]];")


##    file.write("\nCONST num number_data:="+str(len(array_x))+";")
    file.write("\nVAR num array_draw_x{"+str(len(array_x))+"}:= ["+ (', '.join(array_x))+"];")
    file.write("\nVAR num array_draw_y{"+str(len(array_y))+"}:= ["+ (', '.join(array_y))+"];")
    file.write("\nVAR num array_draw_z{"+str(len(array_z))+"}:= ["+ (', '.join(array_z))+"];")
    
    file.write("\n\tPROC main()")
    file.write("\n\t\tMoveAbsJ [[45,0,0,0,90,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]"+r'\NoEOffs,v100,z50,Bic;')
    file.write("\n\t\tMoveJ Offs("+reference_point_name+',0,0,100),v100'+",fine,"+tool_name+";")
   
    file.write("\n\t\tFOR i FROM 1 TO Dim(array_draw_x, 1) DO")
    file.write("\n\t\t\tMoveL Offs ("+reference_point_name+",array_draw_x{i},array_draw_y{i},array_draw_z{i}), v"+str(vel)+",z1,Bic;")
    file.write("\n\t\tENDFOR")


    file.write("\n\t\tMoveL Offs"+" ("+str(reference_point_name)+",0,0,100),v"+str(vel)+",z10,"+tool_name+";")

    file.write('\n\t\tWaitTime 2;')
    file.write("\n\tENDPROC")
    file.write("\nENDMODULE")
    file.close()

################################################################################



################################################################################
########################              Main               #######################
################################################################################
if __name__ == "__main__":
    # Drawing plane Height Constant - According to the VREP model
    Z_draw=0.1065 

    # Define image and process it
    # Bigger size better results, but it takes longer to process the TSP optimization
    # Lower size can lead to errors, but takes shortest time to process
    factor, path, points, x, y = image_processing_optimization(img = 'Images/LAIR.jpg', factor = 1, size = 350) # Restriction -> factor*size < 350

    # Create the ABB Rapid code
    script_RAPID_draw("example", "P1", "Bic", points, 100, 3)

    # Start Drawing
    draw_VREP(factor, path, points, Z_draw, x, y) 

    # Wait until a key press
    cv2.waitKey()
################################################################################