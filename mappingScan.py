import serial
import math

import pandas as pd

import numpy as np
import open3d as o3d

import pyvista as pv
from pyvista import examples
                                                                                                                                                         
#constants
COM_NUMBER = "7"
numberMeasurements = 32
numberFrames = 1

# change port as needed
ser = serial.Serial(port="COM"+COM_NUMBER, baudrate=115200, bytesize=8, timeout=10, stopbits=serial.STOPBITS_ONE)

# returns list [x, y]                                       
def getXY(degrees, rawDistance, j):
    x = rawDistance * math.cos(math.radians(degrees))
    y = rawDistance * math.sin(math.radians(degrees))
    z = j

    return [x, y, z]

# determine if a message is data
def isData(string):
    try:
        list(map(int, string.split(", ")))
        return 1
    except:
        return 0

# reads serial port for all its data
def getData(successfulReadsToEndAt=numberMeasurements):
    allData = []
    successfulReads = 0

    if not ser.is_open:
        ser.open()

    while successfulReads != successfulReadsToEndAt:
        message = ser.readline() # get bytes
        message = message.decode()
        message = message.strip("\r\n") # strip garbage characters

        if not isData(message):
            print("GARBAGE:", message)
            continue

        data = list(map(int, message.split(", "))) # get the data, if its numbers. If it is not numbers it will raise exception
        print("DATA:", data)

        # the data being collected
        rangeStatus, distance, signalRate, ambientRate, spadNb = data

        allData.append(data)
        successfulReads += 1
    
    ser.close()
    print("Returning:", allData)
    return allData
    
allXYCoords = []
def collect(z):
# main
    data = getData(numberMeasurements)
    #allXYCoords = []

    degs = 0
    for i in range(numberMeasurements):
        x, y, zVar = getXY(degs, data[i][1], z)
        print(x, y, zVar)
        allXYCoords.append([x, y, zVar])
        degs += 11.25

    print(allXYCoords)

#def main():
for i in range(numberFrames):
    j = i*100
    collect(j)
    
if __name__ == "__main__":
    #Remember the goals of modularization
    #   -- smaller problems, reuse, validation, debugging
    #To simulate the data from the sensor lets create a new file with test data 
    f = open("demofile2dx.xyz", "w")    #create a new file for writing 
    
        #Test data: Lets make a rectangular prism as a point cloud in XYZ format
        #   A simple prism would only require 8 vertices, however we
        #   will sample the prism along its x-axis a total of 10x
        #   4 vertices repeated 10x = 40 vertices
        #   This for-loop generates our test data in xyz format
        
    for j in range(numberFrames*numberMeasurements):
        f.write(str(int(allXYCoords[j][0])) + ' ' + str(int(allXYCoords[j][1])) + ' ' + str(int(allXYCoords[j][2])) + '\n')    #write x,0,0 (xyz) to file as p1
    f.close()   #there should now be a file containing 40 vertex coordinates                               
    
        #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("demofile2dx.xyz", format="xyz")

        #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

        #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])

        #OK, good, but not great, lets add some lines to connect the vertices
        #   For creating a lineset we will need to tell the packahe which vertices need connected
        #   Remember each vertex actually contains one x,y,z coordinate

        #Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0,numberFrames*numberMeasurements):
        yz_slice_vertex.append([x])

        #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,numberFrames*numberMeasurements, numberMeasurements):
        for i in range(numberMeasurements):
            if i==numberMeasurements-1:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x]])
            else:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+1]])
                    
        #lines.append([yz_slice_vertex[x], yz_slice_vertex[(x+1)%8]])

        #Define coordinates to connect lines between current and next yz slice
        
    for x in range(0,numberFrames*numberMeasurements-numberMeasurements-1,numberMeasurements):
        for i in range(numberMeasurements):
            lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+numberMeasurements]])

        #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

        #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])

    #df1 = pd.DataFrame(allXYCoords)
    #df1.to_excel("output.xlsx")  
