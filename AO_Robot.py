 #!/usr/bin/python2.7
#Made by Jonatan Gustavsson, kv14jgn & Kristoffer Karlsson, kv14kkn
#5DV121 HT16.
# -*- coding: utf-8 -*-
MRDS_URL = 'localhost:50000'

import httplib, json, time
from math import sin,cos,pi,atan2,sqrt

HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

def postSpeed(angularSpeed,linearSpeed):
    """Sends a speed command to the MRDS server"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    params = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
    mrds.request('POST','/lokarria/differentialdrive',params,HEADERS)
    response = mrds.getresponse()
    status = response.status
    #response.close()
    if status == 204:
        return response
    else:
        raise UnexpectedResponse(response)
    
    
def getPose():
    """Reads the current position and orientation from the MRDS"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/localization')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData)
    else:
        return UnexpectedResponse(response)
    
def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into 
    a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData)
    else:
        return response
    
def getBearing():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

def bearing(q):
    return rotate(q,{'X':1.0,'Y':0.0,"Z":0.0})

def rotate(q,v):
    return vector(qmult(qmult(q,quaternion(v)),conjugate(q)))

def quaternion(v):
    q=v.copy()
    q['W']=0.0;
    return q

def vector(q):
    v={}
    v["X"]=q["X"]
    v["Y"]=q["Y"]
    v["Z"]=q["Z"]
    return v

def conjugate(q):
    qc=q.copy()
    qc["X"]=-q["X"]
    qc["Y"]=-q["Y"]
    qc["Z"]=-q["Z"]
    return qc

def qmult(q1,q2):
    q={}
    q["W"]=q1["W"]*q2["W"]-q1["X"]*q2["X"]-q1["Y"]*q2["Y"]-q1["Z"]*q2["Z"]
    q["X"]=q1["W"]*q2["X"]+q1["X"]*q2["W"]+q1["Y"]*q2["Z"]-q1["Z"]*q2["Y"]
    q["Y"]=q1["W"]*q2["Y"]-q1["X"]*q2["Z"]+q1["Y"]*q2["W"]+q1["Z"]*q2["X"]
    q["Z"]=q1["W"]*q2["Z"]+q1["X"]*q2["Y"]-q1["Y"]*q2["X"]+q1["Z"]*q2["W"]
    return q

def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData)
    else:
        return response
    
def path_menu():
    """Prints out a menu for the user. Takes the users input (an int). 
    Associates it with a filename.
    Input: none
    Output: string. eg. filename"""
    user_input = raw_input("Select path: \n 1. Path around table and back."+
                           "\n 2. Path to bed. \n 3. Type in filename."+
                           "\n...: ")
        
    filename = give_file(int(user_input))
    return str(filename)
    
def give_file(user_input):
    """Associates a number between 1-3 with a filename. 3 is a user given 
    filename
    input: an int
    Output: a filename"""
    
    filename= str()
    if user_input == 1:
        filename = "Path-around-table-and-back.json"
    elif user_input == 2:
        filename =  "Path-to-bed.json"
    elif user_input == 3:
        filename =  raw_input("New file: ")
    return filename

def open_json(filename):
    """Opens the .jasonfile and extracts the content.
    Input: string, eg. filename
    Output: the files content"""
    
    file_content = json.loads(open(filename).read())    
    
    return file_content

def get_path(filenamne):
    """Takes a filename, uses function open_jason to open the file. This 
    function then extracts the cordinates in the path.
    Input: filename
    Output: array of dictionaries repressenting cordinates"""
    
    path = open_json(filenamne)
    cordinates = []
    for i in range(len(path)):
        cordinates.append(path[i]["Pose"]["Position"])
    return cordinates 

def get_diff(param1, param2):
    """Calculates the difference between two numbers
    input: two ints or flots
    Output: the result"""
    
    result = (param1 - param2)
    return result

def calc_angle(point_Y, point_X):
    """Claculates the angle the robot has to turn in radius
    Input: a X and a Y.
    Output: the angle between X and Y"""
    
    angle = atan2(point_Y, point_X)
    return angle
    

    
    
def calc_distance(path_cord, robot_cord):
    """Calculates the distance from robot to cordinate
    Input: Two cordinates
    Output: the distance between them"""
    px = path_cord["X"] 
    rx = robot_cord["X"]
    py = path_cord["Y"]
    ry = robot_cord["Y"]
    
    dx = (px - rx)
    dy = (py - ry)
    
    distance = sqrt((dx**2) + (dy**2))
    
    return distance


def get_robot_angle():
    """Returns the angle in wich the robot is heading
    Input: none
    Output: The robots heading"""
    
    rob_bearing = getBearing()
    rob_angle = calc_angle(rob_bearing["Y"], rob_bearing["X"])
    return rob_angle
    
    


def turn(rad, forw_speed):
    """Tells the robot how much to turn and go forward. Forwards this message to
    postSpeed.
    Input: a radian eg the turn speed and a forward speed
    output: none"""
    postSpeed(rad, forw_speed)
    
def get_laser_info():
    """Get information from the robots lasers and divids the info into a 
    right-beam list and a left beam list. 
    Input: none
    Output: a 2D list containting the two beam-lists"""
    
    lasers = getLaser()
    lasers = lasers["Echoes"]
        
    echoes = []
    right = []
    left = []
    for i in range(100, 180, 1):
        
        if i >= 100 and i <= 140:
            right.append(lasers[i])
        elif i>140 and i<=180:
            left.append(lasers[i])
    
    echoes.append(right)
    echoes.append(left)
            
    return echoes 



filename = path_menu()
path_cordinates = get_path(filename)


counter = 1
forward_speed = 1
sleep_time = 0.07

#takes a time stamp from epoch in UTC.
start_time = time.time()

#A while loop controlling the robot.
while counter !=len(path_cordinates):
    robot_pos = getPose()
    robot_pos = robot_pos["Pose"]["Position"]    
    
    #Calculates the angle between the robots center and the path coordinate
    x = get_diff(path_cordinates[counter]["X"], robot_pos["X"])
    y = get_diff(path_cordinates[counter]["Y"], robot_pos["Y"])
    angle_to_point = calc_angle(y,x)
    
    distance = calc_distance(path_cordinates[counter], robot_pos)
    
    #The look ahead distance. If it is to short, check next point.
    if distance < 1.4:
        counter +=1
    
    
    else: 
        #Calculates the robots heading and the the total angle.
        robot_angle = get_robot_angle()
        total_angle = angle_to_point - robot_angle
        total_degree = (total_angle*(180/pi))
        
        #Depending on the total angle, the robot turns right, or left.
        if total_degree >= 180 or total_degree < -180:
            
            #Calates how fast the robot has to turn.
            total_angle = robot_angle - angle_to_point
            turn(total_angle, forward_speed)
            
            counter += 1
        else:
            #Calculates how fast the robot has to turn
            total_angle = angle_to_point - robot_angle
            turn(total_angle, forward_speed)
            counter +=1
        time.sleep(sleep_time)
        
        #Gets laser information to avoid obstacles.
        echoes = get_laser_info()
        
        #Find closest obstacle on every side.
        min_right = min(echoes[0])
        min_left = min(echoes[1])        
        
        #Depending on closest obstacle the robot will turn accordingly, if
        #nessecary.
        if min_right < 0.5:
            turn(1, 0)
            time.sleep(0.4)
        elif min_left < 0.5:
            turn(-1, 0)
            time.sleep(0.4)

# takes another timestamp and subtract the two timestapms to get the time.
stop_time = time.time()
postSpeed(0,0)

lap_time = stop_time - start_time

print ("Time: %.3f" )%(lap_time)


