import json
import math


def getTimeSlots():
    sensorFile = open('2.dat')
    lines = sensorFile.readlines()
    indexMin = 0
    indexMax = len(lines) -1 
    lineMin = lines[ indexMin ]
    lineMax = lines[ indexMax]
    dataMin = lineMin.split()
    dataMax = lineMax.split()
    timeMin = int(dataMin[1])
    timeMax = int(dataMax[1])
    timeMin = timeMin - timeMin % 10 - 30
    timeMax = timeMax - timeMax % 10 + 10
    timeSlots = []
    #print(timeMin, timeMax)
    time = timeMin
    #print(timeMin, timeMax)
    while time <= timeMax : 
        timeSlots.append(time)
        time += 10
    return timeSlots

def getMag(): 
    sensorFile = open('2.dat')
    gyro = {}
    # Load gyro data
    lines = sensorFile.readlines()
    for line in lines:
        data = line.split()
        a = (data[2])
        gyroData = json.loads(a)
        time = data[1]
        time = int(time)
        gyroR = gyroData['vals']
        gyro[time] = gyroR
    return gyro

def getAcc(): 
    sensorFile = open('0.dat')
    gyro = {}
    # Load gyro data
    lines = sensorFile.readlines()
    for line in lines:
        data = line.split()
        gyroData = json.loads(data[2])
        time = data[1]
        time = int(time)
        gyroR = gyroData['vals']
        gyro[time] = gyroR
    return gyro

def getGyro():
    sensorFile = open('4.dat')
    gyro = {}
    # Load gyro data
    lines = sensorFile.readlines()
    for line in lines:
        data = line.split()
        gyroData = json.loads(data[2])
        time = data[1]
        time = int(time)
        gyroR = gyroData['vals']
        gyro[time] = gyroR
    return gyro

def  getOrientation():
    O = {}
    sensorFile = open('16.dat')
    lines = sensorFile.readlines()
    for line in lines:
        data = line.split()
        time = data[1]
        time = int(time)
        QData = json.loads(data[2])
        Q = QData["vals"]
        O[time] = Q
    return O 

if __name__ == '__main__':
    data = {}
    gyro = getGyro()
    mag = getMag()
    acc = getAcc()
    #print(acc)
    orientation = getOrientation()
    #####################
    data = {}
    timeList =  getTimeSlots()
    for time in timeList:
        data[time] = {"orientation":[], "mag":[], "gyro":[], "acc":[]}
    
    for time in gyro:
        resi = time % 10
        if  resi <= 5 :
            timeL = time - time % 10
        else: 
            timeL = time - time % 10 + 10
        data[timeL]["gyro"] = gyro[time]
    for time in mag:
        resi = time % 10
        if  resi <= 5 :
            timeL = time - time % 10
        else: 
            timeL = time - time % 10 + 10
        data[timeL]["mag"] = mag[time]
    for time in acc:
        resi = time % 10
        if  resi <= 5 :
            timeL = time - time % 10
        else: 
            timeL = time - time % 10 + 10
        data[timeL]["acc"] = acc[time]
    for time in orientation:
        resi = time % 10
        if  resi <= 5 :
            timeL = time - time % 10
        else: 
            timeL = time - time % 10 + 10
        data[timeL]["orientation"] = orientation[time]
    #print (data)
    dataFile = open('data.dat','w')
    #dataFile.write(str(data))
    for time in data:
        accD = "acc:" + str(data[time]["acc"]) 
        gyroD = "gyro"

        dataFile.write(str(time)+';'+json.dumps(data[time])+'\n')
