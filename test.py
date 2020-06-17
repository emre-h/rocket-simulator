import datetime
import numpy
import math
import sys

status = "normal"

thrustFilePath = sys.argv[1]

rocketWeight = 19.182
propellanWeight = 3.969

g = 9.807

hMax = 3063

engineThrust = []
times = []

burntMassPerTimePart = round(((propellanWeight / (3.46)) / 1000), 3) # 0.001 saniyedeki ortalama kütle yakımı
#joule = 400

def readThrustValues(filePath):
    with open(filePath, 'r') as data:
        lines = data.read().strip().replace('   ', '').split("\n")

        for i in range(2, len(lines)):
            lineArray = lines[i].split(" ")

            times.append(float(lineArray[0]))
            engineThrust.append(float(lineArray[1]))
                
    return None

readThrustValues(thrustFilePath)

stage = 1

def runPhysics():
    thrust = 0.0
    tBefore = 0.0
    velocity = 0.0
    altitude = 0.0
    acceleration = 0.0
    noiseArea = math.pi*(6**2)
    print(noiseArea)
    k = 0.6
    
    for t in numpy.arange(0.0, 3.47, 0.001):
        t = round(t.item(), 3)

        if t < 3.47:
            if t in times:
                thrust = engineThrust[times.index(t)]
        else:
            thrust = 0

        currentMass = rocketWeight + (propellanWeight - burntMassPerTimePart*t)

        airResistance = k*noiseArea*(velocity*velocity)

        if not thrust == 0:
            acceleration = abs((thrust - airResistance - g*currentMass) / currentMass)
            # print(acceleration)
        else:
            acceleration = -1*g

        deltaT = t - tBefore

        velocity += acceleration * deltaT

        altitude += velocity * deltaT

        tBefore = t

    tVar = velocity/g
    # print(tVar)

    altitude += velocity*(tVar) - (0.5)*g*(tVar*tVar)

    print(velocity)
    print(altitude)

    return None

def recovery():
    status = "recovery"
    print("Recovery mode")
    return 1

def avionicAlgorithm(velocity, altitude, oldAltitude, acceleration):
    print(datetime.datetime.now())
    print("Status" + status)
    print("Velocity" + velocity)
    print("Altitude" + altitude)
    print("Old altitude" + oldAltitude)
    print("Acceleration" + acceleration)

    if velocity < 0 or (oldAltitude - altitude) > 0:
        recovery()

    return None

def startSimulation():

    runPhysics()

    return None


startSimulation()