import datetime
import numpy
import math
import sys

status = "normal"

thrustFilePath = sys.argv[1]

rocketWeight = 21.537
propellanWeight = 3.969

g = 9.807

hMax = 3063

engineThrust = []
times = []

burntMassPerTimePart = propellanWeight / (3.46) / 1000 # 0.001 saniyedeki ortalama kütle yakımı

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

def averageThrust():
    r = 0
    for i in engineThrust:
        r += i

    return r/len(engineThrust)

def runPhysics():
    thrust = 0.0
    tBefore = 0.0
    velocity = 0.6
    altitude = 0.0
    pressure = 0
    density = 0
    angle = 85
    acceleration = 0.0
    airResistance = 0.0
    noiseArea = math.pi*((0.06)*(0.06))
    temperature = 15
    cd = 0.42
    currentMass = 0.0

    for t in numpy.arange(0.0, 100, 0.001):
        t = round(t.item(), 3)

        if t <= 3.461:
            if t in times:
                thrust = engineThrust[times.index(t)]
        else:
            thrust = 0

        if not thrust == 0:
            # print("BURNT MASS" + str(burntMassPerTimePart*t))
            currentMass = rocketWeight + (propellanWeight - burntMassPerTimePart*t)
        else:
            currentMass = rocketWeight

        if altitude % 200 == 0:
            temperature -= 1

        acceleration = ((thrust - airResistance - g*currentMass) / currentMass)*math.sin(math.pi*angle/180)

        deltaT = t - tBefore

        velocity += (acceleration * deltaT)

        #hpa -> pa
        density = 100*pressure / ((temperature + 273.15)*(287.05))

        airResistance = cd*((velocity*velocity)*density*noiseArea)/2

        altitude += velocity * deltaT

        base = (1-((0.0065*altitude)/(temperature + 273.15 + 0.0065*altitude)))

        print("Base:" + str(base))
        print("Altitude: " + str(altitude))
        
        pressure = 1013.25*math.pow(base, 5.257)
        # this formula has been taken from https://keisan.casio.com/exec/system/1224579725

        if velocity >= 0 and velocity <= 0.005:
            print("APOGEE")
            print("Time: " + str(t))
            print("Thrust: " + str(thrust))
            print("Pressure: " + str(pressure))
            print("Temperature: " + str(temperature))
            print("Velocity: " + str(velocity))
            print("Altitude: " + str(altitude))
            print("Acceleration: " + str(acceleration))
            break

        #print("Time: " + str(t))
        #print("Thrust: " + str(thrust))
        #print("Pressure: " + str(pressure))
        #print("Temperature: " + str(temperature))
        #print("Velocity: " + str(velocity))
        #print("Altitude: " + str(altitude))
        #print("Acceleration: " + str(acceleration))
        #print("\n")

        tBefore = t

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