import datetime
import numpy
import math
import sys

thrustFilePath = sys.argv[1]

rocketWeight = 21.537
propellantWeight = 3.969
payloadWeight = 4.5
delimiter = '________________________'
g = 9.807

hMax = 3063

engineThrust = []
times = []

burntMassPerTimePart = propellantWeight / (3.46) / 1000 # 0.001 saniyedeki ortalama kütle yakımı

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
density = 0
cd = 0.421
area = math.pi*((0.06)*(0.06))

def averageThrust():
    r = 0
    for i in engineThrust:
        r += i

    return r/len(engineThrust)

def runPhysics():
    global dt, cd, tBefore, oldAltitude, area, stage, density
    thrust = 0.0
    tBefore = 0.0
    velocity = 0.6
    altitude = 0.0 # location of the barometric sensor
    pressure = 0
    angle = 85
    acceleration = 0.0
    airResistance = 0.0
    temperature = 15
    currentMass = 0.0
    terminalVelocity = 0.0
    landingAltitude = 0.0
    apogee = 0.0
    apogeeTime = 0.0
    status = "normal"

    for t in numpy.arange(0.0, 10000, 0.001): #actually it's going to be a numerical integration
        t = round(t.item(), 3)

        if not status == "landing":

            if t <= 3.461:
                if t in times:
                    thrust = engineThrust[times.index(t)]
            else:
                thrust = 0

            if not thrust == 0:
                # print("BURNT MASS" + str(burntMassPerTimePart*t))
                currentMass = rocketWeight + (propellantWeight - burntMassPerTimePart*t)
            else:
                currentMass = rocketWeight

            if round(altitude,2) % 200 == 0:
                temperature -= 1

            #hpa -> pa
            density = 100*pressure / ((temperature + 273.15)*(287.05))

            calc = (1-((0.0065*altitude)/(temperature + 273.15 + 0.0065*altitude)))

            try:
                pressure = 1013.25*math.pow(calc, 5.257)
            except:
                print(calc)
                break

            airResistance = cd*((velocity*velocity)*density*area)/2

            acceleration = ((thrust - airResistance - g*currentMass) / currentMass)*math.sin(math.pi*angle/180)

            deltaT = t - tBefore

            velocity += (acceleration * deltaT)

            altitude += velocity * deltaT

            # this formula has been taken from https://keisan.casio.com/exec/system/1224579725

            if velocity >= 0 and velocity <= 0.005:
                print("APOGEE")
                print("Time (first stage): " + str(t))
                print("Thrust: " + str(thrust))
                print("Pressure: " + str(pressure))
                print("Temperature: " + str(temperature))
                print("Velocity: " + str(velocity))
                print("Altitude: " + str(altitude))
                print("Acceleration: " + str(acceleration) + "\n")
                print(delimiter + "\n")
                apogee = altitude
                velocity = 0
                apogeeTime = t
                altitude = 1
                airResistance = 0
                status = "landing"
        else:
            if round(landingAltitude,2)  % 200 == 0:
                temperature += 1

            density = 100*pressure / ((temperature + 273.15)*(287.05))

            base = (1-((0.0065*landingAltitude)/(temperature + 273.15 + 0.0065*landingAltitude)))

            pressure = 1013.25*math.pow(base, 5.257)

            airResistance = cd*((velocity*velocity)*density*area)/2

            acceleration = ((-g*currentMass + airResistance) / currentMass)*math.sin(math.pi*angle/180)

            deltaT = t - tBefore

            velocity += (acceleration * deltaT)

            if velocity >= terminalVelocity:
                velocity = terminalVelocity

            altitude += -1*(velocity * deltaT)

            landingAltitude = apogee - altitude

            if landingAltitude >= 0 and landingAltitude <= 1:
                print("LANDED")
                print("Total flight time: " + str(t))
                print("Second stage time: " + str(t - apogeeTime))
                print("Thrust: " + str(thrust))
                print("Pressure: " + str(pressure))
                print("Temperature: " + str(temperature))
                print("Velocity: " + str(velocity))
                print("Altitude: " + str(landingAltitude))
                print("Acceleration: " + str(acceleration) + "\n")
                print(delimiter + "\n")
                stage = 4
                status = "landed"
                break

        primaryAvionicSystem(t, pressure, temperature, acceleration)
        secondaryAvionicSystem(t, acceleration)

        tBefore = t
    # print(velocity)
    # print(altitude)
    # print(landingAltitude)
    return None

primaryRecoveryMode = False
secondaryRecoveryMode = False

def firstRecovery():
    global cd, status, stage, density, area, currentMass

    print("First recovery mode\n")
    # parachute

    area = (1.2)*(1.2)*math.pi
    cd = 0.5
    currentMass = rocketWeight - payloadWeight
    terminalVelocity = math.sqrt((8*currentMass*g)/(density*cd*area))

    print("First terminal velocity: " + str(terminalVelocity) + "\n")

    stage = 2
    status = "recovery"
    
    print(delimiter + "\n")
    return 1

def secondRecovery():
    global cd, status, stage, density, area, currentMass
    print("Second recovery mode\n")
    # parachute
    currentMass = rocketWeight - payloadWeight
    area = (2.5)*(2.5)*math.pi
    cd = 0.5

    terminalVelocity = math.sqrt((8*currentMass*g)/(density*cd*area))

    print("Second terminal velocity: " + str(terminalVelocity) + "\n")
    print(delimiter + "\n")

    stage = 3
    status = "recovery"
    return 1

oldAltitude = 0
dt = 0.001

recCheck2 = False

def primaryAvionicSystem(t,pressure, temperature, accelerationY):
    global dt, cd, oldAltitude, primaryRecoveryMode, stage, recCheck2

    #FINDING VELOCITY FROM ALTITUDE

    altitude = ((math.pow(1013.25/pressure, 1/5.257)-1)*(temperature + 273.15)) / 0.0065

    velocity = (altitude - oldAltitude) / dt

    if not primaryRecoveryMode and velocity >= -2 and velocity <= 0 and altitude < 3060 and altitude > 3000:
        print("PRIMARY AVIONIC - FIRST RECOVERY MODE - SIGNAL RECEIVED")
        print("Time: " + str(t))
        print("Pressure: " + str(pressure))
        print("Temperature: " + str(temperature))
        print("Velocity: " + str(velocity))
        print("Altitude: " + str(altitude))
        print("Acceleration: " + str(accelerationY) + "\n")

        primaryRecoveryMode = True
        firstRecovery()

        oldAltitude = altitude
        return 1
    elif primaryRecoveryMode and not recCheck2 and velocityVector < 0 and altitudeVector > 450 and altitudeVector < 600:
        print("SECONDARY AVIONIC - SECOND RECOVERY MODE SIGNAL RECEIVED")
        print("Time: " + str(t))
        print("Calculated altitude: " + str(altitudeVector))
        print("Velocity: " + str(velocityVector))
        print("Acceleration: " + str(accelerationY) + "\n")

        recCheck2 = True
        secondRecovery()

        oldAltitude = altitude
        return 1

    oldAltitude = altitude

    return 0

velocityVector = 0
altitudeVector = 0

recCheck = False

def secondaryAvionicSystem(t, accelerationY):
    global velocityVector, altitudeVector, secondaryRecoveryMode, dt, recCheck

    #FINDING VELOCITY AND ALTITUDE FROM ACCELERATION

    velocityVector += accelerationY*dt

    altitudeVector += velocityVector*dt

    if not secondaryRecoveryMode and velocityVector >= -3 and velocityVector <= -1 and altitudeVector < 3060 and altitudeVector >= 3000:
        print("SECONDARY AVIONIC - FIRST RECOVERY MODE SIGNAL RECEIVED")
        print("Time: " + str(t))
        print("Calculated altitude: " + str(altitudeVector))
        print("Velocity: " + str(velocityVector))
        print("Acceleration: " + str(accelerationY) + "\n")

        secondaryRecoveryMode = True
        firstRecovery()
        return 1
    elif secondaryRecoveryMode and not recCheck and velocityVector < 0 and altitudeVector > 450 and altitudeVector < 600:
        print("SECONDARY AVIONIC - SECOND RECOVERY MODE SIGNAL RECEIVED")
        print("Time: " + str(t))
        print("Calculated altitude: " + str(altitudeVector))
        print("Velocity: " + str(velocityVector))
        print("Acceleration: " + str(accelerationY) + "\n")

        recCheck = True
        secondRecovery()
        

    return 0


def startSimulation():

    runPhysics()

    return None


startSimulation()