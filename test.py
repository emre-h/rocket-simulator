import datetime
import numpy
import math
import sys

thrustFilePath = sys.argv[1]

rocketWeight = 21.537
propellantWeight = 3.969

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
cd = 0.421
area = math.pi*((0.06)*(0.06))

def averageThrust():
    r = 0
    for i in engineThrust:
        r += i

    return r/len(engineThrust)

def runPhysics():
    global dt, cd, tBefore, oldAltitude, area, stage

    payloadWeight = 4.5
    thrust = 0.0
    tBefore = 0.0
    velocity = 0.6
    altitude = 0.0 # location of the barometric sensor
    pressure = 0
    density = 0
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

            if altitude % 200 == 0:
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
                apogee = altitude
                velocity = 0
                apogeeTime = t
                altitude = 1
                currentMass = rocketWeight - payloadWeight
                airResistance = 0
                status = "landing"
                terminalVelocity = math.sqrt((8*currentMass*g)/(density*cd*area))
                print("Terminal velocity: " + str(terminalVelocity) + "\n")
        else:
            if landingAltitude % 200 == 0:
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

primaryRecvoeryMode = False
secondaryRecvoeryMode = False

def firstRecovery():
    global cd, status, stage
    # parachute
    area = (1.2)*(1.2)*math.pi
    cd = 0.5
    stage = 2
    status = "recovery"
    print("Recovery mode\n")
    return 1

oldAltitude = 0
dt = 0.001

def primaryAvionicSystem(t,pressure, temperature, accelerationY):
    global dt, cd, oldAltitude,primaryRecvoeryMode ,stage

    #FINDING VELOCITY FROM ALTITUDE

    altitude = ((math.pow(1013.25/pressure, 1/5.257)-1)*(temperature + 273.15)) / 0.0065

    velocity = (altitude - oldAltitude) / dt

    if not primaryRecvoeryMode and velocity >= -2 and velocity <= 0 and altitude < 3060 and altitude > 3000:
        print("PRIMARY AVIONIC - RECOVERY MODE SIGNAL RECEIVED")
        print("Time: " + str(t))
        print("Pressure: " + str(pressure))
        print("Temperature: " + str(temperature))
        print("Velocity: " + str(velocity))
        print("Altitude: " + str(altitude))
        print("Acceleration: " + str(accelerationY) + "\n")
        primaryRecvoeryMode = True
        firstRecovery()
        return 1

    oldAltitude = altitude

    return 0

velocityVector = 0
altitudeVector = 0

def secondaryAvionicSystem(t, accelerationY):
    global velocityVector, altitudeVector, secondaryRecvoeryMode, dt

    #FINDING VELOCITY AND ALTITUDE FROM ACCELERATION

    velocityVector += accelerationY*dt

    altitudeVector += velocityVector*dt

    if not secondaryRecvoeryMode and velocityVector >= -3 and velocityVector <= -1 and altitudeVector < 3060 and altitudeVector >= 3000:
        print("SECONDARY AVIONIC - RECOVERY MODE SIGNAL RECEIVED")
        print("Time: " + str(t))
        print("Calculated altitude: " + str(altitudeVector))
        print("Velocity: " + str(velocityVector))
        print("Acceleration: " + str(accelerationY) + "\n")
        secondaryRecvoeryMode = True
        firstRecovery()
        return 1

    return 0


def startSimulation():

    runPhysics()

    return None


startSimulation()