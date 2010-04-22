#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import time
import sys
import glHelp
import math
import numpy as np

from collections import deque

from rigidBody import RigidBody, radians
from articulatedBody import ArticulatedBody

paused = False
ESCAPE_KEY = '\033'

windowSize = (800, 800)
worldSize = (20.0, 20.0)

displayFramerate = 60.0 #fps
simulationFramerate = 600.0 #fps

stepsPerFrame = float(simulationFramerate)/displayFramerate

simulationTime = time.time()

mouseAccel = (0.0,0.0)
currentMousePos = (0,0)
#two blocks, one fixed to the ground, with one point-point
#constraint

size1 = (1.0, 5.0)
mass1 = .5

size2 = (2.0, 3.0)
mass2 = 1.0

r1 = [0, -size1[1]/2.0]
r2 = [0, size1[1]/2.0]
r3 = [0, -size2[1]/2.0]

initialPosition1 = ((worldSize[0] + size1[0])/2.0, size1[1]/2.0 + worldSize[1]/2.0)
initAngle = 10.0
initialPosition2 = (initialPosition1[0]  -
                    .5*size2[1]*math.sin(radians(initAngle)),
                    initialPosition1[1] + size1[1]/2.0 +
                    .5*size2[1]*math.cos(radians(initAngle))
                    )

actuationTorque = 0.0

gravity = (0, -9.81)


def rotate(vec, theta):
    return [vec[0]*math.cos(radians(theta)) -
            vec[1]*math.sin(radians(theta)),
            vec[0]*math.sin(radians(theta)) +
            vec[1]*math.cos(radians(theta))]


def keyCallback(key, x, y):
    global paused, ESCAPE_KEY
    if key == ESCAPE_KEY:
        sys.exit()
    elif key == ' ':
        paused = not paused
    elif key == 'r':
        setupObjects()

def mouseMove(x, y):
    global mouseDown, mousePoints, mouseAccel
    if mouseDown:
        curTime = time.time()
        if len(mousePoints) and curTime - mousePoints[0][2] >= .01:
            mousePoints.appendleft([x, y, time.time()])
            while len(mousePoints) > 3:
                mousePoints.pop()
            print "mousepoints"
            for i in mousePoints:
                print i
            if len(mousePoints) == 3:
                dt1 = mousePoints[0][2] - mousePoints[1][2]
                dt2 = mousePoints[1][2] - mousePoints[2][2]
                dt3 = mousePoints[0][2] - mousePoints[2][2]
                
                dx1 = mousePoints[0][0] - mousePoints[1][0]
                dy1 = mousePoints[0][1] - mousePoints[1][1]
                
                dx2 = mousePoints[1][0] - mousePoints[2][0]
                dy2 = mousePoints[1][1] - mousePoints[2][1]
                print "dxs, dys:", dx1, dx2, dy1, dy2
                print "dts:", dt1, dt2, dt3
                v1x = dx1/dt1
                v1y = dy1/dt1
                
                v2x = dx2/dt2
                v2y = dy2/dt2
                print "velocities:", v1x, v1y, v2x, v2y
                ax = (v1x - v2x)/dt3
                ay = (v1y - v2y)/dt3
                        
                print("Computed accel:", ax, ay)
                scaledAccel = (ax*worldSize[0]/windowSize[0], 
                               ay*worldSize[1]/windowSize[1])
                print "Scaled accel:", scaledAccel
                mouseAccel = scaledAccel
def mouseClick(button, state, x, y):
    global mouseDown, mousePoints, mouseAccel
    if button == GLUT_LEFT_BUTTON:
        if state == GLUT_DOWN:
            print "clicked"
            mouseDown = True
            mousePoints = deque([[x,y, time.time()]])
            mouseAccel = (0.0, 0,0)

        else:
            print "released"
            mouseDown = False
            mouseAccel = (0.0, 0.0)
            mousePoints = deque([])


def draw():
    global arb, worldSize
    glClear(GL_COLOR_BUFFER_BIT)
    glColor(0.,1.,0.)
    glBegin(GL_LINES)
    glVertex(0.,0.)
    glVertex( float(windowSize[0]/2.0), 0.)
    glEnd()
    glColor(0,0,1)
    arb.draw(.1)
    glFlush()
    glutSwapBuffers()
    #time.sleep(.1)


dt = 1.0/simulationFramerate
def idleFunc():
    global frames, dt, paused, simulationTime, arb, mouseAccel
    if paused:
        time.sleep(.1)
        return
    frames = 0
    simulationTime = time.time()
    arb.updateAccel(0, mouseAccel)
    #print stepsPerFrame
    while frames < stepsPerFrame:
        arb.step(dt)
        frames += 1
    #wait until its time to update
    delta = time.time() - simulationTime
    while delta < 1.0/displayFramerate:
        time.sleep(delta)
        delta = time.time() - simulationTime
    glutPostRedisplay()
    
def setupObjects():
    global  arb, gravity
    x = RigidBody(mass1, initialPosition1, shape=size1)
    x.grounded = True
    y = RigidBody(mass2, initialPosition2, theta=initAngle, shape=size2)
    y.grounded = True
    arb = ArticulatedBody()
    arb.addBody(x)
    arb.addBody(y)

    arb.add2BodyConstraint((0, r2, 1, r3))
    arb.add1BodyConstraint((0, r1))
    g1 = tuple(x.mass*comp for comp in gravity)
    g2 = tuple(y.mass*comp for comp in gravity)
    arb.addExternalForce(0, (g1, (0,0)))
    arb.addExternalForce(1, (g2, (0,0)))

    arb.finalize()
    print arb.matrix
    print arb.externalForces

glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)
glutMouseFunc(mouseClick)
glutMotionFunc(mouseMove)

setupObjects()
glutMainLoop()
