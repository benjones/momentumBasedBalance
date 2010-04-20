#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import time
import sys
import glHelp
import math
import numpy as np

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
    global frames, dt, paused, simulationTime, arb
    if paused:
        time.sleep(.1)
        return
    frames = 0
    simulationTime = time.time()
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

setupObjects()
glutMainLoop()
