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
mass1 = 3.0

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


#r = (-size[0]/2.0, -size[1]/2.0)#local vector, global computed each step

gravity = (0, -9.81)

stencilMat = np.matrix(np.zeros((10,10), dtype="float"))
stencilMat[0,6] = -1.0
stencilMat[0,8] = -1.0
stencilMat[1,7] = -1.0
stencilMat[1,9] = -1.0
stencilMat[3,8] = 1.0
stencilMat[4,9] = 1.0
stencilMat[6,0] = -1.0
stencilMat[8,0] = -1.0
stencilMat[7,1] = -1.0
stencilMat[9,1] = -1.0
stencilMat[8,3] = 1.0
stencilMat[9,4] = 1.0

                   
def rotate(vec, theta):
    return [vec[0]*math.cos(radians(theta)) -
            vec[1]*math.sin(radians(theta)),
            vec[0]*math.sin(radians(theta)) +
            vec[1]*math.cos(radians(theta))]

def computeForces():
    global bodies, r, gravity, stencilMat, actuationTorque
    bodies[0].clearForces()
    bodies[1].clearForces()

    #rotate r4 to global frame:
    r1g = rotate(r1, bodies[0].theta)
    r2g = rotate(r2, bodies[0].theta)
    r3g = rotate(r3, bodies[1].theta)


    Amat = np.copy(stencilMat)

    Amat[2,6] = r1g[1]
    Amat[2,7] = -r1g[0]
    Amat[2,8] = r2g[1]
    Amat[2,9] = -r2g[0]
    Amat[5,8] = -r3g[1]
    Amat[5,9] = r3g[0]
    Amat[6,2] = r1g[1]
    Amat[7,2] = -r1g[0]
    Amat[8,2] = r2g[1]
    Amat[8,5] = -r3g[1]
    Amat[9,2] = -r2g[0]
    Amat[9,5] = r3g[0]

    #print Amat
    b = np.zeros((10,1))

    omega0 = bodies[0].L/bodies[0].I
    omega1 = bodies[1].L/bodies[1].I

    b[1,0] = mass1*gravity[1]
    b[4, 0] = mass2*gravity[1]
    b[6, 0] = -omega0**2 * r1g[0]
    b[7,0] = -omega0**2 * r1g[1]
    b[8,0] = omega1**2 * r3g[0] - omega0**2 * r2g[0]
    b[9,0] = omega1**2 * r3g[1] - omega0**2 * r2g[1]

   
    #print b
    soln, garbage1, garbage2, garbage3  = np.linalg.lstsq(Amat, b)
    #print soln

    gforce1 = [comp*bodies[0].mass for comp in gravity] 
    bodies[0].addForce(gforce1, (0.0,0.0))
    bodies[0].addForce(soln[6:8], r1)
    bodies[0].addForce(soln[8:10], r2)
    gforce2 = [comp*bodies[1].mass for comp in gravity] 
    bodies[1].addForce(gforce2, (0.0,0.0))
    bodies[1].addForce((-soln[8, 0], -soln[9,0]), r3)


    #pointAccel = [soln[0,0] - soln[2,0]*r4g[1] - omega**2 *r4g[0],
    #              soln[1,0] + soln[2,0]*r4g[0] - omega**2 *r4g[1]
    #              ]
    #    print "Point accel:", pointAccel
    #   print "I *alpha:", bodies[1].I * soln[2,0]

def keyCallback(key, x, y):
    global paused, ESCAPE_KEY
    if key == ESCAPE_KEY:
        sys.exit()
    elif key == ' ':
        paused = not paused
    elif key == 'r':
        setupObjects()
def draw():
    global bodies, worldSize
    glClear(GL_COLOR_BUFFER_BIT)
    glColor(0.,1.,0.)
    glBegin(GL_LINES)
    glVertex(0.,0.)
    glVertex( float(windowSize[0]/2.0), 0.)
    glEnd()
    glColor(0,0,1)
    for body in bodies:
        body.draw(.1)
        #print body
    glFlush()
    glutSwapBuffers()
    #time.sleep(.1)

#dt = .0001
dt = 1.0/simulationFramerate
def idleFunc():
    global frames, dt, bodies, paused, simulationTime
    if paused:
        time.sleep(.1)
        return
    frames = 0
    simulationTime = time.time()
    #print stepsPerFrame
    while frames < stepsPerFrame:
        computeForces()
        for body in bodies:
        
            if not body.grounded and body.getBottom() <= 0.:
                print "Bottomed out"
                #glutPostRedisplay()
                return
            body.step(dt)
        frames += 1
    #wait until its time to update
    delta = time.time() - simulationTime
    while delta < 1.0/displayFramerate:
        time.sleep(delta)
        delta = time.time() - simulationTime
    glutPostRedisplay()
    
def setupObjects():
    global bodies, stencilMat
    x = RigidBody(mass1, initialPosition1, shape=size1)
    x.grounded = True
    y = RigidBody(mass2, initialPosition2, theta=initAngle, shape=size2)
    y.grounded = True
    bodies = [x, y]
    stencilMat[0,0] = x.mass
    stencilMat[1,1] = x.mass
    stencilMat[2,2] = x.I
    stencilMat[3,3] = y.mass
    stencilMat[4,4] = y.mass
    stencilMat[5,5] = y.I

    print stencilMat


glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)

setupObjects()
glutMainLoop()
