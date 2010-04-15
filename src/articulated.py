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
worldSize = (10.0, 10.0)

displayFramerate = 60.0 #fps
simulationFramerate = 600.0 #fps

stepsPerFrame = float(simulationFramerate)/displayFramerate

simulationTime = time.time()

#two blocks, one fixed to the ground, with one point-point
#constraint

size1 = (2.0, 1.0)
mass1 = 3.0

size2 = (2.0, 1.0)
mass2 = 1.0


r1 = [-size1[0]/2.0, -size1[1]/2.0]
r2 = [size1[0]/2.0, -size1[1]/2.0]
r3 = [size1[0]/2.0, size1[1]/2.0]
r4 = [-size2[0]/2.0, 0]

initialPosition1 = (4.0, size1[1]/2.0)
initAngle = 45.0
initialPosition2 = (initialPosition1[0] + size1[0]/2.0 +
                    .5*size2[0]*math.cos(radians(initAngle)),
                    initialPosition1[1] + size1[1]/2.0 +
                    .5*size2[0]*math.sin(radians(initAngle))
                    )


#r = (-size[0]/2.0, -size[1]/2.0)#local vector, global computed each step

gravity = (0, -9.81)

stencilMat = np.matrix([
        [0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
])

def rotate(vec, theta):
    return [vec[0]*math.cos(radians(theta)) -
            vec[1]*math.sin(radians(theta)),
            vec[0]*math.sin(radians(theta)) +
            vec[1]*math.cos(radians(theta))]

def computeForces():
    global bodies, r, gravity, stencilMat
    bodies[0].clearForces()
    bodies[1].clearForces()

    #rotate r4 to global frame:
    r1g = rotate(r1, bodies[0].theta)
    r2g = rotate(r2, bodies[0].theta)
    r3g = rotate(r3, bodies[0].theta)
    r4g = rotate(r4, bodies[1].theta)

    Amat = np.copy(stencilMat)
    Amat[2, 3] = -r1g[1]
    Amat[2, 4] = r1g[0]
    Amat[2, 5] = -r2g[1]
    Amat[2,6] = r2g[0]
    Amat[2,7] = -r3g[1]
    Amat[2,8] = r3g[0]
    
    Amat[5, 7] = -r4g[1]
    Amat[5, 8] = r4g[0]
    Amat[6, 2] = -r4g[1]
    Amat[7, 2] = r4g[0]

    print Amat
    b = np.zeros((8,1))

    omega = bodies[1].L/bodies[1].I
    

    b[1,0] = -mass1*gravity[1]
    b[4, 0] = mass2*gravity[1]
    b[6, 0] = omega**2 * r4g[0]
    b[7,0] = omega**2 * r4g[1]
    
    soln, garbage1, garbage2, garbage3  = np.linalg.lstsq(Amat, b)
    print soln

    gforce1 = [comp*bodies[0].mass for comp in gravity] 
    bodies[0].addForce(gforce1, (0.0,0.0))
    bodies[0].addForce(soln[3:5], r1)
    bodies[0].addForce(soln[5:7], r2)
    bodies[0].addForce(soln[7:9], r3) 
    gforce2 = [comp*bodies[1].mass for comp in gravity] 
    bodies[1].addForce(gforce2, (0.0,0.0))
    bodies[1].addForce((-soln[7, 0], -soln[8,0]), r4)
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
    print "Draw callback"
    glClear(GL_COLOR_BUFFER_BIT)
    glColor(0.,1.,0.)
    glBegin(GL_LINES)
    glVertex(0.,0.)
    glVertex( float(windowSize[0]/2.0), 0.)
    glEnd()
    glColor(0,0,1)
    for body in bodies:
        body.draw(.1)
        print body
        '''y = body.getBottom()
        glBegin(GL_LINES)
        glVertex(0., y)
        glVertex(windowSize[0], y)
        glEnd()'''
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
    print "about to redisplay"
    glutPostRedisplay()
    
def setupObjects():
    global bodies, stencilMat
    x = RigidBody(mass1, initialPosition1, shape=size1)
    x.grounded = True
    y = RigidBody(mass2, initialPosition2, theta=initAngle, shape=size2)
    y.grounded = True
    bodies = [x, y]
    stencilMat[3,0] = y.mass
    stencilMat[4,1] = y.mass
    stencilMat[5,2] = y.I

    print stencilMat


glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)

setupObjects()
glutMainLoop()
