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
simulationFramerate = 1200.0 #fps

stepsPerFrame = float(simulationFramerate)/displayFramerate

simulationTime = time.time()

#two blocks, one fixed to the ground, with one point-point
#constraint

size = (2.0, 1.0)
mass = 3.0
initAngle = 45.0
initPosition = (5.0, 7.0)
r = (-size[0]/2.0, -size[1]/2.0)#local vector, global computed each step

gravity = (0, -9.81)

stencilMat = np.matrix([
        [0.0, 0.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, -1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0, 0.0]
])

def computeForces():
    global bodies, r, gravity, stencilMat
    bodies[0].clearForces()

    #rotate r4 to global frame:
    rg = [r[0]*math.cos(radians(bodies[0].theta)) -
           r[1]*math.sin(radians(bodies[0].theta)),
           r[0]*math.sin(radians(bodies[0].theta)) +
           r[1]*math.cos(radians(bodies[0].theta))]
    
    #print rg

    Amat = np.copy(stencilMat)
    
    Amat[2,3] = rg[1]
    Amat[2,4] = -rg[0]
    Amat[3,2] = rg[1]
    Amat[4,2] = -rg[0]
    
    #print Amat
    b = np.zeros((5,1))

    omega = bodies[0].L/bodies[0].I
    

    b[1,0] = mass*gravity[1]
    b[3,0] = -(omega**2 * rg[0])
    b[4,0] = -(omega**2 * rg[1])


    
    #print b

    soln  = np.linalg.solve(Amat, b)
    #print soln
    #print np.dot(Amat,soln)


    gforce = [comp*bodies[0].mass for comp in gravity] 
    bodies[0].addForce(gforce, (0.0,0.0))
    bodies[0].addForce(tuple(soln[3:]), r)
    #print soln[0,0], soln[1,0], soln[2,0], omega, rg
    #print -soln[2,0]*rg[1], -(omega**2) *rg[0]
    #print -soln[2,0]*rg[0], -(omega**2)* rg[1]
    #pointAccel = [
    #    soln[0,0] - soln[2,0]*rg[1] - (omega**2) * rg[0],
    #    soln[1,0] + soln[2,0]*rg[0] - (omega**2) * rg[1]
    #    ]
    #print("Expected point acceleration: %s" % pointAccel)

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
    x = RigidBody(mass, initPosition, shape=size, theta=initAngle)
    bodies = [x]
    stencilMat[0, 0] = x.mass
    stencilMat[1,1] = x.mass
    stencilMat[2,2] = x.I
    print stencilMat


glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)

setupObjects()
glutMainLoop()
