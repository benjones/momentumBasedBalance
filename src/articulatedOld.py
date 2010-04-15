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
size2 = (2.0, 1.0)
mass1 = 3.0
mass2 = 1.0
initAngle = 45.0
r1 = (-size1[0]/2.0, -size1[1]/2.0)
r2 = (size1[0]/2.0, -size1[1]/2.0)
r3 = (size1[0]/2.0, size1[1]/2.0)
r4 = (-size2[0]/2.0, 0)#local vector, global computed each step

gravity = (0, -9.81)

stencilMat = np.matrix([
        [0,0,0,1,0,1,0,1,0],
        [0,0,0,0,1,0,1,0,1],
        [0,0,0,0,0,0,0,0,0],
        [1,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0],
        [mass2,0,0,0,0,0,0,1,0],
        [0,mass2,0,0,0,0,0,0,1],
        [0,0,0,0,0,0,0,0,0]])

def computeForces():
    global bodies, r1, r2, r3, r4, gravity, stencilMat
    bodies[0].clearForces()
    bodies[1].clearForces()
    

    #rotate r4 to global frame:
    r4g = [r4[0]*math.cos(radians(bodies[1].theta)) -
           r4[1]*math.sin(radians(bodies[1].theta)),
           r4[0]*math.sin(radians(bodies[1].theta)) +
           r4[1]*math.cos(radians(bodies[1].theta))]
    
    Amat = np.copy(stencilMat)
    
    Amat[2, 3:] =  np.array([[-r1[1], r1[0], -r2[1], r2[0], -r3[1], r3[0]]])
    
    Amat[3, 2] = -r4g[1]
    Amat[4,2] = r4g[0]
    
    Amat[7, 7:] = np.array([[r4g[1], -r4g[0]]])
    print Amat
    b = np.zeros((8,1))
    b[1,0] = -mass1*gravity[1]
    b[6,0] = mass2*gravity[1]
    
    print b

    soln, res, rank, sv = np.linalg.lstsq(Amat, b)

    bodies[0].addForce(gravity, (0.0,0.0))
    bodies[1].addForce(gravity, (0.0,0.0))
    print soln, rank
    #print soln[3:5]
    print Amat*np.matrix(soln)
    bodies[0].addForce(tuple(soln[3:5,0]), r1)
    bodies[0].addForce(tuple(soln[5:7,0]), r2)
    bodies[0].addForce(tuple(soln[7:]), r3)
    
    bodies[1].addForce((-soln[7,0], -soln[8,0]), r4)

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
    x = RigidBody(3.0, (3.0,size1[1]/2.0),  shape=size1)
    x.grounded = True
    ypos = [ x.pos[0] + x.shape[0]/2.0 + size2[0]*math.cos(radians(initAngle))/2.0,
             x.pos[1] + x.shape[1]/2.0 + size2[0]*math.sin(radians(initAngle))/2.0]
    y = RigidBody(1.0,  ypos, theta=initAngle, shape=size2)
    bodies = [x, y]
    stencilMat[7, 2] = -y.I
    print stencilMat


glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)

setupObjects()
glutMainLoop()
