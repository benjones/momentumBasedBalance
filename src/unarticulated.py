#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import time
import sys
import glHelp

from rigidBody import RigidBody

paused = False
ESCAPE_KEY = '\033'

windowSize = (800, 800)
worldSize = (100, 100)

displayFramerate = 60 #fps
simulationFramerate = 600 #fps

stepsPerFrame = float(simulationFramerate)/displayFramerate

simulationTime = time.time()

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
    glVertex( float(windowSize[0]), 0.)
    glEnd()
    glColor(0,0,1)
    for body in bodies:
        body.draw()
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
    global bodies
    x = RigidBody(1,  (50, 70))
    x.addForce([0, -20],[5,5])
    x.addForce([0, 10],[-5, -5])
    x.grounded = True
    y = RigidBody(1,  (20, 80))
    y.addForce([3, -10],[0,0])
    bodies = [x, y]
    

glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)

setupObjects()
glutMainLoop()
