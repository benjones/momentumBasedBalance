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


def keyCallback(key, x, y):
    global paused, ESCAPE_KEY
    if key == ESCAPE_KEY:
        sys.exit()
    elif key == ' ':
        paused = not paused
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
    time.sleep(.1)
frames = 0
dt = .0001
def idleFunc():
    global frames, dt, bodies, paused
    if paused:
        time.sleep(.1)
        return
    frames += 1

    for body in bodies:
        
        if not body.grounded and body.getBottom() <= 0.:
            glutPostRedisplay()
            return
        body.step(dt)


    if frames % 100 == 0:
        glutPostRedisplay()
glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)
x = RigidBody(1, 1, (50, 70))
x.addForce([0, -20],[4,5])
x.grounded = True
y = RigidBody(1, 1, (20, 80))
y.addForce([3, -10],[0,0])
bodies = [x, y]

glutMainLoop()
