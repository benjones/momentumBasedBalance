#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import time

import glHelp

from rigidBody import RigidBody

def draw():
    global bodies
    glClear(GL_COLOR_BUFFER_BIT)
    glColor(0,0,1)
    for body in bodies:
        body.draw()
        print body
 #   glColor(1,0,0)
#    glHelp.drawArrow(0,0,0)
    glFlush()
    glutSwapBuffers()
    time.sleep(.1)
frames = 0
dt = .0001
def idleFunc():
    global frames, dt, bodies
    frames += 1

    for body in bodies:
        if body.pos[1] < 10:
            glutPostRedisplay()
            return
        body.step(dt)


    if frames % 100 == 0:
        glutPostRedisplay()
glHelp.setupGL((800, 800), (100, 100), draw)

glutIdleFunc(idleFunc)

x = RigidBody(1, 1, (50, 70))
x.addForce([0, -20],[4,5]) 
y = RigidBody(1, 1, (20, 80))
y.addForce([3, -10],[0,0])
bodies = [x, y]

glutMainLoop()
