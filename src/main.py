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
    '''glBegin(GL_TRIANGLES)
    
    glVertex(20, 20)
    glVertex(20, 80)
    glVertex(90, 10)

    glEnd()
    glColor(0,0,1)
    x = RigidBody(1, 1, (70, 50), (0,0), 30, 0, (30, 10))
    x.draw()
    y = RigidBody(1, 1, (00, 10))
    glColor(1, 0, 0)
    y.draw()
    print str(x)'''
    for body in bodies:
        body.draw()
        print body
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


    if frames % 10 == 0:
        glutPostRedisplay()
glHelp.setupGL((800, 800), (100, 100), draw)

glutIdleFunc(idleFunc)

x = RigidBody(1, 1, (50, 70))
x.addForce([0, -10],[9,10]) 
y = RigidBody(1, 1, (20, 80))
y.addForce([3, -10],[0,0])
bodies = [x, y]

glutMainLoop()
