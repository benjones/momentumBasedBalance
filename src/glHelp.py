#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import math

def degrees(radians):
    return radians * 180.0/math.pi

def setupGL(windowSize, worldSize, drawFunc, glutArgs = []):
    glutInit(glutArgs)
    glutInitWindowSize(*windowSize)
    #glutInitWindowPosition(10,10)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA)
    windID = glutCreateWindow("2D Simulator")
    glutDisplayFunc(drawFunc)

    glClearColor(1.0, 1.0, 1.0, 1.0)
    glDisable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    glColor(0.0,0.0,1.0,1.0)
    glPointSize(5.0)
    glLineWidth(2.0)
    glEnable(GL_LINE_SMOOTH)
    glEnable(GL_POINT_SMOOTH)
    #set up matrix so (0,0), (size[0], size[1]) scales to (-1, -1), (1, 1)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslate(-1, -.95, 0)
    glScalef(2.0/worldSize[0], 1.95/worldSize[1], 1.0)

def drawArrow(magnitude, position, scale):
    #scale, rotate, translate
    
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glTranslate(position[0], position[1], 0.0)
    
    mag = math.sqrt(magnitude[0]**2 + magnitude[1]**2)
    if scale is not None:
        mag *= scale
    glRotate(degrees(math.atan2(magnitude[1], magnitude[0])) + 90., 0, 0, 1)
    glScale(mag, mag, 1)
    glBegin(GL_LINES)
    glVertex(0.,1.)
    glVertex(0.,0.01)

    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    glBegin(GL_TRIANGLES)
    glVertex(0.,0.)
    glVertex(-.1, .1)
    glVertex(.1,.1)
    glEnd()
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)

    glPopMatrix()
    print("Drew arrow")
