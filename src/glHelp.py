#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

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
    glColor(0,0,1,1)
    glPointSize(5)

    #set up matrix so (0,0), (size[0], size[1]) scales to (-1, -1), (1, 1)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslate(-1, -1, 0)
    glScalef(2.0/worldSize[0], 2.0/worldSize[1], 1)
