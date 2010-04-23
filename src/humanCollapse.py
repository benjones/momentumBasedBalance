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
from articulatedBody import ArticulatedBody

paused = False
ESCAPE_KEY = '\033'

windowSize = (800, 800)
worldSize = (3.0, 3.0)

displayFramerate = 60.0 #fps
simulationFramerate = 300.0 #fps

stepsPerFrame = float(simulationFramerate)/displayFramerate

simulationTime = time.time()

#in meters and kilograms
llsize = (.1, .5)
llmass = 2.5

ulsize = (.2, .5)
ulmass = 3.5 

torsosize = (.4, .6)
torsomass = 25.0

uasize = (.3, .05)
uamass = 2.0

uar1 = (uasize[0]/2.0, 0.0)
uar2 = (-uasize[0]/2.0, 0.0)

lasize = (.3, .04)
lamass = 1.5

lar1 = (lasize[0]/2.0, 0.0)

llr1 = (0.0, -llsize[1]/2.0)
llr2 = (0.0, llsize[1]/2.0)

ulr1 = (0.0, -ulsize[1]/2.0)
ulr2 = (0.0, ulsize[1]/2.0)

torsor1 = (-torsosize[0]/2.0, -torsosize[1]/2.0)
torsor2 = (torsosize[0]/2.0, -torsosize[1]/2.0)
torsor3 = (-torsosize[0]/2.0, torsosize[1]/2.0)
torsor4 = (torsosize[0]/2.0, torsosize[1]/2.0)
torsor5 = (0.0, torsosize[1]/2.0)

headmass = 8.0
headsize = (.15, .25)
headr = (0.0, -headsize[1]/2.0)

initUlAngle = -10.0
initUaAngle = 15.0
initLaAngle = 60.0


ll1pos = (1.5, llsize[1]/2.0)
ul1pos = (ll1pos[0] - ulsize[1]*0.5*math.sin(radians(initUlAngle)),
          ll1pos[1] + llsize[1]/2.0 + ulsize[1]*0.5*math.cos(radians(initUlAngle)))

torsopos = (ul1pos[0] - .5*math.sin(radians(initUlAngle))*ulsize[1] + 
            .5*torsosize[0],
            ul1pos[1] + .5*math.cos(radians(initUlAngle))*ulsize[1] + 
            .5*torsosize[1])

ul2pos = (torsopos[0] + torsosize[0]*.5 - ulsize[1]*.5*
          math.sin(radians(initUlAngle)),
          torsopos[1] - torsosize[1]*.5 - ulsize[1]*.5*
          math.cos(radians(initUlAngle)))

ll2pos = (ul2pos[0] - ulsize[1]*.5*math.sin(radians(initUlAngle)),
          ul2pos[1] - ulsize[1]*.5*math.cos(radians(initUlAngle)) - llsize[1]/2.0)

ua1pos = (torsopos[0] - torsosize[0]/2.0 - uasize[0]*.5*math.cos(radians(initUaAngle)),
          torsopos[1] + torsosize[1]/2.0 - uasize[0]*.5*math.sin(radians(initUaAngle)))

la1pos = (ua1pos[0] - uasize[0]*math.cos(radians(initUaAngle))*.5 - 
          math.cos(radians(initLaAngle))*lasize[0]*.5,
          ua1pos[1] - uasize[0]*math.sin(radians(initUaAngle))*.5 -
          math.sin(radians(initLaAngle))*lasize[0]*.5)

ua2pos = (torsopos[0] + torsosize[0]/2.0 + uasize[0]*.5*math.cos(radians(initUaAngle)),
          torsopos[1] + torsosize[1]/2.0 - uasize[0]*.5*math.sin(radians(initUaAngle)))

la2pos = (ua2pos[0] + uasize[0]*math.cos(radians(initUaAngle))*.5 + 
          math.cos(radians(initLaAngle))*lasize[0]*.5,
          ua2pos[1] - uasize[0]*math.sin(radians(initUaAngle))*.5 -
          math.sin(radians(initLaAngle))*lasize[0]*.5)

headpos = (torsopos[0],
           torsopos[1] + torsosize[1]/2.0 + headsize[1]/2.0)

gravity = (0, -9.81/2.0)

def rotate(vec, theta):
    return [vec[0]*math.cos(radians(theta)) -
            vec[1]*math.sin(radians(theta)),
            vec[0]*math.sin(radians(theta)) +
            vec[1]*math.cos(radians(theta))]


def keyCallback(key, x, y):
    global paused, ESCAPE_KEY
    if key == ESCAPE_KEY:
        sys.exit()
    elif key == ' ':
        paused = not paused
    elif key == 'r':
        setupObjects()



def draw():
    global arb, worldSize
    glClear(GL_COLOR_BUFFER_BIT)
    glColor(0.,1.,0.)
    glBegin(GL_LINES)
    glVertex(0.,0.)
    glVertex( float(windowSize[0]/2.0), 0.)
    glEnd()
    glColor(0,0,1)
    arb.draw(.001)
    glFlush()
    glutSwapBuffers()
    #time.sleep(.1)


dt = 1.0/simulationFramerate
def idleFunc():
    global frames, dt, paused, simulationTime, arb, mouseAccel
    if paused:
        time.sleep(.1)
        return
    frames = 0
    simulationTime = time.time()
    motionTime = simulationTime
    #print stepsPerFrame
    while frames < stepsPerFrame:
        arb.step(dt)
        frames += 1
        motionTime += dt
    #wait until its time to update
    delta = time.time() - simulationTime
    while delta < 1.0/displayFramerate:
        time.sleep(delta)
        delta = time.time() - simulationTime
    glutPostRedisplay()
    
def setupObjects():
    global  arb, gravity
    arb = ArticulatedBody()

    ll1 = RigidBody(mass = llmass, pos = ll1pos, shape=llsize)
    ul1 = RigidBody(mass = ulmass, pos = ul1pos, shape=ulsize, theta=initUlAngle)
    torso = RigidBody(mass = torsomass, pos = torsopos, shape = torsosize)
    ul2 = RigidBody(mass = ulmass, pos = ul2pos, shape=ulsize, theta=-initUlAngle)
    ll2 = RigidBody(mass = llmass, pos = ll2pos, shape=llsize)

    ua1 = RigidBody(mass = uamass, pos = ua1pos, shape=uasize, theta=initUaAngle)
    la1 = RigidBody(mass = lamass, pos = la1pos, shape=lasize, theta=initLaAngle)
    ua2 = RigidBody(mass = uamass, pos = ua2pos, shape=uasize, theta=-initUaAngle)
    la2 = RigidBody(mass = lamass, pos = la2pos, shape=lasize, theta=-initLaAngle)
    head = RigidBody(mass = headmass, pos = headpos, shape=headsize)

    arb.addBody(ll1)
    arb.addBody(ul1)
    arb.addBody(torso)
    arb.addBody(ul2)
    arb.addBody(ll2)
    arb.addBody(ua1)
    arb.addBody(la1)
    arb.addBody(ua2)
    arb.addBody(la2)
    arb.addBody(head)
    
    gll = tuple(llmass*comp for comp in gravity)
    gul = tuple(ulmass*comp for comp in gravity)
    gtorso = tuple(torsomass*comp for comp in gravity)
    gua = tuple(uamass*comp for comp in gravity)
    gla = tuple(lamass*comp for comp in gravity)
    ghead = tuple(headmass*comp for comp in gravity)


    arb.addExternalForce(0, (gll, (0.0,0.0)))
    arb.addExternalForce(4, (gll, (0.0,0.0)))
    arb.addExternalForce(1, (gul, (0.0,0.0)))
    arb.addExternalForce(3, (gul, (0.0,0.0)))
    arb.addExternalForce(2, (gtorso, (0.0,0.0)))
    arb.addExternalForce(5, (gua, (0.0,0.0)))
    arb.addExternalForce(7, (gua, (0.0,0.0)))
    arb.addExternalForce(6, (gla, (0.0,0.0)))
    arb.addExternalForce(8, (gla, (0.0,0.0)))
    arb.addExternalForce(9, (ghead, (0.0,0.0)))

    arb.add1BodyConstraint((0, llr1))
    arb.add1BodyConstraint((4, llr1))

    arb.add2BodyConstraint((0, llr2, 1, ulr1))
    arb.add2BodyConstraint((4, llr2, 3, ulr1))

    arb.add2BodyConstraint((1, ulr2, 2, torsor1))
    arb.add2BodyConstraint((3, ulr2, 2, torsor2))

    arb.add2BodyConstraint((5, uar1, 2, torsor3))
    arb.add2BodyConstraint((5, uar2, 6, lar1))

    arb.add2BodyConstraint((7, (-uar1[0], uar1[1]), 2, torsor4))
    arb.add2BodyConstraint((7, (-uar2[0], uar2[1]), 8, (-lar1[0], lar1[1])))

    arb.add2BodyConstraint((2, torsor5, 9, headr))

    arb.finalize()
    print arb.matrix
    print arb.externalForces

glHelp.setupGL(windowSize, worldSize, draw)

glutIdleFunc(idleFunc)
glutKeyboardFunc(keyCallback)

setupObjects()
glutMainLoop()
