#!/usr/bin/python

from OpenGL.GL import *
import glHelp
import math

def radians(degrees):
    return degrees * math.pi/180.0

class RigidBody:


    def __init__(self,mass = 1, pos = (0,0), 
                 momentum = (0,0), theta = 0, 
                 angMom = 0, shape=(10, 10), 
                 grounded = False):
        self.mass = mass
        self.pos = list(pos)
        self.p = list(momentum)
        self.theta = theta
        self.L = angMom
        self.shape=shape

        self.forces = []
        self.torques = []
        self.grounded = grounded
        #compute moment of inertia from geometry
        self.I = self.mass * (self.shape[0]**2 + self.shape[1]**2)/12.0

    def addForce(self, f, pos):
        'add force (global coordinates) f at position p in local coordinates (0 is COM)'
        self.forces.append((f, pos))
        #t = r x f

    def getBottom(self):
        corners = ((self.shape[0]/2., self.shape[1]/2.),
                   (self.shape[0]/2., -self.shape[1]/2.),
                   (-self.shape[0]/2., - self.shape[1]/2.),
                   (-self.shape[0]/2., self.shape[1]/2.))
        return min(
            self.pos[1] + corner[0]*math.cos(radians(self.theta)) + corner[1]*math.sin(radians(self.theta)) 
            for corner in corners)

        
    def clearForces(self):
        self.forces = []
        self.torques = []

    def step(self, dt):
        posDot = [0,0]
        posDot[0] = self.p[0]/self.mass
        posDot[1] = self.p[1]/self.mass
        thetaDot = self.L/self.I
        pdot = [0.0,0.0]
        ldot = 0.0
        for force in self.forces:
            pdot[0] += force[0][0]
            pdot[1] += force[0][1]
        
            #convert local displacement to global vector
            #and compute r x f

            #rotate by theta
            newVec = [
                force[1][0]*math.cos(radians(self.theta)) - 
                force[1][1]*math.sin(radians(self.theta)),
                force[1][0]*math.sin(radians(self.theta)) + 
                force[1][1]*math.cos(radians(self.theta))
                ]
            print "newVec: ", newVec
            #now compute cross product with newVec
            ldot += newVec[0]*force[0][1] - newVec[1]*force[0][0]

        #just do euler integration
        print "Total force/m: (%s, %s)" %  (pdot[0]/self.mass,
                                          pdot[1]/self.mass)
        print "Old omega: ", self.L/self.I
        oldOmega = self.L/self.I
        self.pos[0] += posDot[0]*dt
        self.pos[1] += posDot[1]*dt
        
        self.theta += thetaDot*dt
        
        self.p[0] += pdot[0]*dt
        self.p[1] += pdot[1]*dt

        self.L += ldot*dt

        print "ldot*dt: ", ldot*dt
        rvec = [-math.cos(radians(self.theta)) + .5*math.sin(radians(self.theta)),
                -math.sin(radians(self.theta)) - .5*math.cos(radians(self.theta))]

        print "rvec: ", rvec
        print "Computed alpha: ", ldot/self.I
        omega = self.L/self.I
        print "New omega: ", omega
        a = [pdot[0]/self.mass, pdot[1]/self.mass]
        print "Computed a: ", a
        alpha = ldot/self.I
        print "Computed alpha: ", alpha

        print -alpha*rvec[1], -(oldOmega**2) * rvec[0]
        print -alpha*rvec[0], -(oldOmega**2) * rvec[1]
        pointAccel = [
            a[0] - alpha*rvec[1] - (oldOmega**2) * rvec[0],
            a[1] + alpha*rvec[0] - (oldOmega**2) * rvec[1]
            ]
        print a, alpha, omega, rvec
        print "Point accelerating: %s" % pointAccel

    def draw(self, scale = None):
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()

        glTranslate(self.pos[0], self.pos[1], 0)        
        glRotate(self.theta, 0, 0, 1)
        glTranslate(-self.pos[0], -self.pos[1], 0)

        glBegin(GL_QUADS)
        glVertex(self.pos[0] - self.shape[0]/2.0, 
                 self.pos[1] - self.shape[1]/2.0)
        glVertex(self.pos[0] - self.shape[0]/2.0, 
                 self.pos[1] + self.shape[1]/2.0)
        glVertex(self.pos[0] + self.shape[0]/2.0, 
                 self.pos[1] + self.shape[1]/2.0)
        glVertex(self.pos[0] + self.shape[0]/2.0, 
                 self.pos[1] - self.shape[1]/2.0)

        glEnd()



        glPopMatrix()
        for force in self.forces:
            fpos = (self.pos[0] + force[1][0]*math.cos(radians(self.theta)) - force[1][1]*math.sin(radians(self.theta)),
                    self.pos[1] + force[1][0]*math.sin(radians(self.theta)) + force[1][1]*math.cos(radians(self.theta)))
            glHelp.drawArrow(force[0], fpos, scale)
        glBegin(GL_POINTS)
        glVertex(self.pos[0], self.pos[1])
        glEnd()



    def __str__(self):
        return "mass: %s\nI: %s\nPosition: %s\nVelocity: %s\nTheta: %s\nAngMom: %s\nShape: %s\n Forces: %s\nTorques: %s" % (self.mass, self.I, self.pos, self.p, self.theta, self.L, self.shape, self.forces, self.torques)

    
