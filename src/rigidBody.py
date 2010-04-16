#!/usr/bin/python

from OpenGL.GL import *
import glHelp
import math

def radians(degrees):
    return degrees * math.pi/180.0

def degrees(radians):
    return radians * 180.0/math.pi

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

    def addTorque(self, tau):
        self.torques.append(tau)

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
        posDot =  [self.p[0]/self.mass,
                self.p[1]/self.mass]
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
            #now compute cross product with newVec
            ldot += newVec[0]*force[0][1] - newVec[1]*force[0][0]
        #sum pure torques
        for torque in self.torques:
            ldot += torque
        #just do euler integration

        self.pos[0] += posDot[0]*dt
        self.pos[1] += posDot[1]*dt
        
        self.theta += degrees(thetaDot*dt)
        
        self.p[0] += pdot[0]*dt
        self.p[1] += pdot[1]*dt

        self.L += ldot*dt


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

    
