#!/usr/bin/python

from OpenGL.GL import *


class RigidBody:


    def __init__(self,mass = 1, momInertia = 1, pos = (0,0), momentum = (0,0), theta = 0, angMom = 0, shape=(10, 10)):
        self.mass = mass
        self.I = momInertia
        self.pos = list(pos)
        self.p = list(momentum)
        self.theta = theta
        self.L = angMom
        self.shape=shape

        self.forces = []
        self.torques = []

    def addForce(self, f, pos):
        'add force f at position p in local coordinates (0 is COM)'
        self.forces.append(f)
        #t = r x f
        self.torques.append(pos[0]*f[1] - pos[1]*f[0])
        
    def clearForces(self):
        self.forces = []
        self.torques = []

    def step(self, dt):
        posDot = [0,0]
        posDot[0] = self.p[0]/self.mass
        posDot[1] = self.p[1]/self.mass
        thetaDot = self.L/self.I
        pdot = [0.0,0.0]
        for force in self.forces:
            pdot[0] += force[0]
            pdot[1] += force[1]
        ldot = 0.0
        for torque in self.torques:
            ldot += torque

        #just do euler integration
        self.pos[0] += posDot[0]*dt
        self.pos[1] += posDot[1]*dt
        
        self.theta += thetaDot*dt
        
        self.p[0] += pdot[0]*dt
        self.p[1] += pdot[1]*dt

        self.L += ldot*dt

    def draw(self):
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
        
        glBegin(GL_POINTS)
        glVertex(self.pos[0], self.pos[1])
        glEnd()

        glPopMatrix()



    def __str__(self):
        return "mass: %s\nI: %s\nPosition: %s\nVelocity: %s\nTheta: %s\nAngMom: %s\nShape: %s\n Forces: %s\nTorques: %s" % (self.mass, self.I, self.pos, self.p, self.theta, self.L, self.shape, self.forces, self.torques)

    
