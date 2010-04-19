#!/usr/bin/python

from rigidBody import RigidBody, degrees, radians, rotate
import numpy as np


class ArticulatedBody:

    def __init__(self):
        self.bodies = []
        self.constraints = []
        self.pins = []
        self.externalForces = []
    def addBody(self, body):
        self.bodies.append(body)

    def add2BodyConstraint(self, constraint):
        '''constraints are of the form
        (body index 1, vector to application on body 1,
        body index 2, vector to application on body 2)
        '''
        self.constraints.append(constraint)

    def add1BodyConstraint(self, constraint):
        'pin of the form (index, rvector)'
        self.pins.append(constraint)

    def addExternalForce(self, body, force):
        'force is of the form ((fx, fy), (rx, ry))'
        self.externalForces.append((body, force))
        


    def finalize(self):
        'Once all the bodies/constraints are listed, create matrix'
        self.matsize = 3*len(self.bodies) + 2*len(self.constraints) + 2*len(self.pins)
        self.matrix = np.matrix(np.zeros((self.matsize, self.matsize)))

        
        
        for body in range(len(self.bodies)):
        
            self.bodies[body].clearForces()
        
            self.matrix[3*body, 3*body] = self.bodies[body].mass
            self.matrix[3*body + 1, 3*body + 1] = self.bodies[body].mass
            self.matrix[3*body + 2, 3*body + 2] = self.bodies[body].I

        constraintOffset = 3*len(self.bodies)
        #add constraints
        for constraint in range(len(self.constraints)):
            #fill in the 3x2 block for object 1
            #can't do the rvector part yet though
            self.matrix[3*self.constraints[constraint][0], 
                        constraintOffset + constraint*2] = -1
            self.matrix[3*self.constraints[constraint][0] + 1, 
                        constraintOffset + constraint*2 + 1] = -1

            self.matrix[constraintOffset + constraint*2,
                        3*self.constraints[constraint][0]] = -1
            self.matrix[constraintOffset + constraint*2 +1,
                        3*self.constraints[constraint][0] + 1] = -1


            #object 2:
            self.matrix[3*self.constraints[constraint][2], 
                        constraintOffset + constraint*2] = 1
            self.matrix[3*self.constraints[constraint][2] + 1, 
                        constraintOffset + constraint*2 + 1] = 1

            self.matrix[constraintOffset + constraint*2,
                        3*self.constraints[constraint][2]] = 1
            self.matrix[constraintOffset + constraint*2 +1,
                        3*self.constraints[constraint][2] + 1] = 1

        pinoffset = 3*len(self.bodies) + 2*len(self.constraints)
        for pin in range(len(self.pins)):
            self.matrix[3*self.pins[pin][0],
                        pinoffset + 2*pin] = -1
            self.matrix[3*self.pins[pin][0] + 1,
                        pinoffset + 2*pin +1] = -1
            self.matrix[pinoffset +2*pin,
                        3*self.pins[pin][0]] = -1
            self.matrix[pinoffset + 2*pin +1,
                        3*self.pins[pin][0] +1] = -1

    def step(self, dt):
        for body in self.bodies:
            body.clearForces()

        #fill in the rest of the stuff, including the soln vector
        solVector = np.matrix(np.zeros((self.matsize,1)))
        constraintOffset = 3*len(self.bodies)
        for constraint in range(len(self.constraints)):
            body1 = self.bodies[self.constraints[constraint][0]] 
            body2 = self.bodies[self.constraints[constraint][2]] 
            r1g = rotate(self.constraints[constraint][1],
                         body1.theta)
            r2g = rotate(self.constraints[constraint][3],
                         body2.theta)
            self.matrix[3*self.constraints[constraint][0] + 2,
                        constraintOffset + constraint*2] = r1g[1]
            self.matrix[3*self.constraints[constraint][0] + 2,
                        constraintOffset + constraint*2 + 1] = -r1g[0]

            self.matrix[constraintOffset + constraint*2,
                        3*self.constraints[constraint][0] +2] = r1g[1]
            self.matrix[constraintOffset + constraint*2 + 1,
                        3*self.constraints[constraint][0] +2] = -r1g[0]


            self.matrix[3*self.constraints[constraint][2] + 2,
                        constraintOffset + constraint*2] = -r2g[1]
            self.matrix[3*self.constraints[constraint][2] + 2,
                        constraintOffset + constraint*2 + 1] = r2g[0]
            
            self.matrix[constraintOffset + constraint*2,
                        3*self.constraints[constraint][2] +2] = -r2g[1]
            self.matrix[constraintOffset +constraint*2 + 1,
                        3*self.constraints[constraint][2] +2] = r2g[0]

            #fill in b vector:
            omega1 = body1.L/body1.I
            omega2 = body2.L/body2.I
            solVector[constraintOffset + constraint*2, 
                      0] = omega2**2*r2g[0] - omega1**2*r1g[0]
            solVector[constraintOffset + constraint*2 + 1,
                      0] = omega2**2*r2g[1] - omega1**2*r1g[1]


        pinoffset = constraintOffset + 2*len(self.constraints)
        for pin in range(len(self.pins)):
            body = self.bodies[self.pins[pin][0]]
            rvec = rotate(self.pins[pin][1], body.theta)
            self.matrix[3*self.pins[pin][0] + 2,
                        pinoffset + pin*2] = rvec[1]
            self.matrix[3*self.pins[pin][0] + 2,
                        pinoffset + pin*2 + 1] = -rvec[0]
            self.matrix[pinoffset + pin*2,
                        3*self.pins[pin][0] +2] = rvec[1]
            self.matrix[pinoffset + pin*2 +1,
                        3*self.pins[pin][0] +2] = -rvec[0]
        
            #add entries to soln matrix
            omega = body.L/body.I
            solVector[pinoffset + pin*2, 0] = -omega**2*rvec[0]
            solVector[pinoffset + pin*2 +1, 0] - -omega**2*rvec[1]

        for force in self.externalForces:
            solVector[force[0]*3] += force[1][0][0]
            solVector[force[0]*3 +1] += force[1][0][1]
            #torque:
            rvec = rotate(force[1][1], self.bodies[force[0]].theta)
            solVector[force[0]*3 + 2] += rvec[0]*force[1][1][1] - rvec[1]*force[1][1][0]

            #apply them to the bodies
            self.bodies[force[0]].addForce(*force[1])
            

        #print "Step:", self.matrix, solVector
        soln = np.linalg.solve(self.matrix, solVector)
        for constraint in range(len(self.constraints)):
            start = constraintOffset + constraint*2
            f1 = tuple(soln[start: start+2, 0])
            f2 = (-f1[0], -f1[1])
            self.bodies[self.constraints[constraint][0]].addForce(
                f1, self.constraints[constraint][1])
            self.bodies[self.constraints[constraint][2]].addForce(
                f2, self.constraints[constraint][3])
        for pin in range(len(self.pins)):
            start = pinoffset + 2*pin
            f = tuple(soln[start: start+2, 0])
            self.bodies[self.pins[pin][0]].addForce(
                f, self.pins[pin][1])
        for body in self.bodies:
            body.step(dt)
    def draw(self, scale = 1.0):
        for body in self.bodies:
            body.draw(scale)
            #print body
if __name__ == '__main__':
    x = RigidBody(12, shape=(6, 2))
    y = RigidBody(24, shape=(10, 4))

    arb = ArticulatedBody()
    arb.addBody(x)
    arb.addBody(y)

    arb.add2BodyConstraint((0, (3, 1), 1, (-5, -2)))
    arb.add1BodyConstraint((0, (-3, -1)))
    
    arb.finalize()
    arb.addExternalForce(0, ((0, -9.81*x.mass), (0,0)))
    arb.addExternalForce(1, ((0, -9.81*y.mass), (0,0)))

    print arb.matrix
    arb.step(100)

