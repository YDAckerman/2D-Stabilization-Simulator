import numpy as np
from CtrlSys import CtrlSys as cs

class Pod(object):

    """ A pod looks like: 
                          r_3 -- r_0
                          |  \  /  |
                          |   CM   |
                          |  /  \  |
                          r_2 __ r_1
    With mass equally distributed over the square.

    Attributes:
    ctrlSys: a CtrlSys object that computes the thruster forces

    note: This class is sort-of unnecessary right now, but if I
          build alternative 'controllers', it will be useful to
          have a wrapper that maintains the thruster location
          and direction data... (?)
    """

    def __init__(self, m, body_l):
        """Return a new Pod object"""

        # relative and scalable thrust vectors & thruster positions
        base_d = zip([1,1,-1,-1, 1, -1], [1,-1,-1,1, 0, 0], [0,0,0,0,0,0])
        base_r = zip([1,1,-1,-1, 1, -1], [1,-1,-1,1, 1, 1], [0,0,0,0,0,0])

        # r is a 6x3 matrix of the thruster positions
        # relative to the pod's center (center of mass)
        r = np.matrix(
            [[body_l * x, body_l * y, body_l * z] for x,y,z in base_r]
        )

        # d is a 6x3 matrix of the directions of the
        # force exerted by each thruster, relative
        # to the pod's center    
        d = np.matrix(
            [[x / np.sqrt(np.power(x,2) + np.power(y,2)),
              y / np.sqrt(np.power(x,2) + np.power(y,2)), z] for x,y,z in base_d]
        )
        
        # J is the moment of inertia matrix (needed to be
        ## calculated out by hand)
        J = np.identity(3) * m * np.power(body_l, 2) / 6

        # M is the mass matrix
        M = m * np.identity(3)
        
        self.ctrlSys = cs(M, J, r, d)

    def calcNewLocalVel(self, v, w, percent):
        self.ctrlSys.calcF(v, w, percent)
        return self.ctrlSys.predictDeltaV(v, w)

    def getThrusterForces(self):
        return self.ctrlSys.getF()


if __name__ == '__main__':
    p = Pod(40, 10)
    v = np.transpose(np.matrix([10,10,0]))
    w = np.transpose(np.matrix([0,0,10]))
    percent = .7
    p.ctrlSys.calcF(v,w,percent)
    print(p.ctrlSys.F)
