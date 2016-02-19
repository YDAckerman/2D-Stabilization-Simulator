import numpy as np

class CtrlSys(object):

    """ A Control System object uses external velocity information
        to determine how to fire a Pod's thrusters for stabilization.

        Attributes:
        M: a numeric mass matrix
        J: a numeric moment of inertia matrix
        Gamma: a numeric matrix representing the pod's thrusters
               and forces.
        CapM: a numeric matrix combining M and J for computational ease
    """

    def __init__(self, M, J, r, d):
        """Return a CtrlSys object"""

        self.M = M
        self.J = J

        ## initialize the matrix of
        ## thruster directions and positions
        rxd = np.concatenate(
            [np.cross(r[i], d[i]) for i in range(len(r))]
        )
        self.Gamma = np.concatenate(
            [np.transpose(mat) for mat in [d, rxd]],
            axis = 0
        )

        ## initialize the matrix of moments of inertia
        ## and mass
        MO = np.concatenate([M, np.zeros([M.shape[0], J.shape[1]])],
            axis = 1
        )
        OJ = np.concatenate([np.zeros([J.shape[0], M.shape[1]]), J],
            axis = 1
        )
        self.CapM = np.concatenate([MO, OJ], axis = 0)

    def calcF(self, v, w, percent, report = False):
        X = np.concatenate([v, w], axis = 0)
        tGamma = np.transpose(self.Gamma)
        if report:
            print np.transpose(tGamma * X)
        self.F = np.transpose(
            np.matrix(
                [-percent * np.max([0, F_i]) for F_i in tGamma*X]
            ))

    def predictDeltaV(self, v, w):
        Xdot = np.linalg.inv(self.CapM) * ( -np.transpose(
            np.transpose(self.M * np.transpose(
                np.matrix(
                    np.cross(
                        np.transpose(w),
                        np.transpose(v)
                    )))) *
            np.transpose(np.matrix(
                np.cross(
                    np.transpose(w),
                    np.transpose(self.J * w)
                )))
            ) - self.Gamma * self.F)
        return Xdot

    def getF(self):
        return self.F
        
    
if __name__ == '__main__':
    
    body_l = 10
    m = 40
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
    
    # M is the mass matrix
    M = m * np.identity(3)
    
    # J is the moment of inertia matrix
    J = np.identity(3) * m * np.power(body_l, 2) / 6
    
    cs = CtrlSys(M, J, r, d)
    v = np.transpose(np.matrix([40,-40,0]))
    w = np.transpose(np.matrix([0,0,-40]))
    percent = .7
    cs.calcF(v, w, percent, report = True)
    print np.transpose(cs.F)
    
        
