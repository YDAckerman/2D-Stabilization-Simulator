# unit tests for 2D space stabilization

import numpy as np
from run import SpacePod as spd
from CtrlSys import CtrlSys as cs

def test_local_control(ctrlsys):
    ## these test take the form of
    ## randomly generated v and w
    ## arrays to perform control on.
    ## a test fails if the return values
    ## have a norm larger than the input values
    print "Running main local control tests"
    ret = False
    vs = [np.transpose(np.matrix([ np.power(-1, i % 3) * np.random.randint(60),
                                   np.power(-1, i % 2) * np.random.randint(60),
                                   0])) for i in range(10)]
    ws = [np.transpose(np.matrix([ 0,
                                   0,
                                   np.power(-1, i % 2) * np.pi / 
                                   np.random.randint(1, 60)])) for i in range(10)]

    for i in range(10):
        X = np.concatenate([vs[i], ws[i]], axis = 0)
        ctrlsys.calcF(vs[i], ws[i], percent = .7)
        Xdot = ctrlsys.predictDeltaV(vs[i], ws[i])
        if np.linalg.norm(X - Xdot) >= np.linalg.norm(X):
            print "test failed with v and w values:"
            print vs[i], ws[i]
            ret = True
    if not ret:
        print "Main local test passed"
    return ret

def test_global_control(spacepod):
    ## these test take the form of
    ## randomly generated v, w, and positional
    ## orienation
    ## arrays to perform control on.
    ## the additional caveat here is that
    ## the internal control system will be
    ## oriented in varying ways.
    ## again, a test fails if the return values
    ## have a norm larger than the input values
    print "Running main global control tests"
    ret = False
    vs = [np.transpose(np.matrix([ np.power(-1, i % 3) *
                                   np.random.randint(60),
                                   np.power(-1, i % 2) *
                                   np.random.randint(60)])) for i in range(10)]
    ws = [np.power(-1, i % 2) * np.pi /
          np.random.randint(1, 60) for i in range(10)]
    os = [np.power(-1, i % 3) * np.pi /
          np.random.randint(1, 60) for i in range(10)]

    for i in range(10):
        X = np.concatenate([
            np.concatenate([vs[i],np.matrix(0)], axis = 0),
            np.concatenate([np.matrix(0),np.matrix(0),
                            np.matrix(ws[i])])], axis = 0)
        spacepod.setOrientation(os[i])
        new_v, new_w = spacepod.calcNewGlobalVel(vs[i], ws[i])
        new_X = np.concatenate([new_v, np.matrix(new_w)], axis = 0)
        if np.linalg.norm(new_X) >= np.linalg.norm(X):
            print "test failed with v, w, and o values:"
            print vs[i], ws[i], os[i]
            ret = True

    if not ret:
        print "All global tests passed"
    return ret

def test_local_angular_control(ctrlsys):
    ## These tests check that local angular velocity is reduced
    ## by the controller when linear velocity is 0
    print "Running local angular velocity tests"
    ret = False
    vs = np.transpose(np.matrix([0,0,0]))
    ws = [np.transpose(np.matrix([ 0,
                                   0,
                                   np.power(-1, i % 2) * np.pi /
                                   np.random.randint(1, 60)])) for i in range(10)]
    
    for i in range(10):
        X = np.concatenate([vs, ws[i]], axis = 0)
        ctrlsys.calcF(vs, ws[i], percent = .7)
        Xdot = ctrlsys.predictDeltaV(vs, ws[i])
        if np.linalg.norm(X - Xdot) >= np.linalg.norm(X):
            print "test failed with w value:"
            print ws[i]
            ret = True
    if not ret:
        print "local angular velocity tests passed"
    return ret


def test_local_linear_control(ctrlsys):
    ## These tests check that local angular velocity is reduced
    ## by the controller when linear velocity is 0
    print "Running local linear velocity tests"
    ret = False
    vs = [np.transpose(np.matrix([ np.power(-1, i % 3) *
                                   np.random.randint(60),
                                   np.power(-1, i % 2) *
                                   np.random.randint(60),
                                   0])) for i in range(10)]
    
    ws = np.transpose(np.matrix([0,0,0]))

    for i in range(10):
        X = np.concatenate([vs[i], ws], axis = 0)
        ctrlsys.calcF(vs[i], ws, percent = .7)
        Xdot = ctrlsys.predictDeltaV(vs[i], ws)
        if np.linalg.norm(X - Xdot) >= np.linalg.norm(X):
            print "test failed with v value:"
            print vs[i]
            ret = True
    if not ret:
        print "local linear velocity tests passed"
    return ret


def test_global_angular_control(spacepod):
    ## These tests check that global angular velocity is reduced
    ## by the controller when linear velocity is 0
    print "Running global angular velocity tests"
    ret = False
    vs = np.transpose(np.matrix([0,0]))
    ws = [np.power(-1, i % 2) * np.pi /
          np.random.randint(1, 60) for i in range(10)]

    os = [np.power(-1, i % 3) * np.pi /
          np.random.randint(1, 60) for i in range(10)]

    for i in range(10):
        X = np.concatenate([
            np.concatenate([vs,np.matrix(0)], axis = 0),
            np.concatenate([np.matrix(0),np.matrix(0),
                            np.matrix(ws[i])])], axis = 0)
        spacepod.setOrientation(os[i])
        new_v, new_w = spacepod.calcNewGlobalVel(vs, ws[i])
        new_X = np.concatenate([new_v, np.matrix(new_w)], axis = 0)
        if np.linalg.norm(new_X) >= np.linalg.norm(X):
            print "test failed with w, and o values:"
            print ws[i], os[i]
            ret = True

    if not ret:
        print "global angular velocity tests passed"
    return ret

def test_global_linear_control(spacepod):
    ## These tests check that global angular velocity is reduced
    ## by the controller when linear velocity is 0
    print "Running global linear velocity tests"
    ret = False
    vs = [np.transpose(np.matrix([ np.power(-1, i % 3) *
                                   np.random.randint(60),
                                   np.power(-1, i % 2) *
                                   np.random.randint(60)])) for i in range(10)]
    ws = 0

    os = 0

    for i in range(10):
        X = np.concatenate([vs[i], np.matrix(ws)], axis = 0)
        spacepod.setOrientation(os)
        new_v, new_w = spacepod.calcNewGlobalVel(vs[i], ws)
        new_X = np.concatenate([new_v, np.matrix(new_w)], axis = 0)
        if np.linalg.norm(new_X) >= np.linalg.norm(X):
            print "test failed with v value:"
            print vs[i]
            ret = True

    if not ret:
        print "global linear velocity tests passed"
    return ret



if __name__ == '__main__':

    pos = np.transpose(np.matrix([200, 200]))
    pod = spd(pos, 500, 20, 0)

    ## test the controller with angular velocity
    test_local_angular_control(pod.pd.ctrlSys)
    ## test the controller with linear velocity
    test_local_linear_control(pod.pd.ctrlSys)
    
    ## test the controller with global angular velocity
    test_global_angular_control(pod)
    ## test the controller with global linear velocity
    test_global_linear_control(pod)

    ## test the controller:
    test_local_control(pod.pd.ctrlSys)

    ## test controller from globals frame
    test_global_control(pod)

