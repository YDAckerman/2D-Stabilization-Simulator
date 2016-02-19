"""
The run a simulation for a Space-pod with
controlled stabilization. In this file the SpacePod
class is defined.
"""

from graphics import *
from Pod import Pod as Pd
import numpy as np
import random, time

win_size = 500
fps_limit = 240

class SpacePod:
    def __init__(self, position, m, l, theta):
        """init SpacePod object 
           with a position, length, and orientation
        """
        # x,y are the center of the pod
        # position is an [2,1] np.array:
        self.position = position % win_size
        # l is the length of a side of the pod
        self.l = l
        # m is the mass of the pod
        self.m = m
        # theta is the angle of the 'front'
        # to the center (in radians!)
        self.theta = theta % (2 * np.pi)
        # set our graphics body object
        self.setBody()
        # set the 'internal' pod and its control system
        self.pd = Pd(m, l)

    def setOrientation(self, theta):
        self.theta = theta

    def setBody(self):
        """create a graphics polygon representing the SpacePod
        """
        # create a rotation matrix
        rotMat = np.matrix([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]               
        ])

        # create a body matrix relative to the pod's center of mass
        bodyMat = np.matrix([
            [ -self.l / 2, -self.l / 2],
            [ self.l / 2,  self.l / 2],
            [ -self.l / 2, self.l / 2],
            [ self.l / 2, -self.l / 2]
        ])

        bodyMat.shape = (2,4)

        # rotate and translate the bodyMat to create the
        # pod's position
        posMat = (rotMat * bodyMat + self.position) % win_size

        # convert to vertices for graphics lib
        vertices = [Point(int(posMat[0, i]),
                          int(posMat[1, i])) for i in range(4)]

        self.body = Polygon(vertices)

    def display(self):
        self.body.draw(win)

    def erase(self):
        self.body.undraw()

    def move(self, dpos, dtheta):
        self.position = (self.position + dpos) % win_size
        self.theta += dtheta
        self.setBody()

    def calcNewGlobalVel(self, v, w):
        percent = .7 # set this locally for now

        rotMat_to_bf = np.matrix([
            [np.cos(self.theta), np.sin(self.theta)],
            [-np.sin(self.theta), np.cos(self.theta)]               
        ])

        rotMat_from_bf = np.matrix([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]               
        ])

        # change v to body-fixed coords
        bf_v = rotMat_to_bf * v
        v_calc = np.concatenate([bf_v, np.matrix([0])], axis = 0)
        w_calc = np.transpose(np.concatenate([np.matrix([0,0]),
                                              np.matrix([w])], axis = 1))
        # calculate thruster forces and
        # estimate the changes in velocity
        vdot = self.pd.calcNewLocalVel(v_calc, w_calc, percent)

        # change the velocities accordingly
        # and rotate out of the body-fixed frame
        new_bf_v = (v_calc - vdot[0:3])[0:2]
        new_v = rotMat_from_bf * new_bf_v
        new_w = float((w_calc - vdot[3:6])[2])
        return new_v, new_w

def checkEdgePos(pod):
        return (np.sum(pod.position <= pod.l / np.sqrt(2)) +
                np.sum(pod.position >= win_size - pod.l / np.sqrt(2))) >= 1

def motion(event):
    x, y = event.x, event.y
    print('{}, {}'.format(x, y))

if __name__ == '__main__':

    win = GraphWin("Spacepod Stabilizer!", win_size, win_size)
    win.bind('<Motion>', motion) # works!
    run_me = True
    pos = np.transpose(np.matrix([win_size / 2, win_size / 2]))
    pod = SpacePod(pos, 500, 20, np.pi/2)

    # keep v in pixels per second
    v = np.transpose(np.matrix([-40,40])) / fps_limit
    theta = np.pi / (.5 * fps_limit)
    init_time = time.time()
    change_time = time.time()
    while run_me:
        
        # do work at the frame rate
        time.sleep(1 / fps_limit)

        # record current time
        cur_time = time.time()
        
        # change the auto flush settings
        # and flush changes to the screen
        # after making the new drawing
        win.autoflush = False
        if not checkEdgePos(pod) :
            pod.display()
            win.flush()
            pod.erase()
            win.autoflush = True

        # move the pod
        pod.move(v, theta)

        # calculate the thruster forces
        # and update velocity
        v, theta = pod.calcNewGlobalVel(v, theta)

        if cur_time - change_time > 17:
            v = np.transpose(np.matrix([np.random.randint(-40, 40) / fps_limit,
                                        np.random.randint(-40, 40) / fps_limit]))
            theta = np.pi / (np.random.rand() * fps_limit)
            change_time = cur_time
        # only run for 1 minute
        stop = True and cur_time - init_time > 60
        stop = stop and np.linalg.norm(v) * fps_limit <= 1
        stop = stop and np.abs(theta) * fps_limit * 2 * np.pi / 360 <= 2
        
        if stop:
            run_me = False
            win.close()
