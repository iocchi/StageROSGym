import os, sys
import random

sys.path.append(os.getenv('MARRTINO_APPS_HOME')+'/program')

import robot_cmd_ros
from robot_cmd_ros import *

# rosrun stage_environments start_simulation.py --no_gui DISB1
# roslaunch $MARRTINO_APPS_HOME/navigation/obstacle_avoidance.launch max_vel_x:=0.3
# rosparam set /gradientBasedNavigation/max_vel_x 0.3
#
# cd playground
# python <thisfile>
# ...
#
# rosrun stage_environments quit.sh



class StageEnv:

    def __init__(self):
        # current velocities
        self.tv = 0
        self.rv = 0
        self.dt = 0.2
        self.state = [0, 0, 0, 0, 0]  # x,y,th,tv,rv
        self.actions = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7']

    def a1(self):
        self.tv += 0.2
        return setSpeed(self.tv,self.rv,self.dt,False)

    def a2(self):
        self.tv += 0.1
        return setSpeed(self.tv,self.rv,self.dt,False)

    def a3(self):
        self.tv -= 0.1
        return setSpeed(self.tv,self.rv,self.dt,False)

    def a4(self):
        self.rv += 0.1
        return setSpeed(self.tv,self.rv,self.dt,False)

    def a5(self):
        self.rv -= 0.1
        return setSpeed(self.tv,self.rv,self.dt,False)

    def a6(self):
        self.rv = 0.0
        return True

    def a7(self):
        return True

    def getstate(self):
        p = getRobotPose(frame='gt')
        v = getRobotVel()
        #print("%.2f %.2f %.2f %.2f %.2f" %(p[0],p[1],p[2],v[0],v[1]))
        self.state = [p[0],p[1],p[2],v[0],v[1]]

    def reset(self):
        gx=10
        gy=2
        gth=270 # deg
        stage_setpose(gx,gy,gth)
        wait(0.5)
        return self.getstate()


    def action_sample(self):
        return random.choice(self.actions)

    def step(self,a):
        astr = 'self.'+a+'()'
        done = not eval(astr)
        s = self.getstate()
        r = 0
        return s, r, done

    def close(self):
        pass

def episode(env):

    print('\nStart')
    s = env.reset()
    done = False
    i=0
    n=90
    while i<n and not done:
      a = env.action_sample()
      s,r,done = env.step(a)
      i+=1
    stop()
    return not done


begin()

setMaxSpeed(0.5,1.0)
enableObstacleAvoidance(True)

env = StageEnv()

r=True
i=0
n=3
while i<n and r:
    r = episode(env)
    i += 1

env.reset()
env.close()

end()

