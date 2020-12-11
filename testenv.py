import os, sys
import random
from datetime import datetime

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

start_pose = [2, 2, 0]
time_limit = 10
n_episodes = 1000

max_tv = 0.5
max_rv = 0.5

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
        self.rv += 0.05
        return setSpeed(self.tv,self.rv,self.dt,False)

    def a5(self):
        self.rv -= 0.05
        return setSpeed(self.tv,self.rv,self.dt,False)

    def a6(self):
        self.rv = 0.0
        wait(self.dt)        
        return True

    def a7(self):
        wait(self.dt)
        return True

    def getstate(self):
        p = getRobotPose(frame='gt')
        v = getRobotVel()
        #print("%.2f %.2f %.2f %.2f %.2f" %(p[0],p[1],p[2],v[0],v[1]))
        self.state = [p[0],p[1],p[2],v[0],v[1]]
        return self.state

    def reset(self):
        global start_pose
        stage_setpose(start_pose[0],start_pose[1],start_pose[2])
        self.tv = 0
        self.rv = 0
        wait(0.5)
        return self.getstate()


    def action_sample(self):
        return 'a2' #random.choice(self.actions)

    def step(self,a):
        astr = 'self.'+a+'()'
        done = not eval(astr)
        s = self.getstate()
        r = 0
        return s, r, done

    def close(self):
        pass

def episode(env):

    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%Y%m%d-%H%M%S")
    print('Current Timestamp %s' %timestampStr)

    #f = open('data/%s.csv' %(timestampStr), 'w')

    print('\nStart')
    s = env.reset()
    done = False
    t = rospy.get_rostime()
    tend = t.secs + time_limit
    while t.secs<tend and not done:
      a = env.action_sample()
      s,r,done = env.step(a)
      loge = "%06d.%03d ; %.2f ; %.2f ; %.2f ; %.2f ; %.2f; %s\n" \
            %(t.secs,t.nsecs/1e6,s[0],s[1],s[2],s[3],s[4],str(a))
      print(loge)
      #f.write(loge)
      t = rospy.get_rostime()
    stop()
    #f.close()
    return not done


begin()

os.system("rosparam set /gradientBasedNavigation/max_vel_x %.2f" %max_tv)
setMaxSpeed(max_tv,max_rv)
enableObstacleAvoidance(True)


env = StageEnv()

r=True
i=0
while i<n_episodes and r:
    r = episode(env)
    i += 1

env.reset()
env.close()

end()

