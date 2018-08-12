import gym
import rospy
import os
import signal
import subprocess
import time
from os import path
import random


class GazeboEnv(gym.Env):
    """Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile):
        random_number = random.randint(10000, 15000)
        self.port = '11311'  # str(random_number) # os.environ["ROS_PORT_SIM"]
        self.port_gazebo = str(random_number + 1)  # os.environ["ROS_PORT_SIM"]
        # os.environ["ROS_MASTER_URI"] = "http://localhost:" + self.port
        # os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + self.port_gazebo
        #
        # self.ros_master_uri = os.environ["ROS_MASTER_URI"];

        with open("log.txt", "a") as myfile:
            myfile.write("export ROS_MASTER_URI=http://localhost:" + self.port + "\n")
            myfile.write("export GAZEBO_MASTER_URI=http://localhost:" + self.port_gazebo + "\n")

        # Start roscore
        self.proc_core = subprocess.Popen(['roscore', '-p', self.port])
        time.sleep(1)
        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node('gym', anonymous=True)

        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", "launch", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        self.proc_launch = subprocess.Popen(['roslaunch', '-p', self.port, fullpath])
        print ("Gazebo launched!")

    def set_ros_master_uri(self):
        os.environ["ROS_MASTER_URI"] = self.ros_master_uri

    def _step(self, action):
        # Implement this method in every subclass
        # Perform a step in gazebo. E.g. move the robot
        raise NotImplementedError

    def _reset(self):
        # Implemented in subclass
        raise NotImplementedError

    def _render(self, mode="human", close=False):
        pass

    def _close(self):
        self.proc_launch.terminate()
        self.proc_core.terminate()
        self.proc_launch.wait()
        self.proc_core.wait()

    def _configure(self):
        # TODO
        # From OpenAI API: Provides runtime configuration to the enviroment
        # Maybe set the Real Time Factor?
        pass

    def _seed(self):
        # TODO
        # From OpenAI API: Sets the seed for this env's random number generator(s)
        pass
