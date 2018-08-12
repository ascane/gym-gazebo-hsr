from gazebo_env import GazeboEnv
import rospy
import random
import time
from actionlib import SimpleActionClient
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState, ModelStates
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

model_names = ["base_plate", "compound_gear", "gear1", "gear2", "gear2_1", "shaft1", "shaft2"]
arm_joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]


class GazeboHsrAssemblyEnv(GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launch file
        GazeboEnv.__init__(self, "gazebo_hsr_assembly-v0.launch")
        self.model_states = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback)
        pass

    @staticmethod
    def _set_model_state(modelState):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_model_state_client(modelState)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    @staticmethod
    def _randomize_model_position(model_name, xmin, xmax, ymin, ymax, z):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = random.uniform(xmin, xmax)
        model_state.pose.position.y = random.uniform(ymin, ymax)
        model_state.pose.position.z = z
        model_state.reference_frame = "world"
        GazeboHsrAssemblyEnv._set_model_state(model_state)

    @staticmethod
    def _randomize_models(xmin, xmax, ymin, ymax, z):
        # Put the objects somewhere else to prevent obstructing other objects.
        for i, model_name in enumerate(model_names):
            GazeboHsrAssemblyEnv._randomize_model_position(model_name, 10 + i, 10.1 + i, 10, 10.1, 0)
        for model_name in model_names:
            GazeboHsrAssemblyEnv._randomize_model_position(model_name, xmin, xmax, ymin, ymax, z)
            time.sleep(1)

    def _reset(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world_client = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world_client()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        GazeboHsrAssemblyEnv._randomize_models(0.9, 1.4, -0.5, 0.5, 1)
        # TODO: reset the robot

    def _model_states_callback(self, data):
        self.model_states = data

    def _get_observation(self):
        return self.model_states

    def _step(self, action):
        observation = self._get_observation()
        reward = 0
        done = False
        info = None
        return observation, reward, done, info
