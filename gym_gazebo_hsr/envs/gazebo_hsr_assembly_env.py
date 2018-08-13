from gazebo_env import GazeboEnv
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
import hsrb_interface
from hsrb_interface import geometry
import random
import rospy
from std_srvs.srv import Empty
import time

model_names = ["base_plate", "compound_gear", "gear1", "gear2", "gear2_1", "shaft1", "shaft2"]
arm_joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
table_xmin, table_xmax, table_ymin, table_ymax, table_z = 0.9, 1.4, -0.5, 0.5, 1


class GazeboHsrAssemblyEnv(GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launch file
        GazeboEnv.__init__(self, "gazebo_hsr_assembly-v0.launch")
        self.robot = None
        self.model_states = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback)

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

        GazeboHsrAssemblyEnv._randomize_models(table_xmin, table_xmax, table_ymin, table_ymax, table_z)
        # TODO: reset the robot

    def _model_states_callback(self, data):
        self.model_states = data

    def _get_observation(self):
        return self.model_states

    def _set_robot_if_unset(self):
        if self.robot is None:
            self.robot = hsrb_interface.Robot()

    def _move(self, axis, distance):
        self._set_robot_if_unset()
        whole_body = self.robot.get('whole_body')
        whole_body.move_end_effector_by_line(axis, distance)

    def _gripper_apply_force(self, effort):
        self._set_robot_if_unset()
        gripper = self.robot.get('gripper', self.robot.Items.END_EFFECTOR)
        delicate = False
        if 0.2 <= effort <= 0.6:
            delicate = True
        gripper.apply_force(effort, delicate)

    def _gripper_command(self, open_angle):
        self._set_robot_if_unset()
        gripper = self.robot.get('gripper', self.robot.Items.END_EFFECTOR)
        gripper.command(open_angle)

    def _step(self, action):
        # Big move
        if action == 0:
            self._move(geometry.Vector3(1, 0, 0), 0.1)
        elif action == 1:
            self._move(geometry.Vector3(-1, 0, 0), 0.1)
        elif action == 2:
            self._move(geometry.Vector3(0, 1, 0), 0.1)
        elif action == 3:
            self._move(geometry.Vector3(0, -1, 0), 0.1)
        elif action == 4:
            self._move(geometry.Vector3(0, 0, 1), 0.1)
        elif action == 5:
            self._move(geometry.Vector3(0, 0, -1), 0.1)
        # Small moves
        elif action == 6:
            self._move(geometry.Vector3(1, 0, 0), 0.01)
        elif action == 7:
            self._move(geometry.Vector3(-1, 0, 0), 0.01)
        elif action == 8:
            self._move(geometry.Vector3(0, 1, 0), 0.01)
        elif action == 9:
            self._move(geometry.Vector3(0, -1, 0), 0.01)
        elif action == 10:
            self._move(geometry.Vector3(0, 0, 1), 0.01)
        elif action == 11:
            self._move(geometry.Vector3(0, 0, -1), 0.01)
        # Gripper
        elif action == 12:
            self._gripper_command(1.2)  # Open
        elif action == 13:
            self._gripper_command(0.0)  # Close
        elif action == 14:
            self._gripper_apply_force(0.5)
        elif action == 15:
            self._gripper_apply_force(1.0)

        else:
            raise NotImplementedError

        observation = self._get_observation()
        reward = 0
        done = False
        info = None
        return observation, reward, done, info
