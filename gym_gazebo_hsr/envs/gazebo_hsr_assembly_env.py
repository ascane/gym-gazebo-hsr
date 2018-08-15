import argparse
from gazebo_env import GazeboEnv
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
import hsrb_interface
from hsrb_interface import geometry
import random
import rospy
from std_srvs.srv import Empty
import time

parser = argparse.ArgumentParser()
parser.add_argument('--world', type=str,
                    help='optional, which world to launch, three_cubes or assembly', default='three_cubes')
args = parser.parse_args()

if args.world == 'three_cubes':
    model_names = ["box_red", "box_green", "box_blue"]
    launch_file = "gazebo_hsr_three_cubes-v0.launch"
elif args.world == 'assembly':
    model_names = ["base_plate", "compound_gear", "gear1", "gear2", "gear2_1", "shaft1", "shaft2"]
    launch_file = "gazebo_hsr_assembly-v0.launch"

arm_joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
table_xmin, table_xmax, table_ymin, table_ymax, table_z = 0.9, 1.4, -0.5, 0.5, 1


class GazeboHsrAssemblyEnv(GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launch file
        GazeboEnv.__init__(self, launch_file)
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
    def _reset_robot_position_orientation():
        model_state = ModelState()
        model_state.model_name = "hsrb"
        model_state.pose.position.x = 0
        model_state.pose.position.y = 0
        model_state.pose.position.z = 0
        model_state.pose.orientation.x = 0
        model_state.pose.orientation.y = 0
        model_state.pose.orientation.z = 0
        model_state.pose.orientation.w = 1
        model_state.reference_frame = "world"
        GazeboHsrAssemblyEnv._set_model_state(model_state)

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
        # Reset the robot
        try:
            GazeboHsrAssemblyEnv._reset_robot_position_orientation()
            whole_body = self._get_robot().get('whole_body')
            whole_body.move_to_neutral()
            self._gripper_command(0.3)  # Open the gripper a little bit
        except Exception as e:
            print("Exception: %s" % e)
        # Return initial OpenAI gym observation
        return self._get_observation()

    def _model_states_callback(self, data):
        self.model_states = data

    def _get_observation(self):
        return self.model_states

    def _get_robot(self):
        if self.robot is None:
            self.robot = hsrb_interface.Robot()
        return self.robot

    def _move(self, axis, distance):
        whole_body = self._get_robot().get('whole_body')
        whole_body.move_end_effector_by_line(axis, distance)

    def _gripper_apply_force(self, effort):
        robot = self._get_robot()
        gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
        delicate = False
        if 0.2 <= effort <= 0.6:
            delicate = True
        gripper.apply_force(effort, delicate)

    def _gripper_command(self, open_angle):
        robot = self._get_robot()
        gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
        gripper.command(open_angle)

    def _step(self, action):
        try:
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
        except Exception as e:
            print("Action %d is unavailable: %s" % (action, e))

        observation = self._get_observation()
        reward = 0
        done = False
        info = None
        return observation, reward, done, info
