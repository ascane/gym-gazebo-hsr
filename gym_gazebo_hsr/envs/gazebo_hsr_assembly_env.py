import argparse
import copy
from gazebo_env import GazeboEnv
from gazebo_msgs.msg import LinkStates, ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
import hsrb_interface
from hsrb_interface import geometry
import math
import numpy as np
import random
import rospy
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty
import time

parser = argparse.ArgumentParser()
parser.add_argument('--world', type=str,
                    help='optional, which world to launch, three_cubes or assembly', default='three_cubes')
args = parser.parse_args()

if args.world == 'three_cubes':
    launch_file = "gazebo_hsr_three_cubes-v0.launch"
    moving_object_model_names = ["box_red", "box_green", "box_blue"]
    model_names = ['ground_plane', 'low_table_rotate', 'box_red', 'box_green', 'box_blue', 'hsrb']
    link_names = ['ground_plane::link', 'low_table_rotate::link', 'box_red::link_1', 'box_green::link_2',
                  'box_blue::link_3', 'hsrb::base_footprint', 'hsrb::arm_lift_link', 'hsrb::arm_flex_link',
                  'hsrb::arm_roll_link', 'hsrb::wrist_flex_link', 'hsrb::wrist_ft_sensor_mount_link',
                  'hsrb::wrist_ft_sensor_frame', 'hsrb::hand_l_proximal_link', 'hsrb::hand_l_spring_proximal_link',
                  'hsrb::hand_l_mimic_distal_link', 'hsrb::hand_l_distal_link', 'hsrb::hand_motor_dummy_link',
                  'hsrb::hand_r_proximal_link', 'hsrb::hand_r_spring_proximal_link', 'hsrb::hand_r_mimic_distal_link',
                  'hsrb::hand_r_distal_link', 'hsrb::base_roll_link', 'hsrb::base_l_drive_wheel_link',
                  'hsrb::base_l_passive_wheel_x_frame', 'hsrb::base_l_passive_wheel_y_frame',
                  'hsrb::base_l_passive_wheel_z_link',
                  'hsrb::base_r_drive_wheel_link', 'hsrb::base_r_passive_wheel_x_frame',
                  'hsrb::base_r_passive_wheel_y_frame',
                  'hsrb::base_r_passive_wheel_z_link', 'hsrb::torso_lift_link', 'hsrb::head_pan_link',
                  'hsrb::head_tilt_link']
elif args.world == 'assembly':
    launch_file = "gazebo_hsr_assembly-v0.launch"
    moving_object_model_names = ["base_plate", "compound_gear", "gear1", "gear2", "gear2_1", "shaft1", "shaft2"]
    model_names = ['ground_plane', 'low_table_rotate', 'base_plate', 'compound_gear', 'gear1', 'gear2', 'gear2_1',
                   'shaft1', 'shaft2', 'hsrb']
    link_names = ['ground_plane::link', 'low_table_rotate::link', 'base_plate::link_0', 'compound_gear::link_1',
                  'gear1::link_2', 'gear2::link_4', 'gear2_1::link_3', 'shaft1::link_5', 'shaft2::link_6',
                  'hsrb::base_footprint', 'hsrb::arm_lift_link', 'hsrb::arm_flex_link', 'hsrb::arm_roll_link',
                  'hsrb::wrist_flex_link', 'hsrb::wrist_ft_sensor_mount_link', 'hsrb::wrist_ft_sensor_frame',
                  'hsrb::hand_l_proximal_link', 'hsrb::hand_l_spring_proximal_link', 'hsrb::hand_l_mimic_distal_link',
                  'hsrb::hand_l_distal_link', 'hsrb::hand_motor_dummy_link', 'hsrb::hand_r_proximal_link',
                  'hsrb::hand_r_spring_proximal_link', 'hsrb::hand_r_mimic_distal_link', 'hsrb::hand_r_distal_link',
                  'hsrb::base_roll_link', 'hsrb::base_l_drive_wheel_link', 'hsrb::base_l_passive_wheel_x_frame',
                  'hsrb::base_l_passive_wheel_y_frame', 'hsrb::base_l_passive_wheel_z_link',
                  'hsrb::base_r_drive_wheel_link',
                  'hsrb::base_r_passive_wheel_x_frame', 'hsrb::base_r_passive_wheel_y_frame',
                  'hsrb::base_r_passive_wheel_z_link',
                  'hsrb::torso_lift_link', 'hsrb::head_pan_link', 'hsrb::head_tilt_link']
else:
    raise NotImplementedError

model_names_dict = {model_names[i]: i for i in range(len(model_names))}
link_names_dict = {link_names[i]: i for i in range(len(link_names))}

arm_joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
hand_gripper_link_names = ['hsrb::arm_lift_link', 'hsrb::arm_flex_link', 'hsrb::arm_roll_link', 'hsrb::wrist_flex_link',
                           'hsrb::hand_l_mimic_distal_link', 'hsrb::hand_r_mimic_distal_link']
table_xmin, table_xmax, table_ymin, table_ymax, table_z = 0.9, 1.4, -0.5, 0.5, 1


class HsrState(object):
    def __init__(self):
        self.model_states = None  # gazebo_msgs.msgModelStates
        self.link_states = None  # gazebo_msgs.msgLinkStates
        self.hand_image = None  # int array
        self.head_center_image = None  # int array
        self.head_l_image = None  # int array
        self.head_r_image = None  # int array

    def to_array(self):
        result = []
        # self may be changed during to_array thus causing problems.
        hs = copy.deepcopy(self)
        # for model_name in moving_object_model_names + ['hsrb']:
        #     pose = self.model_states.pose[model_names_dict[model_name]]
        #     result.extend([pose.position.x, pose.position.y, pose.position.z])
        #     result.extend([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        # for link_name in hand_gripper_link_names:
        #     pose = self.model_states.pose[model_names_dict[link_name]]
        #     result.extend([pose.position.x, pose.position.y, pose.position.z])
        #     result.extend([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        for pose in hs.model_states.pose:
            result.extend([pose.position.x, pose.position.y, pose.position.z])
            result.extend([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        result.extend([0.0] * (7 * (len(model_names) - len(hs.model_states.pose))))
        for pose in hs.link_states.pose:
            result.extend([pose.position.x, pose.position.y, pose.position.z])
            result.extend([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        result.extend([0.0] * (7 * (len(link_names) - len(hs.link_states.pose))))
        result.extend(hs.head_l_image / 256.0)
        # The size of head_l_image varies. Pad with 0 to make the network input size a constant.
        result.extend([0.0] * (65536 - len(hs.head_l_image)))
        result.extend(hs.head_r_image / 256.0)
        # The size of head_r_image varies. Pad with 0 to make the network input size a constant.
        result.extend([0.0] * (65536 - len(hs.head_r_image)))

        return result


class GazeboHsrAssemblyEnv(GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launch file
        GazeboEnv.__init__(self, launch_file)
        self.robot = None
        self.state = HsrState()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self._link_states_callback)
        rospy.Subscriber("/hsrb/hand_camera/image_raw/compressed", CompressedImage, self._hand_image_callback)
        rospy.Subscriber("/hsrb/head_center_camera/image_raw/compressed", CompressedImage,
                         self._head_center_image_callback)
        rospy.Subscriber("/hsrb/head_l_stereo_camera/image_rect_color/compressed", CompressedImage,
                         self._head_l_image_callback)
        rospy.Subscriber("/hsrb/head_r_stereo_camera/image_rect_color/compressed", CompressedImage,
                         self._head_r_image_callback)

    def _model_states_callback(self, data):
        self.state.model_states = data

    def _link_states_callback(self, data):
        self.state.link_states = data

    def _hand_image_callback(self, data):
        self.state.hand_image = np.fromstring(data.data, np.uint8)

    def _head_center_image_callback(self, data):
        self.state.head_center_image = np.fromstring(data.data, np.uint8)

    def _head_l_image_callback(self, data):
        self.state.head_l_image = np.fromstring(data.data, np.uint8)

    def _head_r_image_callback(self, data):
        self.state.head_r_image = np.fromstring(data.data, np.uint8)

    @staticmethod
    def _set_model_state(model_state):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_model_state_client(model_state)
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
        for i, model_name in enumerate(moving_object_model_names):
            GazeboHsrAssemblyEnv._randomize_model_position(model_name, 10 + i, 10.1 + i, 10, 10.1, 0)
        for model_name in moving_object_model_names:
            GazeboHsrAssemblyEnv._randomize_model_position(model_name, xmin, xmax, ymin, ymax, z)
            time.sleep(1)

    def reset(self):
        self.state = HsrState()

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
        return self.state

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

    def step(self, action):
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

        observation = self.state
        reward = self._reward()
        done = False if reward < 0.9 else True
        info = None

        return observation, reward, done, info

    def get_model_pose(self, model_name):
        return self.state.model_states.pose[model_names_dict[model_name]]

    def get_link_pose(self, link_name):
        return self.state.link_states.pose[link_names_dict[link_name]]

    def get_hand_position(self):
        l_pos = self.get_link_pose('hsrb::hand_l_mimic_distal_link').position
        r_pos = self.get_link_pose('hsrb::hand_r_mimic_distal_link').position
        return geometry.vector3((l_pos.x + r_pos.x) * 0.5, (l_pos.y + r_pos.y) * 0.5, (l_pos.z + r_pos.z) * 0.5)

    @staticmethod
    def norm2(v1, v2):
        return math.sqrt(math.pow(v1.x - v2.x, 2) + math.pow(v1.y - v2.y, 2) + math.pow(v1.z - v2.z, 2))

    def _reward(self):  # Green box on top of red box
        if args.world == 'three_cubes':
            box_green_position = self.get_model_pose("box_green").position
            box_red_position = self.get_model_pose("box_red").position
            dist_z = box_green_position.z - box_red_position.z
            if dist_z < 0.09 or dist_z > 0.11:
                return 0
            box_red_position.z = box_green_position.z
            dist_xy = GazeboHsrAssemblyEnv.norm2(box_green_position, box_red_position)
            if dist_xy > 0.1:
                return 0
            return 1 if dist_xy < 0.05 else 1 - 10 * dist_xy
        else:
            raise NotImplementedError
