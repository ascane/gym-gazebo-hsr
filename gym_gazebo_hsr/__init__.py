from gym.envs.registration import register

register(
    id='gazebo-hsr-assembly-v0',
    entry_point='gym_gazebo_hsr.envs:GazeboHsrAssemblyEnv',
)
