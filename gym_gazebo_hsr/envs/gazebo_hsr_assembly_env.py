from gazebo_env import GazeboEnv


class GazeboHsrAssemblyEnv(GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launch file
        GazeboEnv.__init__(self, "gazebo_hsr_assembly-v0.launch")
        pass

    def _step(self, action):
        pass

    def _reset(self):
        pass
