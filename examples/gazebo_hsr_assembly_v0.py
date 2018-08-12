import gym
import gym_gazebo_hsr  # Don't remove this. It registers the env.
import signal


if __name__ == "__main__":
    env = gym.make('gazebo-hsr-assembly-v0')
    # env.reset()
    signal.pause()
