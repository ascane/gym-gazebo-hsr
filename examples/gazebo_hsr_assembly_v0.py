import time
import gym

if __name__ == "__main__":
    env = gym.make('gazebo-hsr-assembly-v0')
    # TODO: How to wait until the env has been launched properly?
    # time.sleep(10)
    env.reset()
    env.close()
