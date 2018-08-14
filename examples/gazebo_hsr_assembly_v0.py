import gym
import gym_gazebo_hsr  # Don't remove this. It registers the env.
import signal
import time


if __name__ == "__main__":
    env = gym.make('gazebo-hsr-assembly-v0')

    env.reset()

    env.step(0)
    env.step(1)

    env.render()

    env.step(2)
    env.step(3)
    env.step(4)
    env.step(5)
    env.step(6)

    env.render(close=True)

    env.step(7)
    env.step(8)
    env.step(9)
    env.step(10)
    env.step(11)
    env.step(12)

    env.render()

    env.step(13)
    env.step(14)
    env.step(15)

    env.reset()

    env.render()

    time.sleep(5)

    env.close()

    signal.pause()
