import numpy as np
import time

from perls2.robot.real_panda_interface import RealPandaInterface
from perls2.utils.yaml_config import YamlConfig


def millis_to_secs(millis):
    return 0.001 * millis


def main():
    config_file = "demos/adam_cfg.yaml"
    config = YamlConfig(config_file)
    robot = RealPandaInterface(config, controlType="JointVelocity")

    # robot.connect()  # already done by above
    robot.reset()

    q0 = np.copy(robot.q)
    K = 0.5 * np.eye(7)
    amp = 0.1
    freq = 2

    millis = 0
    N = 3000
    for i in range(N):
        # joint feedback
        q = robot.q

        # desired joint angles
        secs = millis_to_secs(i)
        qd = q0 + [0, 0, 0, 0, 0, amp * (1 - np.cos(freq * secs)), 0]
        vd = np.array([0, 0, 0, 0, 0, amp * freq * np.sin(freq * secs), 0])

        # joint velocity controller
        # TODO experiment with just sending zero commands, holding object, etc.
        v = K @ (qd - q) + vd

        # send the command
        robot.set_joint_velocities(v)

        millis += i
        time.sleep(millis_to_secs(1))

    # put robot back to home position
    robot.reset()
    robot.disconnect()


if __name__ == "__main__":
    main()
