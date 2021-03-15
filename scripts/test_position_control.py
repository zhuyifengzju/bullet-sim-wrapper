import _init_paths
from bullet_world import BulletWorld, ViSIIBulletWorld
import cv2

env = BulletWorld(default_init=True)

target_joint_positions = env.robot_arms[0].compute_ik_joints([[-0.3, 0.5, 1.2], [1., 0., 0., 0.]])
print(target_joint_positions)
env.robot_arms[0].set_position_control_target(target_joint_positions)
for _ in range(1000):
    import time; time.sleep(0.1)
    env.step_simulation()
