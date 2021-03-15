import _init_paths
from bullet_world import BulletWorld, ViSIIBulletWorld
import cv2

bulletworld = BulletWorld(default_init=True)


# print(bulletworld.robot_arms[0].zero_decoupled_jacobian)

# for _ in range(1000):
#     img = bulletworld._physics.get_camera_image(800, 800)
#     cv2.imshow("", img)
#     cv2.waitKey(1)
#     import time; time.sleep(1.0)

# bulletworld.step_simulation()
# input()

# visii_bulletworld = ViSIIBulletWorld(default_init=True)
# visii_bulletworld.render()


input()
