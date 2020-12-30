from bullet import BulletPhysics, Body
from bullet.math_utils import Pose

class BulletWorld():
    def __init__(self, assets_dir="assets/"):
        self._physics = BulletPhysics()
        self._physics.start()
        self._assets_dir = assets_dir
        
        self.default_initialization()
        
    def step_simulation(self):
        self._physics.step()

    def add_body(self, file_name, pose=Pose(), scale=1.0):
        body_uid = self._physics.add_body(file_name, pose=pose, scale=scale)
        return body_uid
        
    def default_initialization(self):
        self.add_body(self._assets_dir + 'envs/planes/plane.urdf')
        self.add_body(self._assets_dir + 'robots/panda/panda.urdf')
        self.add_body(self._assets_dir + 'ycb/004_sugar_box/google_16k/textured.obj', scale=0.01)
        
    # def save(self):
