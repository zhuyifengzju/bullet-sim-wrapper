import abc
from bullet_world import Joint, Link
from bullet_world.utils import YamlConfig

class RobotArm():
    """This is just a wrapper to get access to joints / links easily."""
    def __init__(self, config_file, bworld):
        self.config = YamlConfig(config_file).as_easydict()

        # Specify ee link name
        self.ee_link_name = self.config.EE.LINK_NAME

        self.init_joint_positions = self.config.NEUTRAL_JOINT_POSITIONS

        self.urdf_name = self.config.URDF_NAME
        self.assets_dir = None
        
        if self.config.ASSETS_DIR != "default":
            self.assets_dir = self.config.ASSETS_DIR

        self.uid = bworld.add_body(self.urdf_name, is_static=True, assets_dir=self.assets_dir)

        self._joints = [(self.uid, joint_ind) for joint in bworld.physics.get_body_joint_indices(self.uid)]
        # Load arm links
        self._links = [(self.uid, link_ind) for link in bworld.physics.get_body_link_indices(self.uid)]

        # Load arm joints
        self._arm_joint_names = self.config.ARM.JOINT_NAMES

        # name -> (body_uid, joint_ind)
        self._arm_joint_mapping = {}
        for joint in self._joints:
            name = bworld.physics.get_joint_name(self.uid)
            if name in self._arm_joint_names:
                self._arm_joint_mapping[name] = joint

        self.ee_link = None
        for link in self._links:
            name = bworld.physics.get_link_name(self.uid)
            if name == self.ee_link_name:
                self.ee_link = link
                break
        assert(self.ee_link is not None, "link name not found")

    @abc.abstractmethod
    def zero_jacobian(self):
        raise NotImplementedError

    @abc.abstractmethod
    def mass(self):
        raise NotImplementedError


    
