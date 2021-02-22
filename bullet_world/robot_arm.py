import abc
from bullet_world.utils import YamlConfig
from bullet_world.math_utils import Pose

class RobotArm():
    """This is just a wrapper to get access to joints / links easily.

    Args:
       interfaces: EasyDict, containing all interface objects

    """
    def __init__(self, config_file, bworld, interfaces, init_pose=Pose([[0.0, 0.0, 0.5], [0., 0., 0., 1.]])):
        self.config = YamlConfig(config_file).as_easydict()

        # interfaces
        self.interfaces = interfaces
        
        # Specify ee link name
        self.ee_link_name = self.config.EE_NAME

        self.init_joint_positions = self.config.NEUTRAL_JOINT_POSITIONS

        self.urdf_name = self.config.URDF_NAME
        self.assets_dir = None
        
        if self.config.ASSETS_DIR != "default":
            self.assets_dir = self.config.ASSETS_DIR

        self.uid = bworld.add_body(self.urdf_name, pose=init_pose, is_static=True, assets_dir=self.assets_dir)

        self._joints = [(self.uid, joint_ind) for joint_ind in bworld.physics.get_body_joint_indices(self.uid)]
        # Load arm links
        self._links = [(self.uid, link_ind) for link_ind in bworld.physics.get_body_link_indices(self.uid)]

        # Load arm joints
        self._arm_joint_names = self.config.ARM.JOINT_NAMES

        # name -> (body_uid, joint_ind)
        self._arm_joint_mapping = {}
        self._arm_joints = []
        for joint in self._joints:
            name = bworld.physics.get_joint_name(joint)
            if name in self._arm_joint_names:
                self._arm_joint_mapping[name] = joint
                self._arm_joints.append(joint)

        self.ee_link = None
        for link in self._links:
            name = bworld.physics.get_link_name(link)
            if name == self.ee_link_name:
                self.ee_link = link
                break
        assert(self.ee_link is not None, "link name not found")

    @property
    def arm_joints(self):
        return self._arm_joints

    @property
    def joints(self):
        return self._joints

    @property
    def zero_decoupled_jacobian(self):
        return self.interfaces.dynamics.zero_jacobian(self.joints, self.ee_link, decoupled=True)

    @property
    def zero_coupled_jacobian(self):
        return self.interfaces.dynamics.zero_jacobian(self.joints, self.ee_link, coupled=True)

    @property
    def mass_matrix(self):
        return self.interfaces.dynamics.mass(self.uid, self.joints)
