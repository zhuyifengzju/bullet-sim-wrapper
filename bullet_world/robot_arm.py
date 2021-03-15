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
        self.phyiscs = bworld.physics
        
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

        neutral_positions = [0., 0.458, 0.31, -2.24, -0.30, 2.66, 2.32]
        for (i, joint_uid) in enumerate(self.arm_joints):
            self.interfaces.joints.set_position(joint_uid, neutral_positions[i])
                
        self.ee_link = None
        for link in self._links:
            name = bworld.physics.get_link_name(link)
            if name == self.ee_link_name:
                self.ee_link = link
                break
        assert(self.ee_link is not None, "link name not found")

        self.position_control_param = {"position_gain": None,
                                       "velocity_gain": None}
    # def motion_plan(self, plan_seq, type="pose"):

    def compute_ik_joints(self, pose):
        target_joints = self.interfaces.links.ik_joints(self.ee_link, pose)
        return target_joints

    def set_position_control_param(self,
                                   position_gain,
                                   velocity_gain):
        self.position_control_param["position_gain"] = position_gain
        self.position_control_param["velocity_gain"] = velocity_gain
    
    def set_position_control_target(self, target_positions, arm_only=True):
        if arm_only:
            positions = []
            for (i, joint_uid) in enumerate(self.arm_joints):
                self.interfaces.joints.position_control(joint_uid, target_positions[i], **self.position_control_param)
                positions.append(target_positions[i])
        else:
            raise NotImplementedError

    
    @property
    def arm_joints(self):
        return self._arm_joints

    @property
    def joints(self):
        return self._joints
    
    @property
    def joint_positions(self):
        return self.interfaces.joints.get_positions(self.joints)

    @property
    def zero_decoupled_jacobian(self):
        return self.interfaces.dynamics.zero_jacobian(self.joints, self.ee_link, decoupled=True)

    @property
    def zero_coupled_jacobian(self):
        return self.interfaces.dynamics.zero_jacobian(self.joints, self.ee_link, coupled=True)

    @property
    def mass_matrix(self):
        return self.interfaces.dynamics.mass(self.uid, self.joints)
