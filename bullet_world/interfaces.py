
class BaseInterface():
    def __init__(self, bworld):
        self.physics = bworld.physics

class JointInterface(BaseInterface):
    def __init__(self, bworld):
        super().__init__(bworld)

    def name(self, joint_uid):
        return self.physics.get_joint_name(joint_uid)
    
    def get_position(self, joint_uid):
        return self.physics.get_joint_position(joint_uid)

    def get_positions(self, joint_uids):
        joint_positions = []
        for joint_uid in joint_uids:
            joint_positions.append(self.get_position(joint_uid))
        return joint_positions

    def set_position(self, joint_uid, position):
        self.physics.set_joint_position(joint_uid, position)

    def get_velocity(self, joint_uid):
        return self.physics.get_joint_velocity(joint_uid)
    
    def set_velocity(self, joint_uid, velocity):
        self.physics.set_joint_velocity(velocity)

    def get_torque(self, joint_uid):
        return self.physics.get_joint_torque(joint_uid)

    def get_num_joints(self, body_uid):
        return self.physics.get_num_joints(body_uid)

    def position_control(self, joint_uid, target_position, **kwargs):
        self.physics.position_control(joint_uid, target_position, **kwargs)
    
    # def set_torque(self, joint_uid, torque):
    #     self.physics.set_joint_torque(joint_uid, torque)


class LinkInterface(BaseInterface):
    def __init__(self, bworld):
        super().__init__(bworld)

    def name(self, link_uid):
        return self.physics.get_link_name(link_uid)

    def get_pose(self, link_uid):
        return self.physics.get_link_pose(link_uid)

    def local_offset(self, link_uid):
        return self.physics.get_link_local_offset(link_uid)

    def com(self, link_uid):
        return self.physics.get_link_center_of_mass(link_uid)

    def ik_joints(self,
                  link_uid,
                  link_pose,
                  **kwargs):
        return self.physics.compute_inverse_kinematics(link_uid=link_uid,
                                                       link_pose=link_pose,
                                                       **kwargs)
    
class DynamicsParams():
    def __init__(self, bworld, joint_interface, link_interface):
        self.physics = bworld.physics
        self.joint_interface = joint_interface
        self.link_interface = link_interface
        
    def zero_jacobian(self, joint_uids, ee_link_uid, decoupled=True):
        """
        Args:
           arm_joint_positions (list or np.array): positions of arm joints
           ee_link_uid (tuple): (body_uid, link_ind)
           decoupled (bool): if True, return a tuple of (jac_tr, jac_r), else a whole jacobian
        
        """
        body_uid, link_ind = ee_link_uid
        joint_positions = self.joint_interface.get_positions(joint_uids)
        com = self.link_interface.local_offset(ee_link_uid)
        zero_vec = [0.] * len(joint_positions)

        jac_tr, jac_r = self.physics.calculate_jacobian(ee_link_uid, com.position, joint_positions, zero_vec, zero_vec)
        
        if decoupled:
            return (jac_tr, jac_r)
        else:
            return np.concatenate((jac_tr, jac_r), dim=0)

    def mass(self, body_uid, joint_uids):
        joint_positions = self.joint_interface.get_positions(joint_uids)
        mass = self.physics.calculate_mass_matrix(body_uid, joint_positions)
        return mass
