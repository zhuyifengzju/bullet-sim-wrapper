
class BaseInterface():
    def __init__(self, bworld):
        self.physics = bworld.physics

class JointInterface():
    def __init__(self, bworld):
        super().__init__(bworld)

    def name(self, joint_uid):
        return self.physics.get_joint_name(joint_uid)
    
    def get_position(self, joint_uid):
        return self.physics.get_joint_position(joint_uid)

    def set_position(self, joint_uid, position):
        self.physics.set_joint_position(joint_uid, position)

    def get_velocity(self, joint_uid):
        return self.physics.get_joint_velocity(joint_uid)

    def set_velocity(self, joint_uid, velocity):
        self.physics.set_joint_velocity(velocity)

    def get_torque(self, joint_uid):
        return self.physics.get_joint_torque(joint_uid)

    # def set_torque(self, joint_uid, torque):
    #     self.physics.set_joint_torque(joint_uid, torque)


class LinkInterface():
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
