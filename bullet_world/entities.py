

class Body():
    """Body.

    Args:
      physics_client (BulletPhysics): The BulletPhysics object
      body_uid (int): Unique id of a body.
      body_name (str): Name of the body added to the simulation
    """
    def __init__(self, physics_client, body_uid, body_name):
        self.uid = body_uid
        self.client = physics_client
        self.name = body_name

        link_indices = self.client.get_body_link_indices(self.uid)
        joint_indices = self.client.get_body_joint_indices(self.uid)

        self.links = {}
        self.joints = {}
        
        for link_index in link_indices:
            link_name = self.client.get_link_name(link_index)
            self.links[link_name] = Link(self.client, body_uid, link_index)

        for joint_index in joint_indices:
            joint_name = self.client.get_joint_name(joint_index)
            self.joints[joint_name] = Joint(self.client.joint_uid, joint_index)

    @property
    def pose(self):
        return self.client.get_body_pose(self.uid)

    @pose.setter
    def pose(self, value):
        self.client.set_body_pose(self.uid, value)


    @property
    def joint_positions(self):
        return [joint.position for joint in self.joints.values()]

    @joint_positions.setter
    def joint_positions(self, value):
        for joint, joint_position in zip(self.joints.values(), value):
            joint.position = joint_position
    
    @property
    def joint_velocities(self):
        return [joint.velocity for joint in self.joints]

    @property
    def mass(self):
        return self.client(self.uid)['mass']


class Joint():
    """Joint.

    Args:
       physics_client (BulletPhysics): The BulletPhysics object
       body_uid (int): Unique id of the body that the joint belongs to
       joint_ind (int): The index of the joint in the body.
    """
    def __init__(self, physics_client, body_uid, joint_ind):
        self.uid = (body_uid, joint_ind)
        self.client = physics_client

        self.body_uid = body_uid
        self.ind = joint_ind

        self.name = self.client.get_joint_name(ind)

    @property
    def limit(self):
        return self.physics.get_joint_limit(self.uid)

    @property
    def lower_limit(self):
        return self.physics.get_joint_limit(self.uid)['lower']

    @property
    def upper_limit(self):
        return self.physics.get_joint_limit(self.uid)['upper']

    @property
    def max_effort(self):
        return self.physics.get_joint_limit(self.uid)['effort']

    @property
    def max_velocity(self):
        return self.physics.get_joint_limit(self.uid)['velocity']

    @property
    def range(self):
        return self.upper_limit - self.lower_limit

    @property
    def dynamics(self):
        return self.physics.get_joint_dynamics(self.uid)

    @property
    def damping(self):
        return self.physics.get_joint_dynamics(self.uid)['damping']

    @property
    def friction(self):
        return self.physics.get_joint_dynamics(self.uid)['friction']

    @property
    def position(self):
        return self.physics.get_joint_position(self.uid)

    @property
    def velocity(self):
        return self.physics.get_joint_velocity(self.uid)

    @property
    def reaction_force(self):
        if not self._has_sensor:
            raise ValueError('Joint %s has no sensor enabled.' % (self.name))
        return self.physics.get_joint_reaction_force(self.uid)

    @position.setter
    def position(self, value):
        self.client.set_joint_position(self.uid, position=value)

class Link():
    """Link.

    Args:
       physics_client (BulletPhysics): The BulletPhysics object
       body_uid (int): Unique id of the body that the link belongs to
       link_ind (int): The index of the link in the body.
    """
    def __init__(self, physics_client, body_uid, link_ind):
        self.uid = (body_uid, link_ind)
        self.client = physics_client

        self.body_uid = body_uid
        self.ind = link_ind

        self.name = self.client.get_link_name(ind)

    @property
    def pose(self):
        return self.physics.get_link_pose(self.uid)

    @property
    def center_of_mass(self):
        return self.physics.get_link_center_of_mass(self.uid)

    @property
    def mass(self):
        if self._mass is None:
            self._mass = self.physics.get_link_mass(self.uid)
        return self._mass

    @property
    def dynamics(self):
        return self.physics.get_link_dynamics(self.uid)

    def set_dynamics(self,
                     mass=None,
                     lateral_friction=None,
                     rolling_friction=None,
                     spinning_friction=None):
        """Set dynmamics.

        Args:
            mass: The mass of the body.
            lateral_friction: The lateral friction coefficient.
            rolling_friction: The rolling friction coefficient.
            spinning_friction: The spinning friction coefficient.
        """
        return self.physics.set_link_dynamics(
            self.uid,
            mass=mass,
            lateral_friction=lateral_friction,
            rolling_friction=rolling_friction,
            spinning_friction=rolling_friction,
        )

# class Constraint

# class Camera

