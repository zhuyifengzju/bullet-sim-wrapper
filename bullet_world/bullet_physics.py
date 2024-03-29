"""The object-oriented wrapper of PyBullet."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time

import numpy as np
import pybullet
import pybullet_data
import six

from bullet_world.math_utils import Orientation
from bullet_world.math_utils import Pose
from bullet_world.logging import logger


JOINT_TYPES_MAPPING = {
    'revolute': pybullet.JOINT_REVOLUTE,
    'prismatic': pybullet.JOINT_PRISMATIC,
    'fixed': pybullet.JOINT_FIXED,
    'point2point': pybullet.JOINT_POINT2POINT
}

class BulletPhysics():
    """Physics API wrapper for Bullet."""

    def __init__(self,
                 time_step=1e-3,
                 use_visualizer=True,
                 worker_id=0):
        """
        Initialization function.

        Args:
            time_step: The time step of the simulation. Run real-time
                simulation if it is set to None.
            use_visualizer: If use the visualizer.
            worker_id: The key of the simulation client.
        """
        logger.info('pybullet API Version: %s.' % (pybullet.getAPIVersion()))
        if use_visualizer:
            self._uid = pybullet.connect(pybullet.GUI)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 1)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
            assert worker_id == 0
            logger.info('Connected client %d to GUI.', self._uid)
        else:
            logger.info('Use worker_id %d for the simulation.', worker_id)
            self._uid = pybullet.connect(pybullet.DIRECT, key=worker_id)
            logger.info('Connected client %d to DIRECT.', self._uid)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        self._time_step = time_step
        self._start_time = None
        self._num_steps = None

        self._gravity = None

        self._use_visualizer = use_visualizer

    #
    # Properties
    #
    def joint_type(self, joint_type_name):
        return JOINT_TYPES_MAPPING[joint_type_name]

    @property
    def geom_box(self):
        return pybullet.GEOM_BOX

    @property
    def geom_cylinder(self):
        return pybullet.GEOM_CYLINDER
    
    @property
    def uid(self):
        return self._uid

    @property
    def time_step(self):
        return self._time_step

    @property
    def num_steps(self):
        return self._num_steps

    @property
    def gravity(self):
        return self._gravity

    def __del__(self):
        pybullet.disconnect(physicsClientId=self.uid)
        logger.info('Disconnected client %d to pybullet server.', self._uid)

    def reset(self):
        """Reset the simulation."""
        pybullet.resetSimulation(physicsClientId=self.uid)
        self._start_time = None
        self._num_steps = None

    def start(self):
        """Start the simulation."""
        if self._time_step is None:
            pybullet.setRealTimeSimulation(1, physicsClientId=self.uid)
        else:
            pybullet.setRealTimeSimulation(0, physicsClientId=self.uid)
            pybullet.setTimeStep(self._time_step, physicsClientId=self.uid)

        self._start_time = time.time()
        self._num_steps = 0

    def step(self):
        """Step the simulation."""
        pybullet.stepSimulation(physicsClientId=self.uid)
        self._num_steps += 1

    def is_real_time(self):
        """If the simulation is real-time.

        Returns:
            True or False.
        """
        if self._time_step is None:
            return True
        else:
            return False

    def time(self):
        """Return the simulation time."""
        if self.is_real_time():
            return time.time() - self._start_time
        else:
            return self._time_step * self._num_steps

    def set_gravity(self, gravity):
        """Set the gravity.

        Args:
            gravity: The gravity as a 3-dimensional vector.
        """
        self._gravity = gravity
        pybullet.setGravity(gravity[0], gravity[1], gravity[2],
                            physicsClientId=self.uid)

    #
    # Body
    #

    def add_body(self,
                 filename,
                 pose,
                 scale=1.0,
                 is_static=False,
                 **kwargs):
        """Load a body into the simulation.

        Args:
            filename: The path to the body file. The current supported file
                formats are urdf and sdf.
            pose: The pose of the base, as an instance of robovat.math.Pose or
                a tuple of position and orientation.
            scale: The global scaling factor.
            is_static: If set the pose of the base to be fixed.
            **kwargs: extra arguments intended for loading obj file

        Returns:
            body_uid: The unique ID of the body.
        """
        filename = os.path.abspath(filename)
        assert os.path.exists(filename), 'File %s does not exist.' % filename
        _, ext = os.path.splitext(filename)

        pose = Pose(pose)
        position = list(pose.position)
        quaternion = list(pose.quaternion)

        if ext == '.urdf':
            # Do not use pybullet.URDF_USE_SELF_COLLISION since it will cause
            # problems for the motor control in Bullet.
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
            body_uid = pybullet.loadURDF(
                fileName=filename,
                basePosition=position,
                baseOrientation=quaternion,
                globalScaling=scale,
                useFixedBase=is_static,
                physicsClientId=self.uid,
                flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                )
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

        elif ext == '.obj':
            collision_kwargs = {'physicsClientId': self.uid}
            visual_kwargs = {'physicsClientId': self.uid}
            body_kwargs = {'physicsClientId': self.uid}

            if 'collisionFramePosition' in kwargs:
                collision_kwargs['collisionFramePosition'] = kwargs['collisionFramePosition']
            if 'collisionFrameOrientation' in kwargs:
                collision_kwargs['collisionFrameOrientation'] = kwargs['collisionFrameOrientation']

            collision_shape_id = pybullet.createCollisionShape(pybullet.GEOM_MESH,
                                                               fileName=filename,
                                                               meshScale=[scale] * 3,
                                                               **collision_kwargs)
            if 'visualFramePosition' in kwargs:
                visual_kwargs['visualFramePosition'] = kwargs['visualFramePosition']
            if 'visualFrameOrientation' in kwargs:
                visual_kwargs['visualFrameOrientation'] = kwargs['visualFrameOrientation']
                
            visual_shape_id = pybullet.createVisualShape(pybullet.GEOM_MESH,
                                                         fileName=filename,
                                                         meshScale=[scale] * 3,
                                                         **visual_kwargs)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

            if 'baseMass' in kwargs:
                body_kwargs['baseMass'] = kwargs['baseMass']
            else:
                # Default baseMass to be 0.1
                body_kwargs['baseMass'] = 0.1
            
            body_uid = pybullet.createMultiBody(baseCollisionShapeIndex=collision_shape_id,
                                                baseVisualShapeIndex=visual_shape_id,
                                                basePosition=position,
                                                baseOrientation=quaternion,
                                                **body_kwargs
            )
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
        
        else:
            raise ValueError('Unrecognized extension %s.' % ext)

        return int(body_uid)

    def create_collision_shape(self, *args, **kwargs):
        kwargs["physicsClientId"] = self.uid
        return pybullet.createCollisionShape(*args, **kwargs)

    def create_visual_shape(self, *args, **kwargs):
        kwargs["physicsClientId"] = self.uid
        return pybullet.createVisualShape(*args, **kwargs)

    def add_primitive_body(self, **kwargs):
        kwargs["physicsClientId"] = self.uid
        body_uid = pybullet.createMultiBody(**kwargs)
        return int(body_uid)

    def remove_body(self, body_uid):
        """Remove the body.

        Args:
            body_uid: The body Unique ID.
        """
        pybullet.removeBody(
                bodyUniqueId=body_uid, physicsClientId=self.uid)

    def get_body_pose(self, body_uid):
        """Get the pose of the body.

        The pose of the body is defined as the pose of the base of the body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            An instance of Pose.
        """
        position, quaternion = pybullet.getBasePositionAndOrientation(
                bodyUniqueId=body_uid, physicsClientId=self.uid)
        return Pose([position, quaternion])

    def get_body_position(self, body_uid):
        """Get the position of the body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A 3-dimensional float32 numpy array.
        """
        position, _ = pybullet.getBasePositionAndOrientation(
            bodyUniqueId=body_uid, physicsClientId=self.uid)
        return np.array(position, dtype=np.float32)

    def get_body_linear_velocity(self, body_uid):
        """Get the lienar velocity of the body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A 3-dimensional float32 numpy array.
        """
        linear_velocity, _ = pybullet.getBaseVelocity(
            bodyUniqueId=body_uid, physicsClientId=self.uid)
        return np.array(linear_velocity, dtype=np.float32)

    def get_body_angular_velocity(self, body_uid):
        """Get the angular velocity of the body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A 3-dimensional float32 numpy array.
        """
        _, angular_velocity = pybullet.getBaseVelocity(
            bodyUniqueId=body_uid, physicsClientId=self.uid)
        return np.array(angular_velocity, dtype=np.float32)

    def get_body_mass(self, body_uid):
        """Get the mass of the body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A 3-dimensional float32 numpy array.
        """
        mass, _, _, _, _, _, _, _, _, _ = pybullet.getDynamicsInfo(
            bodyUniqueId=body_uid, linkIndex=-1, physicsClientId=self.uid)
        return mass

    def get_body_dynamics(self, body_uid):
        """Get the dynamics of the body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A dictionary of body dynamics.
        """
        (mass, lateral_friction, _, _, _, _,
         rolling_friction, spinning_friction, _, _) = pybullet.getDynamicsInfo(
            bodyUniqueId=body_uid, linkIndex=-1, physicsClientId=self.uid)
        return {
            'mass': mass,
            'lateral_friction': lateral_friction,
            'rolling_friction': rolling_friction,
            'spinning_friction': spinning_friction,
        }

    def get_body_link_indices(self, body_uid):
        """Get the indices of the links of a body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A list of integers.
        """
        num_joints = pybullet.getNumJoints(
            bodyUniqueId=body_uid, physicsClientId=self.uid)
        link_indices = range(num_joints)
        return link_indices

    def get_body_joint_indices(self, body_uid):
        """Get the indices of the joints of a body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A list of integers.
        """
        num_joints = pybullet.getNumJoints(
            bodyUniqueId=body_uid, physicsClientId=self.uid)
        joint_indices = range(num_joints)
        return joint_indices

    def set_body_pose(self, body_uid, pose):
        """Set the pose of the body.

        Args:
            body_uid: The body Unique ID.
            pose: An instance of Pose.
        """
        pose = Pose(pose)
        position = list(pose.position)
        quaternion = list(pose.quaternion)
        pybullet.resetBasePositionAndOrientation(
            bodyUniqueId=body_uid, posObj=position, ornObj=quaternion,
            physicsClientId=self.uid)

    def set_body_position(self, body_uid, position):
        """Set the position of the body.

        Args:
            body_uid: The body Unique ID.
            position: A 3-dimensional float32 numpy array or a list of 3
                float32 values.
        """
        position = list(position)
        _, quaternion = pybullet.getBasePositionAndOrientation(body_uid)
        pybullet.resetBasePositionAndOrientation(
            bodyUniqueId=body_uid, posObj=position, ornObj=quaternion,
            physicsClientId=self.uid)

    def set_body_orientation(self, body_uid, orientation):
        """Set the orientation of the body.

        Args:
            body_uid: The body Unique ID.
            orientation: An instance of Orientation.
        """
        position, _ = pybullet.getBasePositionAndOrientation(body_uid)
        quaternion = list(orientation.quaternion)
        pybullet.resetBasePositionAndOrientation(
            bodyUniqueId=body_uid, posObj=position, ornObj=quaternion,
            physicsClientId=self.uid)

    def set_body_linear_velocity(self, body_uid, linear_velocity):
        """Set the linear velocity of the body.

        Args:
            body_uid: The body Unique ID.
            linear_velocity: A 3-dimensional float32 numpy array or a list of 3
                float32 values.
        """
        linear_velocity = list(linear_velocity)
        pybullet.resetBaseVelocity(
            bodyUniqueId=body_uid, linearVelocity=linear_velocity,
            physicsClientId=self.uid)

    def set_body_angular_velocity(self, body_uid, angular_velocity):
        """Set the angular velocity of the body.

        Args:
            body_uid: The body Unique ID.
            angular_velocity: A 3-dimensional float32 numpy array or a list of
                3 float32 values.
        """
        angular_velocity = list(angular_velocity)
        pybullet.resetBaseVelocity(
            bodyUniqueId=body_uid, angularVelocity=angular_velocity,
            physicsClientId=self.uid)

    def set_body_mass(self, body_uid, mass):
        """Set the mass of the body.

        Args:
            body_uid: The body Unique ID.
            mass: A float32 value.
        """
        pybullet.changeDynamics(
            bodyUniqueId=body_uid, linkIndex=-1, mass=mass,
            physicsClientId=self.uid)

    def set_body_dynamics(self,
                          body_uid,
                          mass=None,
                          lateral_friction=None,
                          rolling_friction=None,
                          spinning_friction=None,
                          contact_damping=None,
                          contact_stiffness=None):
        """Set the dynamics of the body.

        Args:
            body_uid: The body Unique ID.
            mass (float, optional): mass of the body
            lateral_friction (float, optional): lateral friction coefficient
            rolling_friction (float, optional): rolling friction coefficient
            spinning_friction (float, optional): spinning friction coefficient
            contact_damping (float, optional): damping cofficient for contact
            contact_stiffness (float, optional): stiffness coefficient for contact
        """
        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['linkIndex'] = -1

        if mass is not None:
            kwargs['mass'] = mass

        if lateral_friction is not None:
            kwargs['lateralFriction'] = lateral_friction

        if rolling_friction is not None:
            kwargs['rollingFriction'] = rolling_friction

        if spinning_friction is not None:
            kwargs['spinningFriction'] = spinning_friction

        if contact_damping is not None:
            kwargs['contactDamping'] = contact_damping

        if contact_stiffness is not None:
            kwargs['contactStiffness'] = contact_stiffness

        pybullet.changeDynamics(**kwargs)

    def set_body_color(self, body_uid, rgba, specular):
        """Set the mass of the body.

        Args:
            body_uid: The body Unique ID.
            rgba: A 4-dimensional float32 vector.
            specular: A 4-dimensional float32 vector.
        """
        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['objectUniqueId'] = body_uid
        kwargs['linkIndex'] = -1

        if rgba is not None:
            kwargs['rgbaColor'] = rgba

        if specular is not None:
            kwargs['specularColor'] = specular

        pybullet.changeVisualShape(**kwargs)

    def get_num_joints(self, body_uid):
        return pybullet.getNumJoints(bodyUniqueId=body_uid,
                                     physicsClientId=self.uid)
    #
    # Link
    #

    def get_link_name(self, link_uid):
        """Get the name of the link.

        Args:
            link_uid: A tuple of the body Unique ID and the link index.

        Returns:
            The name of the link.
        """
        body_uid, link_ind = link_uid
        _, _, _, _, _, _, _, _, _, _, _, _, link_name, _, _, _, _ = (
            pybullet.getJointInfo(bodyUniqueId=body_uid,
                                  jointIndex=link_ind,
                                  physicsClientId=self.uid))
        if isinstance(link_name, bytes):
            link_name = link_name.decode('utf-8')
        
        return link_name

    def get_link_pose(self, link_uid):
        """Get the pose of the link.

        Args:
            link_uid: A tuple of the body Unique ID and the link index.

        Returns:
            An instance of Pose.
        """
        body_uid, link_ind = link_uid
        _, _, _, _, position, quaternion = pybullet.getLinkState(
            bodyUniqueId=body_uid, linkIndex=link_ind,
            physicsClientId=self.uid)
        return Pose([position, quaternion])

    def get_link_local_offset(self, link_uid):
        """Get the local offset of the link.

        Args:
            link_uid: A tuple of the body Unique ID and the link index.

        Returns:
            An instance of Pose.
        """
        body_uid, link_ind = link_uid
        _, _, position, quaternion, _, _ = pybullet.getLinkState(
            bodyUniqueId=body_uid, linkIndex=link_ind,
            physicsClientId=self.uid)
        return Pose([position, quaternion])
    
    def get_link_center_of_mass(self, link_uid):
        """Get the center of mass of the link.

        Args:
            link_uid: A tuple of the body Unique ID and the link index.

        Returns:
            An instance of Pose.
        """
        body_uid, link_ind = link_uid
        position, quaternion, _, _, _, _ = pybullet.getLinkState(
            bodyUniqueId=body_uid, linkIndex=link_ind,
            physicsClientId=self.uid)
        return Pose([position, quaternion])

    def get_link_mass(self, link_uid):
        """Get the mass of the link.

        Args:
            link_uid: A tuple of the body Unique ID and the link index.

        Returns:
            A float32 value.
        """
        raise NotImplementedError('This is still buggy in PyBullet.')
        body_uid, link_ind = link_uid
        mass, _, _, _, _, _, _, _, _, _ = pybullet.getDynamicsInfo(
            bodyUniqueId=body_uid, linkIndex=link_ind,
            physicsClientId=self.uid)
        return mass

    def get_link_dynamics(self, link_uid):
        """Get the dynamics of the body.

        Args:
            body_uid: The body Unique ID.

        Returns:
            A dictionary of body dynamics.
        """
        body_uid, link_ind = link_uid
        (mass, lateral_friction, _, _, _, _,
         rolling_friction, spinning_friction, _, _) = pybullet.getDynamicsInfo(
            bodyUniqueId=body_uid, linkIndex=link_ind,
            physicsClientId=self.uid)
        return {
            'mass': mass,
            'lateral_friction': lateral_friction,
            'rolling_friction': rolling_friction,
            'spinning_friction': spinning_friction,
        }

    def set_link_mass(self, link_uid, mass):
        """Set the mass of the link.

        Args:
            link_uid: A tuple of the body Unique ID and the link index.
            mass: A float32 value.
        """
        raise NotImplementedError('This is still buggy in PyBullet.')
        body_uid, link_ind = link_uid
        pybullet.changeDynamics(
            bodyUniqueId=body_uid, linkIndex=link_ind, mass=mass,
            physicsClientId=self.uid)

    def set_link_dynamics(self,
                          link_uid,
                          mass=None,
                          lateral_friction=None,
                          rolling_friction=None,
                          spinning_friction=None,
                          ):
        """Set the dynamics of the link.

        Args:
            link_uid: A tuple of the body Unique ID and the link index.
        """
        body_uid, link_ind = link_uid

        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['linkIndex'] = link_ind

        if mass is not None:
            kwargs['mass'] = mass

        if lateral_friction is not None:
            kwargs['lateralFriction'] = lateral_friction

        if rolling_friction is not None:
            kwargs['rollingFriction'] = rolling_friction

        if spinning_friction is not None:
            kwargs['spinningFriction'] = spinning_friction

        pybullet.changeDynamics(**kwargs)

    #
    # Joint
    #

    def get_joint_name(self, joint_uid):
        """Get the name of the joint.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.

        Returns:
            The name of the joint.
        """
        body_uid, joint_ind = joint_uid
        _, joint_name, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _ = (
            pybullet.getJointInfo(bodyUniqueId=body_uid,
                                  jointIndex=joint_ind,
                                  physicsClientId=self.uid))
        if isinstance(joint_name, bytes):
            joint_name = joint_name.decode('utf-8')
        
        return joint_name

    def get_joint_dynamics(self, joint_uid):
        """Get the dynamics of the joint.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.

        Returns:
            dynamics: A dictionary of dampling and friction.
        """
        body_uid, joint_ind = joint_uid
        _, _, _, _, _, _, damping, friction, _, _, _, _, _, _, _, _ = (
            pybullet.getJointInfo(bodyUniqueId=body_uid,
                                  jointIndex=joint_ind,
                                  physicsClientId=self.uid))
        dynamics = {
            'damping': damping,
            'friction': friction
        }
        return dynamics

    def get_joint_limit(self, joint_uid):
        """Get the limit of the joint.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.

        Returns:
            limit: A dictionary of lower, upper, effort and velocity.
        """
        body_uid, joint_ind = joint_uid
        (_, _, _, _, _, _, _, _, lower, upper, max_force, max_vel, _,
         _, _, _, _) = pybullet.getJointInfo(bodyUniqueId=body_uid,
                                             jointIndex=joint_ind,
                                             physicsClientId=self.uid)
        limit = {
            'lower': lower,
            'upper': upper,
            'effort': max_force,
            'velocity': max_vel
        }
        return limit

    def get_joint_position(self, joint_uid):
        """Get the joint position of the joint.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.

        Returns:
            A float32 value.
        """
        body_uid, joint_ind = joint_uid
        position, _, _, _ = pybullet.getJointState(
            bodyUniqueId=body_uid, jointIndex=joint_ind,
            physicsClientId=self.uid)
        return position

    def get_joint_velocity(self, joint_uid):
        """Get the joint velocity of the joint.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.

        Returns:
            A float32 value.
        """
        body_uid, joint_ind = joint_uid
        _, vel, _, _ = pybullet.getJointState(
            bodyUniqueId=body_uid, jointIndex=joint_ind,
            physicsClientId=self.uid)
        return vel

    def get_joint_reaction_force(self, joint_uid):
        """Get the reaction force of the joint.

        These are the joint reaction forces, if a torque sensor is enabled for
        this joint it is [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it is
        [0, 0, 0, 0, 0, 0].

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.

        Returns:
            A 3-dimensional float32 numpy array.
        """
        body_uid, joint_ind = joint_uid
        _, _, reaction_force, _ = pybullet.getJointState(
            bodyUniqueId=body_uid, jointIndex=joint_ind,
            physicsClientId=self.uid)
        return np.array(reaction_force, dtype=np.float32)

    def get_joint_torque(self, joint_uid):
        """Get the torque force of the joint.

        This is the motor torque applied during the last stepSimulation().

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.

        Returns:
            A 3-dimensional float32 numpy array.
        """
        body_uid, joint_ind = joint_uid
        _, _, _, torque = pybullet.getJointState(
            bodyUniqueId=body_uid, jointIndex=joint_ind,
            physicsClientId=self.uid)
        return np.array(torque, dtype=np.float32)

    def set_joint_position(self, joint_uid, position):
        """Set the position of the joint.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.
            position: A float32 value.
        """
        body_uid, joint_ind = joint_uid
        pybullet.resetJointState(
            bodyUniqueId=body_uid, jointIndex=joint_ind,
            targetValue=position, physicsClientId=self.uid)

    def set_joint_velocity(self, joint_uid, velocity):
        """Set the position of the joint.

        The joint position will be set to the current position.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.
            vel: A float32 value.
        """
        body_uid, joint_ind = joint_uid
        position, _, _, _ = pybullet.getJointState(
            bodyUniqueId=body_uid, jointIndex=joint_ind,
            physicsClientId=self.uid)
        pybullet.resetJointState(
            bodyUniqueId=body_uid, jointIndex=joint_ind,
            targetValue=position,
            targetVelocity=velocity, physicsClientId=self.uid)

    def enable_joint_sensor(self, joint_uid):
        """Enable joint force torque sensor.

        Args:
            joint_uid: A tuple of the body Unique ID and the joint index.
        """
        body_uid, joint_ind = joint_uid
        pybullet.enableJointForceTorqueSensor(
            bodyUniqueId=body_uid,
            jointIndex=joint_ind,
            enableSensor=1,
            physicsClientId=self.uid)

    #
    # Constraint
    #

    def add_constraint(self,
                       parent_uid,
                       child_uid,
                       joint_type='fixed',
                       joint_axis=[0, 0, 0],
                       parent_frame_pose=None,
                       child_frame_pose=None):
        """Add the constraint.

        A constraint limits the relative movements between two entities using a
        joint. Different types of joints have different degrees of freedom.

        Args:
            parent: The unique ID of the parent entity.
            child: The unique ID of the child entity.
            joint_type: The type of the joint.
            joint_axis: The axis of the joint.
            parent_frame_pose: The pose of the joint in the parent frame.
            child_frame_pose: The pose of the joint in the child frame.

        Returns:
            The constraint unique ID.
        """
        if isinstance(parent_uid, six.integer_types):
            # The parent entity is a body.
            parent_body_uid = parent_uid
            parent_link_ind = -1
        elif isinstance(parent_uid, (tuple, list)):
            # The parent entity is a link.
            parent_body_uid, parent_link_ind = parent_uid
        else:
            raise ValueError

        if isinstance(child_uid, six.integer_types):
            # The child entity is a body.
            child_body_uid = child_uid
            child_link_ind = -1
        elif isinstance(child_uid, (tuple, list)):
            # The child entity is a link.
            child_body_uid, child_link_ind = child_uid
        elif child_uid is None:
            # The child entity is ignored.
            child_body_uid = -1
            child_link_ind = -1
        else:
            raise ValueError

        if parent_frame_pose is None:
            parent_frame_position = [0, 0, 0]
            parent_frame_quaternion = None
        else:
            if not isinstance(parent_frame_pose, Pose):
                parent_frame_pose = Pose(parent_frame_pose)
            parent_frame_position = parent_frame_pose.position
            parent_frame_quaternion = parent_frame_pose.quaternion

        if child_frame_pose is None:
            child_frame_position = [0, 0, 0]
            child_frame_quaternion = None
        else:
            if not isinstance(child_frame_pose, Pose):
                child_frame_pose = Pose(child_frame_pose)
            child_frame_position = child_frame_pose.position
            child_frame_quaternion = child_frame_pose.quaternion

        joint_type = JOINT_TYPES_MAPPING[joint_type]
        joint_axis = list(joint_axis)

        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['parentBodyUniqueId'] = parent_body_uid
        kwargs['parentLinkIndex'] = parent_link_ind
        kwargs['childBodyUniqueId'] = child_body_uid
        kwargs['childLinkIndex'] = child_link_ind
        kwargs['jointType'] = joint_type
        kwargs['jointAxis'] = joint_axis
        kwargs['parentFramePosition'] = parent_frame_position
        kwargs['childFramePosition'] = child_frame_position
        if parent_frame_quaternion is not None:
            kwargs['parentFrameOrientation'] = parent_frame_quaternion
        if child_frame_quaternion is not None:
            kwargs['childFrameOrientation'] = child_frame_quaternion

        constraint_uid = pybullet.createConstraint(**kwargs)

        return constraint_uid

    def change_constraint(self, constraint_uid, **kwargs):
        """Change a constraint parameter
        
        Args:
           constraint_uid: The constraint unique ID
           kwargs: arguments for changig constraints
        """
        pybullet.changeConstraint(constraint_uid, **kwargs)
    
    def remove_constraint(self, constraint_uid):
        """Remove a constraint.

        Args:
            constraint_uid: The constraint unique ID.
        """
        # TODO(kuanfang): removeConstraint dose not work.
        pybullet.changeConstraint(constraint_uid, maxForce=0,
                                  physicsClientId=self.uid)
        pybullet.removeConstraint(constraint_uid, physicsClientId=self.uid)

    def get_constraint_pose(self, constraint_uid):
        """Get the constraint pose.

        Args:
            constraint_uid: The constraint unique ID.

        Returns:
            An instance of Pose.
        """
        _, _, _, _, _, _, _, position, _, quaternion, _, _, _, _, _ = (
            pybullet.getConstraintInfo(constraintUniqueId=constraint_uid,
                                       physicsClientId=self.uid))
        return Pose([position, quaternion])

    def get_constraint_position(self, constraint_uid):
        """Get the constraint position.

        Args:
            constraint_uid: The constraint unique ID.

        Returns:
            A 3-dimenstional float32 numpy array.
        """
        _, _, _, _, _, _, _, position, _, _, _, _, _, _, _ = (
            pybullet.getConstraintInfo(
                constraintUniqueId=constraint_uid, physicsClientId=self.uid))
        return np.array(position, dtype=np.float32)

    def get_constraint_orientation(self, constraint_uid):
        """Get the constraint orientation.

        Args:
            constraint_uid: The constraint unique ID.

        Returns:
            An instance of Orientation.
        """
        _, _, _, _, _, _, _, _, _, quaternion, _, _, _, _, _ = (
            pybullet.getConstraintInfo(
                constraintUniqueId=constraint_uid, physicsClientId=self.uid))
        return quaternion

    def get_constraint_max_force(self, constraint_uid):
        """Get the maximal force of the constraint.

        Args:
            constraint_uid: The constraint unique ID.

        Returns:
            A 3-dimensional float32 numpy array.
        """
        _, _, _, _, _, _, _, _, _, _, max_force, _, _, _, _ = (
            pybullet.getConstraintInfo(
                constraintUniqueId=constraint_uid, physicsClientId=self.uid))
        return np.array(max_force, dtype=np.float32)

    def set_constraint_pose(self, constraint_uid, pose):
        """Set the constraint pose.

        Args:
            constraint_uid: The constraint unique ID.
            pose: An instance of Pose.
        """
        if not isinstance(pose, Pose):
            pose = Pose(pose)

        position = list(pose.position)
        quaternion = list(pose.quaternion)

        pybullet.changeConstraint(
            userConstraintUniqueId=constraint_uid,
            jointChildPivot=position,
            jointChildFrameOrientation=quaternion,
            physicsClientId=self.uid)

    def set_constraint_position(self, constraint_uid, position):
        """Set the constraint position.

        Args:
            constraint_uid: The constraint unique ID.
            position: A 3-dimensional float32 numpy array.
        """
        position = list(position)
        pybullet.changeConstraint(
            userConstraintUniqueId=constraint_uid,
            jointChildPivot=position,
            physicsClientId=self.uid)

    def set_constraint_orientation(self, constraint_uid, orientation):
        """Set the constraint orientation.

        Args:
            constraint_uid: The constraint unique ID.
            orientation: An instance of Orientation.
        """
        quaternion = list(orientation.quaternion)
        pybullet.changeConstraint(
            userConstraintUniqueId=constraint_uid,
            jointChildFrameOrientation=quaternion,
            physicsClientId=self.uid)

    def set_constraint_max_force(self, constraint_uid, max_force):
        """Set the maximal force of the constraint.

        Args:
            constraint_uid: The constraint unique ID.
            max_force: A 3-dimensional float32 numpy array.
        """
        pybullet.changeConstraint(
            userConstraintUniqueId=constraint_uid,
            maxForce=max_force,
            physicsClientId=self.uid)

    def get_num_constraints(self):
        """Get number of constraints present in the env.

        Return:
           num_constraints (int)
        """
        return pybullet.getNumConstraints(physicsClientId=self.uid)
        
    #
    # Motor Control
    #

    def position_control(self,
                         joint_uid,
                         target_position,
                         target_velocity=None,
                         max_velocity=None,
                         max_force=None,
                         position_gain=None,
                         velocity_gain=None):
        """Position control of a joint.

        Args:
            joint_uid: The tuple of the body unique ID and the joint index.
            target_position: The target joint position.
            target_velocity: The target joint velocity.
            max_velocity: The maximal joint velocity.
            max_force: The maximal joint force.
            position_gain: The position gain.
            velocity_gain: The velocity gain.
        """
        body_uid, joint_ind = joint_uid

        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['jointIndex'] = joint_ind
        kwargs['controlMode'] = pybullet.POSITION_CONTROL
        kwargs['targetPosition'] = target_position

        if target_velocity is not None:
            kwargs['targetVelocity'] = target_velocity

        if max_velocity is not None:
            kwargs['maxVelocity'] = max_velocity

        if max_force is not None:
            kwargs['force'] = max_force

        if position_gain is not None:
            kwargs['positionGain'] = position_gain

        if velocity_gain is not None:
            kwargs['velocityGain'] = velocity_gain

        pybullet.setJointMotorControl2(**kwargs)

    def velocity_control(self,
                         joint_uid,
                         target_velocity,
                         max_force=None,
                         position_gain=None,
                         velocity_gain=None):
        """Velocity control of a joint.

        Args:
            joint_uid: The tuple of the body unique ID and the joint index.
            target_velocity: The joint velocity.
            max_joint_force: The maximal force of the joint.
            position_gain: The position gain.
            velocity_gain: The velocity gain.
        """
        body_uid, joint_ind = joint_uid

        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['jointIndex'] = joint_ind
        kwargs['controlMode'] = pybullet.VELOCITY_CONTROL
        kwargs['targetVelocity'] = target_velocity

        if max_force is not None:
            kwargs['force'] = max_force

        if position_gain is not None:
            kwargs['positionGain'] = position_gain

        if velocity_gain is not None:
            kwargs['velocityGain'] = velocity_gain

        pybullet.setJointMotorControl2(**kwargs)

    def torque_control(self, joint_uid, target_torque):
        """Torque control of a joint.

        Args:
            joint_uid: The tuple of the body unique ID and the joint index.
            joint_torque: The torque of the joint.
        """
        body_uid, joint_ind = joint_uid

        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['jointIndex'] = joint_ind
        kwargs['controlMode'] = pybullet.TORQUE_CONTROL
        kwargs['force'] = target_torque

        pybullet.setJointMotorControl2(**kwargs)

    def position_control_array(self,
                               body_uid,
                               joint_inds,
                               target_positions,
                               target_velocities=None,
                               max_velocities=None,
                               max_forces=None,
                               position_gains=None,
                               velocity_gains=None):
        """Position control of a list of joints of a body.

        Args:
            body_uid: The body unique ID.
            joint_inds: The list of joint indices.
            target_positions: The list of target joint positions.
            target_velocities: The list of of target joint velocities.
            max_velocities: The list of maximal joint velocities.
            max_forces: The list of maximal joint forces.
            position_gains: The list of position gains.
            velocity_gains: The list of velocity gains.
        """
        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['jointIndices'] = joint_inds
        kwargs['controlMode'] = pybullet.POSITION_CONTROL
        kwargs['targetPositions'] = target_positions

        if target_velocities is not None:
            kwargs['targetVelocities'] = target_velocities

        if max_velocities is not None:
            raise NotImplementedError('This is not implemented in pybullet.')

        if max_forces is not None:
            kwargs['forces'] = max_forces

        if position_gains is not None:
            kwargs['positionGains'] = position_gains

        if velocity_gains is not None:
            kwargs['velocityGains'] = velocity_gains

        pybullet.setJointMotorControlArray(**kwargs)

    def velocity_control_array(self,
                               body_uid,
                               joint_inds,
                               joint_velocities,
                               max_joint_forces=None,
                               position_gains=None,
                               velocity_gains=None):
        """Velocity control of a list of joints of a body.

        Args:
            body_uid: The body unique ID.
            joint_inds: The list of joint indices.
            joint_velocities: The list of joint velocities for each specified
                joint.
            max_joint_forces: The list of maximal forces, set to None to
                ignore.
            position_gains: The list of position gains, set to None to ignore.
            velocity_gains: The list of position gains, set to None to ignore.
        """
        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['jointIndices'] = joint_inds
        kwargs['controlMode'] = pybullet.VELOCITY_CONTROL
        kwargs['targetVelocities'] = joint_velocities
        if max_joint_forces is not None:
            kwargs['forces'] = max_joint_forces
        if position_gains is not None:
            kwargs['positionGains'] = position_gains
        if velocity_gains is not None:
            kwargs['velocityGains'] = velocity_gains

        pybullet.setJointMotorControlArray(**kwargs)

    def torque_control_array(self, body_uid, joint_inds, joint_torques):
        """Torque control of a list of joints of a body.

        Args:
            body_uid: The body unique ID.
            joint_inds: The list of joint indices.
            joint_torques: The list of torques for each specified joint.
        """
        kwargs = dict()
        kwargs['physicsClientId'] = self.uid
        kwargs['bodyUniqueId'] = body_uid
        kwargs['jointIndices'] = joint_inds
        kwargs['controlMode'] = pybullet.VELOCITY_CONTROL
        kwargs['forces'] = joint_torques

        pybullet.setJointMotorControlArray(**kwargs)

    #
    # Apply external disturbances
    #

    def apply_force_to_body(self, uid, force, position):
        pybullet.applyExternalForce(
            objectUniqueId=uid,
            linkIndex=-1,
            forceObj=list(force),
            posObj=list(position),
            flags=pybullet.LINK_FRAME,
            physicsClientId=self.uid)

    def apply_torque_to_body(self, uid, force, position):
        pybullet.applyExternalTorque(
            objectUniqueId=uid,
            linkIndex=-1,
            forceObj=list(force),
            posObj=list(position),
            flags=pybullet.LINK_FRAME,
            physicsClientId=self.uid)

    def apply_force_to_link(self, uid, force, position):
        body_uid, link_ind = uid
        pybullet.applyExternalForce(
            objectUniqueId=body_uid,
            linkIndex=link_ind,
            forceObj=list(force),
            posObj=list(position),
            flags=pybullet.LINK_FRAME,
            physicsClientId=self.uid)

    def apply_torque_to_link(self, uid, force, position):
        body_uid, link_ind = uid
        pybullet.applyExternalTorque(
            objectUniqueId=body_uid,
            linkIndex=link_ind,
            forceObj=list(force),
            posObj=list(position),
            flags=pybullet.LINK_FRAME,
            physicsClientId=self.uid)

    #
    # Inverse Kinematics
    #

    def compute_inverse_kinematics(self,
                                   link_uid,
                                   link_pose,
                                   upper_limits=None,
                                   lower_limits=None,
                                   ranges=None,
                                   damping=None,
                                   neutral_positions=None):
        """Compute the inverse kinematics.

        Args:
            link_uid: The unique ID of the link.
            link_pose: The target pose of the link.
            upper_limits: The upper limits of joints.
            lower_limits: The lower limits of joints.
            ranges: The ranges of joints.
            dampings: The list of joint damping parameters.
            neutral_positions: The neutral joint positions.

        returns:
            target_positions: The list of target joint positions.
        """
        body_uid, link_ind = link_uid

        if not isinstance(link_pose, Pose):
            link_pose = Pose(link_pose)

        position = link_pose.position
        quaternion = link_pose.quaternion

        kwargs = dict()

        kwargs['bodyUniqueId'] = body_uid
        kwargs['endEffectorLinkIndex'] = link_ind
        kwargs['targetPosition'] = list(position)

        if quaternion is not None:
            kwargs['targetOrientation'] = list(quaternion)

        # TODO(kuanfang): Not sure if those are necessary for computing IK.
        if lower_limits is not None:
            kwargs['lowerLimits'] = lower_limits

        if upper_limits is not None:
            kwargs['upperLimits'] = upper_limits

        if ranges is not None:
            kwargs['jointRanges'] = ranges

        if damping is not None:
            kwargs['jointDamping'] = damping

        if neutral_positions is not None:
            kwargs['restPoses'] = neutral_positions

        kwargs['physicsClientId'] = self.uid

        target_positions = pybullet.calculateInverseKinematics(**kwargs)

        return target_positions

    #
    # Dynamics
    # 

    def calculate_inverse_dynamics(self, body_uid, joint_positions, joint_velocities, joint_accelerations):
        kwargs = dict()
        kwargs['bodyUniqueId'] = body_uid
        kwargs['joint_positions'] = joint_positions
        kwargs['joint_velocities'] = joint_accelerations
        kwargs['physicsClientId'] = self.uid
        joint_forces = pybullet.calculateInverseDynamics(**kwargs)
        return joint_forces

    def calculate_mass_matrix(self, body_uid, joint_positions):
        return pybullet.calculateMassMatrix(bodyUniqueId=body_uid,
                                            objPositions=joint_positions,
                                            physicsClientId=self.uid)

    def calculate_jacobian(self, link_uid, com_positions, joint_positions, joint_velocities, joint_accelerations):
        body_uid, link_ind = link_uid
        import pdb; pdb.set_trace()
        return pybullet.calculateJacobian(bodyUniqueId=body_uid,
                                         linkIndex=link_ind,
                                         localPosition=com_positions,
                                         objPositions=joint_positions,
                                         objVelocities=joint_velocities,
                                         objAccelerations=joint_accelerations,
                                         physicsClientId=self.uid)
    
    #
    # Contacts
    # Pybullet function getContactPoints return:
    # 
    # 0: contactFlag
    # 1: bodyUniqueIdA
    # 2: bodyUniqueIdB
    # 3: linkIndexA
    # 4: linkIndexB
    # 5: positionOnA
    # 6: positionOnB
    # 7: contactNormalOnB
    # 8: contactDistance
    # 9: normalForce
    # 10: lateralFriction1
    # 11: lateralFrictionDir1
    # 12: lateralFriction2
    # 13: lateralFrictionDir2
    #

    def get_contact_points(self, a_uid, b_uid=None):
        """Check if two entities have contacts.

        Args:
            a_uid: The Unique ID of the fist entity.
            b_uid: The Unique ID of the second entity.

        Returns:
            A list of contact points.
        """
        kwargs = dict()

        if isinstance(a_uid, six.integer_types):
            kwargs['bodyA'] = a_uid
        elif isinstance(a_uid, (tuple, list)):
            kwargs['bodyA'] = a_uid[0]
            kwargs['linkIndexA'] = a_uid[1]
        else:
            raise ValueError

        if b_uid is None:
            pass
        elif isinstance(b_uid, six.integer_types):
            kwargs['bodyB'] = b_uid
        elif isinstance(b_uid, (tuple, list)):
            kwargs['bodyB'] = b_uid[0]
            kwargs['linkIndexB'] = b_uid[1]
        else:
            raise ValueError

        kwargs['physicsClientId'] = self.uid

        contact_points = pybullet.getContactPoints(**kwargs)

        return contact_points

    def get_contact_normal_force(self, a_uid, b_uid=None):
        """get contact normal force

        Args:
            a_uid: The Unique ID of the fist entity.
            b_uid: The Unique ID of the second entity.

        Returns:
            A list of contact force on the contact points
        """
        kwargs = dict()

        if isinstance(a_uid, six.integer_types):
            kwargs['bodyA'] = a_uid
        elif isinstance(a_uid, (tuple, list)):
            kwargs['bodyA'] = a_uid[0]
            kwargs['linkIndexA'] = a_uid[1]
        else:
            raise ValueError

        if b_uid is None:
            pass
        elif isinstance(b_uid, six.integer_types):
            kwargs['bodyB'] = b_uid
        elif isinstance(b_uid, (tuple, list)):
            kwargs['bodyB'] = b_uid[0]
            kwargs['linkIndexB'] = b_uid[1]
        else:
            raise ValueError

        kwargs['physicsClientId'] = self.uid

        contact_points = pybullet.getContactPoints(**kwargs)

        contact_points = [cp[9] for cp in contact_points]

        return contact_points

    def get_contact_body(self, a_uid):
        """get contact body with object of inquiry

        Args:
            a_uid: The Unique ID of the fist entity.

        Returns:
            A list of body uids of contacting with body A
        """
        kwargs = dict()

        if isinstance(a_uid, six.integer_types):
            kwargs['bodyA'] = a_uid
        elif isinstance(a_uid, (tuple, list)):
            kwargs['bodyA'] = a_uid[0]
            kwargs['linkIndexA'] = a_uid[1]
        else:
            raise ValueError

        kwargs['physicsClientId'] = self.uid

        contact_points = pybullet.getContactPoints(**kwargs)

        contact_points = [cp[2] for cp in contact_points]

        return contact_points


    #
    # Debug Visualizer
    #

    def get_debug_visualizer_info(self, info_args):
        """Return a dictionary of info with keys from info_args

        Args:
           info_args: A list of strings for debug visualizer information
        """
        width, height, view_matrix, projection_matrix, camera_up, camera_forward, horizontal, vertical, yaw, pitch, dist, target = pybullet.getDebugVisualizerCamera(self.uid)

        info = {'width': width,
                'height': height,
                'viewMatrix': view_matrix,
                'projectionMatrix': projection_matrix,
                'cameraUp': camera_up,
                'cameraForward': camera_forward,
                'horizontal': horizontal,
                'vertical': vertical,
                'yaw': yaw,
                'pitch': pitch,
                'dist': dist,
                'target': target
        }
        
        visualizer_info = {}
        for arg in info_args:
            visualizer_info[arg] = info[arg]
        return visualizer_info
    
    def reset_debug_visualizer(self, camera_distance, camera_yaw, camera_pitch, camera_target_position):
        """

        Args:
           camera_distance (float): distance from eye to camera target position
           camera_yaw (float): camera yaw angle (in degrees) left/right
           camera_pitch (float): camera pitch angle (in degrees) up/down
           camera_target_position (list or tuple of 3 floats): the camera focus point
        """
        pybullet.resetDebugVisualizerCamera(camera_distance,
                                            camera_yaw,
                                            camera_pitch,
                                            camera_target_position,
                                            self.uid)

    #
    # Save state management
    #

    def save_state(self, save_full_state=True):
        """

        Return:
          state_id (int): id for saved state
          save_full_state (bool, optional): Save the full state or not
        """
        if save_full_state:
            state_id = pybullet.saveState(self.uid)
        else:
            state_id = pybullet.saveWorld(self.uid)

    def restore_state_from_id(self, state_id):
        """
        
        Args:
           state_id (int): saved state id to restore
        """
        pybullet.restore(stateId=state_id, clientServerId=self.uid)

    def remove_state(self, state_id):
        """
        
        Args:
           state_id (int): saved state id to remove
        """
        pybullet.removeState(state_id, self.uid)

    # Other functions
    def get_visual_shape_data(self, object_id):
        return pybullet.getVisualShapeData(object_id, self.uid)


    @property
    def geom_mesh(self):
        return pybullet.GEOM_MESH

    #
    # Camera Image
    #

    def compute_view_matrix(self, camera_eye_position, camera_target_position, camera_up_vector):
        """

        Return:
          view_matrix (np.array): 4x4 matrix
        """
        view_matrix = pybullet.computeViewMatrix(cameraEyePosition=camera_eye_position,
                                                 cameraTargetPosition=camera_target_position,
                                                 cameraUpVector=camera_up_vector,
                                                 physicsClientId=self.uid)
        view_matrix = np.array(view_matrix).reshape(4, 4)
        return view_matrix

    def compute_view_matrix(self, camera_target_position, distance, yaw, pitch, roll, up_axis_index):
        raise NotImplementedError

    def compute_projection_matrix_fov(self, fov, aspect, near_val, far_val):
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov=fov,
                                                                aspect=aspect,
                                                                nearVal=near_val,
                                                                farVal=far_val,
                                                                physicsClientId=self.uid)
        projection_matrix = np.array(projection_matrix).reshape(4, 4)
        return projection_matrix
    
    def get_camera_image(self, width, height, mode="rgb", view_matrix=None, projection_matrix=None):
        kwargs = {"width": width,
                  "height": height,
                  "physicsClientId": self.uid}

        if view_matrix is not None:
            kwargs["viewMatrix"] = view_matrix
        if projection_matrix is not None:
            kwargs["projectionMatrix"] = projection_matrix

        if not self._use_visualizer:
            kwargs["renderer"] = pybullet.ER_TINY_RENDERER
            
        _, _, rgb_pixels, depth_pixels, seg_mask = pybullet.getCameraImage(**kwargs)
        if mode == "rgb":
            return rgb_pixels
        elif mode == "depth":
            return depth_pixels
        elif mode == 'rgbd':
            return np.concatenate((rgb_pixels, depth_pixels), axis=-1)
        else:
            raise NotImplementedError
