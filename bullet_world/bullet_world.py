from bullet_world import BulletPhysics, Body
from bullet_world.math_utils import Pose
from bullet_world.interfaces import JointInterface, LinkInterface, DynamicsParams
from bullet_world.robot_arm import RobotArm
from easydict import EasyDict
import os
import abc

try:
    import nvisii as v
    USE_VISII = True
except:
    USE_VISII = False

class BulletWorld():
    """
    Args:

       debug_camera_config (dict): 

    """
    def __init__(self,
                 assets_dir=os.path.join(os.path.dirname(__file__), "assets/"),
                 default_init=False,
                 time_step=1e-3,
                 use_visualizer=True,
                 worker_id=0,
                 debug_camera_config=None
                 ):
        self._physics = BulletPhysics(time_step=time_step,
                                      use_visualizer=use_visualizer,
                                      worker_id=worker_id)

        self.interfaces = EasyDict()
        joint_interface = JointInterface(self)
        link_interface = LinkInterface(self)
        dynamics_params = DynamicsParams(self, joint_interface, link_interface)
        self.interfaces.joints = joint_interface
        self.interfaces.links = link_interface
        self.interfaces.dynamics = dynamics_params

        if debug_camera_config is None:
            debug_camera_config = {}
            debug_camera_config["camera_distance"] = 1.5
            debug_camera_config["camera_yaw"] = 0.
            debug_camera_config["camera_pitch"] = -30
            debug_camera_config["camera_target_position"] = [0.0, 0.0, 0.7]
        self._physics.reset_debug_visualizer(**debug_camera_config)

        self._physics.start()
        self._assets_dir = assets_dir

        if default_init:
            self.default_initialization()
        else:
            self.custom_initialization()

    def update_camera(self, *args, **kwargs):
        self._physics.reset_debug_visualizer(*args, **kwargs)

    def step_simulation(self):
        self._physics.step()

    def add_body(self, file_name, pose=Pose(), scale=1.0, is_static=False, assets_dir=None):
        if assets_dir is None:
            body_file_name = self._assets_dir + file_name
        else:
            body_file_name = assets_dir + file_name
        body_uid = self._physics.add_body(body_file_name, pose=pose, scale=scale, is_static=is_static)
        return body_uid

    def add_primitive_body(self, **kwargs):
        body_uid = self._physics.add_primitive_body(**kwargs)
        return body_uid
        
    def default_initialization(self):
        self.add_body('envs/planes/plane.urdf')
        self.table_uid = self.add_body('envs/tables/table.urdf')

        self.robot_arms = []
        self.robot_arms.append(RobotArm(config_file=os.path.join(os.path.dirname(__file__), "configs/default_franka_panda.yaml"),
                                        bworld=self,
                                        interfaces=self.interfaces))

        self.robot_uids = []
        for arm in self.robot_arms:
            self.robot_uids.append(arm.uid)
        
        # self.robot_uid = self.add_body('robots/panda/panda.urdf', pose=Pose([[0.0, 0.0, 0.5], [0., 0., 0., 1.]]), is_static=True)
        
        # self.laikago_uid = self.add_body('robots/laikago/laikago_toes.urdf', pose=Pose([[0.0, 0.5, 1.0], [0., 0., 1., 0.]]))

        # self.add_body(self._assets_dir + 'ycb/004_sugar_box/google_16k/textured.obj', scale=0.01)

    @property
    def physics(self):
        return self._physics
        
        
    @abc.abstractmethod
    def custom_initialization(self):
        raise NotImplementedError

    # def save(self):

class ViSIIBulletWorld(BulletWorld):
    def __init__(self,
                 use_visii=True,
                 *args, **kwargs):
        super().__init__(*args, **kwargs)
        if use_visii:
            v.initialize(headless=True, verbose=False)
            v.enable_denoiser()

            self.visii_camera = v.entity.create(
                name="camera",
                transform=v.transform.create("camera"),
                camera=v.camera.create("camera"))
            self.update_visii_camera()
            v.set_camera_entity(self.visii_camera)
            v.set_dome_light_intensity(0)
            v.disable_dome_light_sampling()

            # Light
            self.light_0 = v.entity.create(
                name="light_0",
                mesh=v.mesh.create_plane("light_0", flip_z=True),
                transform=v.transform.create("light_1"),
                light=v.light.create("light_1")
            )

            self.light_0.get_light().set_intensity(100)
            self.light_0.get_transform().set_scale((1, 1, 1))
            self.light_0.get_transform().set_position((3, 0, 6))
            self.light_0.get_transform().look_at(
                at=(0, 0, 0),
                up=(0, 0, 1))

            # Floor
            self.floor = v.entity.create(
                name="floor",
                mesh=v.mesh.create_plane("floor"),
                transform=v.transform.create("floor"),
                material=v.material.create("floor"))
            self.floor.get_transform().set_position((0, 0, 0))
            self.floor.get_transform().set_scale((10, 10, 10))

            self.visii_objects = {}

    def update_visii_camera(self,
                      at_vec=(0, 0, 0.5),
                      up_vec=(0, 0, 1),
                      eye_vec=(3.0, 0.0, 1.5)):
        self.visii_camera.get_transform().look_at(
            at=at_vec,
            up=up_vec,
            eye=eye_vec)

    def visii_render(self,
               width=800,
               height=800,
               spp=800):
        for robot_uid in self.robot_uids:
            self.update_visii(robot_uid)
        return v.render(width=width, height=height, samples_per_pixel=spp)

    def visii_render_to_file(self,
                             width=800,
                             height=800,
                             spp=100,
                             file_name="./example.png"):
        for robot_uid in self.robot_uids:
            self.update_visii(robot_uid)        
        v.render_to_file(
            width=width,
            height=height,
            samples_per_pixel=spp,
            file_path=file_name,
        )        
                             
    
    def get_image(self,
                  width=800,
                  height=800):
        return v.render_data(width=width,
                             height=height,
                             start_frame=0,
                             frame_count=500,
                             bounce=0,
                             options='none')
        # 1. save to a tmp image file

        # 2. read as np array and return

    def update_visii(self, object_id, name_string=""):
        print("Debugging: ", self._physics.get_visual_shape_data(object_id))
        for (i, visual) in enumerate(self._physics.get_visual_shape_data(object_id)):
            # Extract visual data from pybullet
            objectUniqueId = visual[0]
            linkIndex = visual[1]
            visualGeometryType = visual[2]
            dimensions = visual[3]
            meshAssetFileName = visual[4]
            localVisualFramePosition = visual[5]
            localVisualFrameOrientation = visual[6]
            rgbaColor = visual[7]

            # position = localVisualFramePosition
            # orientation = localVisualFrameOrientation
            # orientation = (orientation[3], orientation[0], orientation[1], orientation[2])

            additional_pos = None
            additional_rot = None
            if linkIndex != -1:
                linkState = self._physics.get_link_pose((objectUniqueId, linkIndex))
                position = linkState[0]
                orientation = linkState[1].quaternion.to_xyzw()

                linkOffsetState = self._physics.get_link_local_offset((objectUniqueId, linkIndex))
                additional_pos = linkOffsetState[0]
                additional_rot = linkOffsetState[1].quaternion.to_xyzw()     
                
            else:
                linkState = self._physics.get_body_pose(objectUniqueId)
                additional_pos = localVisualFramePosition
                additional_rot = localVisualFrameOrientation
                # additional_rot = (additional_rot[3], additional_rot[0], additional_rot[1], additional_rot[2])
                position = linkState[0]
                orientation = linkState[1].quaternion.to_xyzw()
                
            # Name to use for visii components
            object_name = name_string + str(objectUniqueId) + "_" + str(i)
            print(object_name)
            if object_name not in self.visii_objects:
                # Create mesh component if not yet made
                if visualGeometryType == self._physics.geom_mesh:
                    meshAssetFileName = meshAssetFileName.decode('UTF-8')
                    
                    self.visii_objects[object_name] = v.import_scene(
                        meshAssetFileName
                    )
                else:
                    if visualGeometryType == self._physics.geom_box:
                        self.visii_objects[object_name] = v.entity.create(
                            name=object_name,
                            mesh = v.mesh.create_box(object_name),
                            transform = v.transform.create(object_name),
                            material = v.material.create(object_name)
                        )
                        self.visii_objects[object_name].get_transform().set_scale((dimensions[0] / 2,
                                                                                   dimensions[1] / 2,
                                                                                   dimensions[2] / 2))
                    elif visualGeometryType == self._physics.geom_cylinder:
                        self.visii_objects[object_name] = v.entity.create(
                            name=object_name,
                            mesh = v.mesh.create_cylinder(object_name, radius=dimensions[1] / 2, size=dimensions[0] / 2),
                            transform = v.transform.create(object_name),
                            material = v.material.create(object_name)
                        )
                        
                    self.visii_objects[object_name].get_transform().set_position(position)
                    if additional_pos is not None and linkIndex == -1:
                        self.visii_objects[object_name].get_transform().add_position(additional_pos)
                    self.visii_objects[object_name].get_transform().set_rotation(orientation)
                    if additional_rot is not None:
                        self.visii_objects[object_name].get_transform().add_rotation(additional_rot)
                        
                    self.visii_objects[object_name].get_material().set_base_color(rgbaColor[:3])
                    self.visii_objects[object_name].get_material().set_roughness(0.7)
                    
            if visualGeometryType != 5: continue

            self.visii_objects[object_name].transforms[0].set_scale(dimensions)
            self.visii_objects[object_name].transforms[0].set_position(position)
            if additional_pos is not None and linkIndex == -1:
                self.visii_objects[object_name].transforms[0].add_position(additional_pos)

            self.visii_objects[object_name].transforms[0].set_rotation(orientation)
            if additional_rot is not None:
                self.visii_objects[object_name].transforms[0].add_rotation(additional_rot)

            
            # print(meshAssetFileName)
            # print(position, additional_pos)
            # print(orientation, additional_rot)

    def __del__(self):
        v.deinitialize()
