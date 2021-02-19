from bullet_world import Joint, Link
from bullet_world.utils import YamlConfig

class RobotArm():
    """This is just a wrapper to get access to joints / links easily."""
    def __init__(self, config_file):
        self.config = YamlConfig(config_file).as_easydict()

        # Load arm joints

        # Load arm links

        # Load gripper joints

        # Load gripper links
        

        # Specify ee link name
        self.ee_link_name = self.config.EE.LINK_NAME
        
    def 


    
