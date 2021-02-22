"""
Utility functions for grabbing user inputs
"""

import numpy as np

def input2action(device, control_type="OSC_POSE", robot_name="Panda", gripper_dof=1):
    state = device.get_controller_state()
    # Note: Devices output rotation with x and z flipped to account for robots starting with gripper facing down
    #       Also note that the outputted rotation is an absolute rotation, while outputted dpos is delta pos
    #       Raw delta rotations from neutral user input is captured in raw_drotation (roll, pitch, yaw)
    dpos, rotation, raw_drotation, grasp, reset = (
        state["dpos"],
        state["rotation"],
        state["raw_drotation"],
        state["grasp"],
        state["reset"],
    )

    drotation = raw_drotation[[1, 0, 2]]

    action = None
    if control_type == "OSC_POSE":
        drotation[2] = -drotation[2]
        # Scale rotation for teleoperation (tuned for OSC) -- gains tuned for each device
        drotation = drotation * 50 if isinstance(device, SpaceMouse) else drotation * 1.5
        dpos = dpos * 125 if isinstance(device, SpaceMouse) else dpos * 75

        grasp = 1 if grasp else -1
        action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

    return action, grasp
