"""Seed skill: Approach an object from above."""

SKILL_METADATA = {
    "name": "approach_object",
    "description": "Move end-effector to position above an object for grasping",
    "tags": ["motion", "approach", "manipulation", "pregrasp"],
}


def run(robot, **kwargs) -> bool:
    """
    Approach an object from above.
    
    kwargs:
        object_name: name of object to approach (default "cube")
        height_offset: height above object to stop (default 0.15m)
    
    Returns:
        True if approach position reached
    """
    object_name = kwargs.get("object_name", "cube")
    height_offset = kwargs.get("height_offset", 0.15)
    
    robot.log(f"Approaching object: {object_name}")
    
    # Get object position
    obj_pos = robot.get_object_position(object_name)
    robot.log(f"Object position: {obj_pos}")
    
    # Compute approach position (directly above)
    approach_pos = (obj_pos[0], obj_pos[1], obj_pos[2] + height_offset)
    robot.log(f"Approach position: {approach_pos}")
    
    # Move to approach position
    success = robot.move_ee(approach_pos, pos_tolerance=0.02)
    
    if success:
        robot.log("Approach successful")
    else:
        robot.log("Failed to reach approach position")
    
    return success
