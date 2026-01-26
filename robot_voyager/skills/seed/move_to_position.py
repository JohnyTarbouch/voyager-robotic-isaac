"""Seed skill: Move end-effector to a target position."""

SKILL_METADATA = {
    "name": "move_to_position",
    "description": "Move the end-effector to a specified xyz position",
    "tags": ["motion", "reach", "basic", "primitive"],
}


def run(robot, **kwargs) -> bool:
    """
    Move end-effector to target position.
    
    kwargs:
        target_xyz: tuple of (x, y, z) coordinates
        tolerance: position tolerance (default 0.02)
    
    Returns:
        True if position reached within tolerance
    """
    target_xyz = kwargs.get("target_xyz")
    tolerance = kwargs.get("tolerance", 0.02)
    
    if target_xyz is None:
        target_xyz = robot.get_target_position()
    
    robot.log(f"Moving to position: {target_xyz}")
    
    success = robot.move_ee(target_xyz, pos_tolerance=tolerance)
    
    if success:
        robot.log("Reached target position")
    else:
        robot.log("Failed to reach target position")
    
    return success
