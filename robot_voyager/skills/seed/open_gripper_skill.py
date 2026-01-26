"""Seed skill: Open the gripper."""

SKILL_METADATA = {
    "name": "open_gripper_skill",
    "description": "Open the robot gripper to a specified width",
    "tags": ["gripper", "manipulation", "primitive"],
}


def run(robot, **kwargs) -> bool:
    """
    Open the gripper.
    
    kwargs:
        width: target width in meters (default 0.08 = fully open)
        steps: simulation steps to execute (default 120)
    
    Returns:
        True if gripper opened successfully
    """
    width = kwargs.get("width", 0.08)
    steps = kwargs.get("steps", 120)
    
    robot.log(f"Opening gripper to width: {width}m")
    
    robot.open_gripper(width=width, steps=steps)
    
    actual_width = robot.get_gripper_width()
    robot.log(f"Gripper width after open: {actual_width}m")
    
    return True
