"""Seed skill: Close the gripper."""

SKILL_METADATA = {
    "name": "close_gripper_skill",
    "description": "Close the robot gripper to grasp an object",
    "tags": ["gripper", "manipulation", "grasp", "primitive"],
}


def run(robot, **kwargs) -> bool:
    """
    Close the gripper.
    
    kwargs:
        steps: simulation steps to execute (default 120)
    
    Returns:
        True if gripper closed successfully
    """
    steps = kwargs.get("steps", 120)
    
    robot.log("Closing gripper")
    
    robot.close_gripper(steps=steps)
    
    actual_width = robot.get_gripper_width()
    robot.log(f"Gripper width after close: {actual_width}m")
    
    return True
