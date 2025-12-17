SKILL_METADATA = {
    "name": "Touch Cube Object",
    "description": "Move end-effector to touch the object 'cube1' within 0.08m",
    "tags": []
}

def run(robot, **kwargs) -> bool:
    # Get the position of cube1
    cube1_position = robot.get_object_position("cube1")
    robot.log(f"Cube1 position: {cube1_position}")
    
    # Calculate the target position to touch the cube
    target_position = (cube1_position[0], cube1_position[1], cube1_position[2] + 0.025)
    robot.log(f"Target position: {target_position}")
    
    # Move the end-effector to the target position
    if not robot.move_ee(target_position, pos_tolerance=0.06, timeout_s=15.0):
        robot.log("Failed to reach target position")
        return False
    
    robot.log("Successfully touched cube1")
    return True