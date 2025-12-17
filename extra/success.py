SKILL_METADATA = {
    "name": "Touch Cube Object",
    "description": "Move end-effector to touch the object 'cube1' within 0.08m",
    "tags": []
}

def run(robot, **kwargs) -> bool:
    cube1_position = robot.get_object_position("cube1")
    robot.log(f"Cube1 position: {cube1_position}")
    
    # Calculate target position to touch the cube
    target_position = (cube1_position[0], cube1_position[1], cube1_position[2] + 0.005)
    robot.log(f"Target position: {target_position}")
    
    # Move end-effector to target position
    if not robot.move_ee(target_position, pos_tolerance=0.06, timeout_s=15.0):
        robot.log("Failed to reach target position")
        return False
    
    # Check if end-effector is close enough to the target position
    ee_position = robot.get_ee_position()
    distance_to_target = ((ee_position[0] - target_position[0])**2 + (ee_position[1] - target_position[1])**2 + (ee_position[2] - target_position[2])**2)**0.5
    robot.log(f"Distance to target: {distance_to_target:.3f}m")
    
    # Return True if end-effector is close enough, False otherwise
    return distance_to_target < 0.08