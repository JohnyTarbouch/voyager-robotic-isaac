SKILL_METADATA = {
    "name": "Touch Cube1",
    "description": "Move end-effector to touch the object 'cube1' within 0.08m",
    "tags": ["motion", "reach"]
}

def run(robot, **kwargs) -> bool:
    cube1 = robot.get_object_position("cube1")
    robot.log(f"Cube1 at {cube1}")
    
    # Approach from above
    above = (cube1[0], cube1[1], cube1[2] + 0.10)
    if not robot.move_ee(above, pos_tolerance=0.06, timeout_s=15.0):
        robot.log("Failed to reach above cube1")
        return False
    
    # Descend to touch
    touch = (cube1[0], cube1[1], cube1[2] + 0.02)
    if not robot.move_ee(touch, pos_tolerance=0.06, timeout_s=15.0):
        robot.log("Failed to descend to cube1")
        return False
    
    robot.log("Touched cube1")
    return True