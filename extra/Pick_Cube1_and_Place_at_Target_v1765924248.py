SKILL_METADATA = {
    "name": "Pick Cube1 and Place at Target",
    "description": "Pick up 'cube1' and place it at (0.4, -0.1)",
    "tags": ["manipulation", "pick_and_place"]
}

def run(robot, **kwargs) -> bool:
    target_xy = (0.4, -0.1)
    robot.log(f"Pick and place to target: {target_xy}")
    
    # === PHASE 1: PICK ===
    cube1 = robot.get_object_position("cube1")
    robot.log(f"Cube1 at {cube1}")
    
    # Approach from above
    above_cube1 = (cube1[0], cube1[1], cube1[2] + 0.10)
    if not robot.move_ee(above_cube1, pos_tolerance=0.06, timeout_s=15.0):
        robot.log("Failed to reach above cube1")
        return False
    
    robot.open_gripper(width=0.08, steps=120)
    
    # Descend to grasp
    grasp = (cube1[0], cube1[1], cube1[2] + 0.02)
    if not robot.move_ee(grasp, pos_tolerance=0.06, timeout_s=15.0):
        robot.log("Failed to descend to cube1")
        return False
    
    robot.close_gripper(steps=200)
    robot.wait(30)
    
    # Lift
    lift = (cube1[0], cube1[1], 0.20)
    robot.move_ee(lift, pos_tolerance=0.03, timeout_s=15.0)
    
    # Check actual cube1 position (this is the real success check)
    new_pos = robot.get_object_position("cube1")
    robot.log(f"Cube1 now at {new_pos}")
    if new_pos[2] < 0.10:
        robot.log("Failed to lift cube1")
        return False
    
    # === PHASE 2: MOVE TO TARGET ===
    above_target = (target_xy[0], target_xy[1], 0.15)
    robot.log(f"Moving to target: {above_target}")
    robot.move_ee(above_target, pos_tolerance=0.06, timeout_s=15.0)
    
    # === PHASE 3: PLACE ===
    place_pos = (target_xy[0], target_xy[1], 0.05)
    robot.move_ee(place_pos, pos_tolerance=0.06, timeout_s=15.0)
    
    robot.open_gripper(width=0.08, steps=120)
    robot.wait(30)
    
    # Move up to clear
    robot.move_ee(above_target, pos_tolerance=0.06, timeout_s=10.0)
    
    # Verify cube1 is at target
    final_pos = robot.get_object_position("cube1")
    robot.log(f"Cube1 final position: {final_pos}")
    dist = ((final_pos[0] - target_xy[0])**2 + (final_pos[1] - target_xy[1])**2)**0.5
    robot.log(f"Distance from target: {dist:.3f}m")
    return dist < 0.08