'''

import numpy as np


def main():
    print("\n" + "="*60)
    print("NVIDIA PickPlaceController Test")
    print("="*60 + "\n")
    
    # Initialize Isaac Sim
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})
    
    # Import after SimulationApp
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
    from isaacsim.robot.manipulators.examples.franka import Franka
    from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
    
    # Create world (from NVIDIA docs)
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Add Franka (from NVIDIA docs)
    franka = world.scene.add(
        Franka(prim_path="/World/Franka", name="franka")
    )
    
    # Add cube (from NVIDIA docs pattern)
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Cube",
            name="cube",
            position=np.array([0.5, 0.15, 0.025]),  # On table, reachable
            scale=np.array([0.05, 0.05, 0.05]),
            color=np.array([1.0, 0.0, 0.0]),
        )
    )
    
    # Add target marker (visual)
    target_marker = world.scene.add(
        VisualCuboid(
            prim_path="/World/Target",
            name="target",
            position=np.array([0.5, -0.15, 0.025]),
            scale=np.array([0.05, 0.05, 0.05]),
            color=np.array([0.0, 1.0, 0.0]),
        )
    )
    
    # Reset world
    world.reset()
    
    # Create PickPlaceController (from NVIDIA docs)
    controller = PickPlaceController(
        name="pick_place_controller",
        gripper=franka.gripper,
        robot_articulation=franka,
    )
    
    # Open gripper initially (from NVIDIA docs)
    franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
    
    # Step to settle
    for _ in range(10):
        world.step(render=True)
    
    def print_state():
        ee_pos, _ = franka.end_effector.get_world_pose()
        gripper_pos = franka.gripper.get_joint_positions()
        gripper_width = gripper_pos[0] + gripper_pos[1] if gripper_pos is not None else 0
        cube_pos, _ = cube.get_world_pose()
        print(f"  EE:      ({ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f})")
        print(f"  Gripper: {gripper_width:.4f}m")
        print(f"  Cube:    ({cube_pos[0]:.4f}, {cube_pos[1]:.4f}, {cube_pos[2]:.4f})")
    
    print("\n--- Initial State ---")
    print_state()
    
    # Get positions (from NVIDIA docs pattern)
    cube_position, _ = cube.get_world_pose()
    goal_position = np.array([0.5, -0.15, 0.025])  # Target position
    
    print(f"\nPick from: {cube_position}")
    print(f"Place at:  {goal_position}")
    
    input("\nPress Enter to start pick-and-place...")
    
    # Run PickPlaceController (EXACTLY from NVIDIA docs)
    print("\nRunning PickPlaceController...")
    step = 0
    max_steps = 3000
    
    while not controller.is_done() and step < max_steps:
        # This is the EXACT pattern from NVIDIA documentation
        current_joint_positions = franka.get_joint_positions()
        
        actions = controller.forward(
            picking_position=cube_position,
            placing_position=goal_position,
            current_joint_positions=current_joint_positions,
        )
        
        franka.apply_action(actions)
        world.step(render=True)
        step += 1
        
        # Progress
        if step % 300 == 0:
            ee_pos, _ = franka.end_effector.get_world_pose()
            gripper_pos = franka.gripper.get_joint_positions()
            gripper_w = gripper_pos[0] + gripper_pos[1] if gripper_pos is not None else 0
            cube_pos, _ = cube.get_world_pose()
            print(f"  Step {step}: EE z={ee_pos[2]:.3f}, Gripper={gripper_w:.4f}, Cube z={cube_pos[2]:.3f}")
    
    print(f"\nCompleted in {step} steps")
    print("\n--- Final State ---")
    print_state()
    
    # Check success
    final_cube_pos, _ = cube.get_world_pose()
    dist_to_goal = np.linalg.norm(final_cube_pos[:2] - goal_position[:2])
    
    print(f"\nDistance to goal (XY): {dist_to_goal:.4f}m")
    if dist_to_goal < 0.05:
        print("\n✅ SUCCESS: Cube placed at goal!")
    else:
        print("\n❌ FAILED: Cube not at goal")
    
    # Interactive mode
    print("\n" + "="*60)
    print("Interactive Mode - Commands: r=reset, p=pick-place, s=state, q=quit")
    print("="*60)
    
    while simulation_app.is_running():
        try:
            cmd = input("\n> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 's':
                print_state()
            elif cmd == 'r':
                world.reset()
                controller.reset()
                franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
                cube.set_world_pose(position=np.array([0.5, 0.15, 0.025]))
                for _ in range(10):
                    world.step(render=True)
                print("Reset complete")
                print_state()
            elif cmd == 'p':
                # Run pick-and-place again
                controller.reset()
                cube_position, _ = cube.get_world_pose()
                print(f"Picking from {cube_position}...")
                
                step = 0
                while not controller.is_done() and step < 3000:
                    actions = controller.forward(
                        picking_position=cube_position,
                        placing_position=goal_position,
                        current_joint_positions=franka.get_joint_positions(),
                    )
                    franka.apply_action(actions)
                    world.step(render=True)
                    step += 1
                
                print(f"Done in {step} steps")
                print_state()
            else:
                world.step(render=True)
                
        except KeyboardInterrupt:
            print("\nUse 'q' to quit")
        except Exception as e:
            print(f"Error: {e}")
    
    simulation_app.close()


if __name__ == "__main__":
    main()'''