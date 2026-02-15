import logging
import numpy as np
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


def dist(a: Tuple[float, ...], b: Tuple[float, ...]) -> float:
    """
    Euclidean dist"""
    return float(np.sqrt(sum((x - y) ** 2 for x, y in zip(a, b))))


# old name -> new name
_LEGACY_ALIASES: Dict[str, str] = {
    "cube": "cube_red",
    "cube1": "cube_red",
    "cube2": "cube_green",
    "cube3": "cube_blue",
    "cube4": "cube_yellow",
}


class IsaacFrankaRobotAPI:
    """
    Robot API for Franka in IsaacSim using NVIDIA classes.
    """

    def __init__(self, headless: bool = False):
        """
        Initialize the Franka robot in Isaac Sim.
        """
        self._headless = headless
        self._world = None
        self._franka = None
        self._target_marker = None
        self._pick_place_controller = None
        self._rmpflow = None
        self._articulation_rmpflow = None
        self._log_buffer = []  

        # Object registry
        self._objects: Dict[str, Any] = {}     
        self._object_metadata: Dict[str, dict] = {}  # name -> {shape, color_name, size, initial_pos}

        # Box pos
        self._box_center = (0.7, -0.30, 0.03)

        self._setup_simulation()

    def _register_object(
        self, name: str, prim: Any,
        shape: str, color_name: str, size: str, initial_pos: Tuple[float, float, float],
    ) -> None:
        self._objects[name] = prim
        self._object_metadata[name] = {
            "shape": shape,
            "color_name": color_name,
            "size": size,
            "initial_pos": initial_pos,
        }

    def _resolve_name(self, name: str) -> str:
        """Resolve legacy aliases to current names."""
        return _LEGACY_ALIASES.get(name, name)

    def _setup_simulation(self):
        from isaacsim import SimulationApp
        
        # Launch simulation 
        self._sim_app = SimulationApp({"headless": self._headless})
        
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import (
            DynamicCuboid,
            VisualCuboid,
        )
        from isaacsim.robot.manipulators.examples.franka import Franka
        from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
        
        # Create world
        self._world = World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()
        
        # add Franka
        self._franka = self._world.scene.add(
            Franka(
                prim_path="/World/Franka",
                name="franka",
            )
        )
        
        # Add cubes 
        # Red
        prim = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/CubeRed",
                name="cube_red",
                position=np.array([0.5, 0.15, 0.025]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([1.0, 0.0, 0.0]),
            )
        )
        self._register_object("cube_red", prim, "cube", "red", "0.05m side", (0.5, 0.15, 0.025))

        # Green
        prim = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/CubeGreen",
                name="cube_green",
                position=np.array([0.5, 0.0, 0.025]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        self._register_object("cube_green", prim, "cube", "green", "0.05m side", (0.5, 0.0, 0.025))

        # Blue
        prim = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/CubeBlue",
                name="cube_blue",
                position=np.array([0.5, -0.15, 0.025]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([0.0, 0.0, 1.0]),
            )
        )
        self._register_object("cube_blue", prim, "cube", "blue", "0.05m side", (0.5, -0.15, 0.025))

        # Yellow
        prim = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/CubeYellow",
                name="cube_yellow",
                position=np.array([0.5, -0.30, 0.025]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([1.0, 1.0, 0.0]),
            )
        )
        self._register_object("cube_yellow", prim, "cube", "yellow", "0.05m side", (0.5, -0.30, 0.025))

        #  Add box 
        self._setup_box(VisualCuboid)
        
        # Add target marker 
        self._target_marker = self._world.scene.add(
            VisualCuboid(
                prim_path="/World/TargetMarker",
                name="target_marker",
                position=np.array([0.5, 0.0, 0.7]),
                scale=np.array([0.02, 0.02, 0.02]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        
        # Reset world 
        self._world.reset()
        
        # Initialize highlevel pick and place
        self._pick_place_controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        
        self._setup_rmpflow()
        
        # Open gripper init
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_opened_positions
        )
        
        for _ in range(10):
            self._world.step(render=not self._headless)
        
        logger.info("IsaacFrankaRobotAPI initialized with 4 cubes and box")

    def _setup_box(self, VisualCuboid):
        bx, by, bz = 0.7, -0.30, 0.0
        wall_t = 0.005 # wall thick
        w = 0.12 # width (Y)
        d = 0.10 # depth (X)
        h = 0.08 # height (Z)
        box_color = np.array([0.4, 0.26, 0.13]) # brown

        # Floor
        self._world.scene.add(VisualCuboid(
            prim_path="/World/Box/Floor",
            name="box_floor",
            position=np.array([bx, by, bz + wall_t / 2]),
            scale=np.array([d + 2 * wall_t, w + 2 * wall_t, wall_t]),
            color=box_color,
        ))
        # Back wall
        self._world.scene.add(VisualCuboid(
            prim_path="/World/Box/BackWall",
            name="box_back",
            position=np.array([bx + d / 2 + wall_t / 2, by, bz + h / 2]),
            scale=np.array([wall_t, w + 2 * wall_t, h]),
            color=box_color,
        ))
        # Front wall
        self._world.scene.add(VisualCuboid(
            prim_path="/World/Box/FrontWall",
            name="box_front",
            position=np.array([bx - d / 2 - wall_t / 2, by, bz + h / 2]),
            scale=np.array([wall_t, w + 2 * wall_t, h]),
            color=box_color,
        ))
        # Left wall
        self._world.scene.add(VisualCuboid(
            prim_path="/World/Box/LeftWall",
            name="box_left",
            position=np.array([bx, by + w / 2 + wall_t / 2, bz + h / 2]),
            scale=np.array([d, wall_t, h]),
            color=box_color,
        ))
        # Right wall
        self._world.scene.add(VisualCuboid(
            prim_path="/World/Box/RightWall",
            name="box_right",
            position=np.array([bx, by - w / 2 - wall_t / 2, bz + h / 2]),
            scale=np.array([d, wall_t, h]),
            color=box_color,
        ))

        self._box_center = (bx, by, bz + 0.03)
        logger.info(f"Open-top box built at ({bx}, {by})")

    def _setup_rmpflow(self):
        from isaacsim.core.utils.extensions import get_extension_path_from_name
        from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
        
        # Get motion generation
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = mg_extension_path + "/motion_policy_configs"
        
        self._rmpflow = RmpFlow(
            robot_description_path=rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=rmp_config_dir + "/franka/lula_franka_gen.urdf",
            rmpflow_config_path=rmp_config_dir + "/franka/rmpflow/franka_rmpflow_common.yaml",
            end_effector_frame_name="right_gripper",
            maximum_substep_size=0.00334,
        )
        
        # Create motion policy
        self._articulation_rmpflow = ArticulationMotionPolicy(self._franka, self._rmpflow)
        
        logger.info("RmpFlow initialized for motion control")

    # ==================== Core Robot API ====================
    def reset(self) -> None:
        """
        Reset the robot and scene
        """
        self._world.reset()
        self._pick_place_controller.reset()
        
        for name, meta in self._object_metadata.items():
            self._objects[name].set_world_pose(
                position=np.array(meta["initial_pos"]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
        

        # Open gripper
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_opened_positions
        )
        
        for _ in range(10):
            self._world.step(render=not self._headless)
        
        logger.info("Robot reset complete")

    def reset_arm(self) -> None:
        # Move to a safe home position above the workspace
        home_pos = (0.4, 0.0, 0.35)
        self.move_ee(home_pos, pos_tolerance=0.06, timeout_s=10.0)
        
        # Open gripper
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_opened_positions
        )
        
        for _ in range(10):
            self._world.step(render=not self._headless)
        
        logger.info("Robot arm reset to home pose")

    def get_observation(self) -> Dict[str, Any]:
        """
        Get current robot and environment obs
        """
        ee_pos = self.get_ee_position()
        target_pos = self.get_target_position()
        gripper_width = self.get_gripper_width()
        
        objects = {}
        for obj_name in self.list_objects():
            pos = self.get_object_position(obj_name)
            meta = self._object_metadata.get(obj_name, {})
            objects[obj_name] = {
                "position": pos,
                "shape": meta.get("shape", "unknown"),
                "color": meta.get("color_name", "unknown"),
            }
        
        return {
            "ee_position": ee_pos,
            "target_position": target_pos,
            "gripper_width": gripper_width,
            "objects": objects,
            "joint_positions": tuple(self._franka.get_joint_positions().tolist()),
            "box_position": self.get_box_position(),
        }

    def get_ee_position(self) -> Tuple[float, float, float]:
        from pxr import UsdGeom
        from omni.isaac.core.utils.stage import get_current_stage
        
        stage = get_current_stage()
        ee_prim = stage.GetPrimAtPath("/World/Franka/panda_rightfinger")
        if ee_prim.IsValid():
            xform = UsdGeom.Xformable(ee_prim)
            world_transform = xform.ComputeLocalToWorldTransform(0)
            pos = world_transform.ExtractTranslation()
            return (float(pos[0]), float(pos[1]), float(pos[2]))
        
        pos, _ = self._franka.end_effector.get_world_pose()
        return tuple(pos.tolist())

    def get_target_position(self) -> Tuple[float, float, float]:
        pos, _ = self._target_marker.get_world_pose()
        return tuple(pos.tolist())

    def get_gripper_width(self) -> float:
        positions = self._franka.gripper.get_joint_positions()
        # Franka has two finger sum both
        return float(positions[0] + positions[1])



    # ==================== Object Interaction ====================
    def list_objects(self) -> List[str]:
        return list(self._objects.keys())

    def get_object_position(self, name: str) -> Tuple[float, float, float]:
        resolved = self._resolve_name(name)
        if resolved in self._objects:
            pos, _ = self._objects[resolved].get_world_pose()
            return tuple(pos.tolist())
        raise ValueError(f"Unknown object: {name}")

    def set_object_position(self, name: str, pos: Tuple[float, float, float]) -> None:
        """
        Set position of the object
        """
        resolved = self._resolve_name(name)
        if resolved not in self._objects:
            raise ValueError(f"Unknown object: {name}")

        self._objects[resolved].set_world_pose(position=np.array(pos))
        for _ in range(5):
            self._world.step(render=not self._headless)

    def get_object_metadata(self, name: str) -> dict:
        resolved = self._resolve_name(name)
        if resolved in self._object_metadata:
            return dict(self._object_metadata[resolved])
        raise ValueError(f"Unknown object: {name}")

    def get_box_position(self) -> Tuple[float, float, float]:
        return self._box_center


    # ==================== Motion Control ====================
    def move_ee(
        self,
        target_xyz: Tuple[float, float, float],
        *,
        timeout_s: float = 10.0,
        pos_tolerance: float = 0.02,
    ) -> bool:
        """
        Move end-effector to target position using RmpFlow.
        
        Args:
            target_xyz: Target (x, y, z) position
            timeout_s: Timeout in sec
            pos_tolerance: Position tolerance for success
            
        Returns:
            True if target reached within tolerance
        """
        import numpy as np
        
        target = np.array(target_xyz)
        logger.info(f"Moving EE to {target_xyz}")
        
        # Set RmpFlow target
        self._rmpflow.set_end_effector_target(
            target_position=target,
            target_orientation=None,  # Keep current orientation
        )
        
        max_steps = int(timeout_s * 60) 
        
        for step in range(max_steps):
            current_pos = np.array(self.get_ee_position())
            error = dist(tuple(current_pos), target_xyz)
            
            if error <= pos_tolerance:
                logger.info(f"Reached target (error: {error:.4f}m)")
                return True
            
            action = self._articulation_rmpflow.get_next_articulation_action(1.0 / 60.0)
            self._franka.apply_action(action)
            
            self._world.step(render=not self._headless)
        
        final_error = dist(self.get_ee_position(), target_xyz)
        logger.warning(f"Timeout reached. Final error: {final_error:.4f}m")
        return final_error <= pos_tolerance

    # ==================== Gripper Control ====================
    
    def open_gripper(self, width: float = 0.08, steps: int = 120) -> None:
        """
        Open the gripper using NVIDIA gripper interface
        """
        logger.info(f"Opening gripper")
        
        from isaacsim.core.utils.types import ArticulationAction
        
        target_pos = self._franka.gripper.joint_opened_positions
        
        for _ in range(steps):
            action = ArticulationAction(joint_positions=target_pos)
            self._franka.gripper.apply_action(action)
            self._world.step(render=not self._headless)

    def close_gripper(self, steps: int = 200) -> None:
        """
        Close the gripper using incremental position control.
        
        Uses apply_action with small position increments so physics
        can detect collisions and stop the gripper at the object.
        
        Args:
            steps: Number of simulation steps for closing
        """
        logger.info("Closing gripper")
        
        from isaacsim.core.utils.types import ArticulationAction
        
        # Get current gripper posituin
        current = self._franka.gripper.get_joint_positions()
        if current is None:
            current = self._franka.gripper.joint_opened_positions
        
        target = self._franka.gripper.joint_closed_positions
        
        # Close incrementally - small steps let physics detect collisions
        for i in range(steps):
            # Go towards closed position
            t = min(1.0, (i + 1) / (steps * 0.5)) 
            pos = current * (1 - t) + target * t
            
            action = ArticulationAction(joint_positions=pos)
            self._franka.gripper.apply_action(action)
            self._world.step(render=not self._headless)
        
        # Report final width
        gripper_width = self.get_gripper_width()
        logger.info(f"Gripper closed to width: {gripper_width:.4f}m")




    ########################################################################
    # NOT USED FOR OUR PORJECT (just for testing at the beggining)
    ########################################################################
    # ==================== High-Level Actions ====================
    def pick_object(
        self,
        object_name: str = "cube_red",
        lift_height: float = 0.15,
    ) -> bool:

        obj_pos = self.get_object_position(object_name)
        logger.info(f"Picking {object_name} at {obj_pos}, lifting to {lift_height}m")
        place_pos = (obj_pos[0], obj_pos[1], lift_height)
        
        success = self.pick_and_place(obj_pos, place_pos)
        
        if success:
            new_pos = self.get_object_position(object_name)
            if new_pos[2] >= lift_height - 0.03:
                logger.info(f"Object lifted to z={new_pos[2]:.3f}")
                return True
            else:
                logger.warning(f"Object not at expected height: {new_pos[2]:.3f} < {lift_height}")
                return False
        
        return False
    
    def pick_and_place(
        self,
        pick_position: Tuple[float, float, float],
        place_position: Tuple[float, float, float],
    ) -> bool:
        logger.info(f"Starting pick-and-place: {pick_position} -> {place_position}")
        
        self._pick_place_controller.reset()
        
        pick_pos = np.array(pick_position)
        place_pos = np.array(place_position)
        
        max_steps = 2000
        
        for step in range(max_steps):
            current_joint_positions = self._franka.get_joint_positions()
            
            actions = self._pick_place_controller.forward(
                picking_position=pick_pos,
                placing_position=place_pos,
                current_joint_positions=current_joint_positions,
            )
            
            self._franka.apply_action(actions)
            self._world.step(render=not self._headless)
            
            if self._pick_place_controller.is_done():
                logger.info("Pick-and-place completed!")
                return True
        
        logger.warning("Pick-and-place timed out")
        return False

    # ==================== Utility Methods ====================
    def wait(self, steps: int = 60) -> None:
        for _ in range(steps):
            self._world.step(render=not self._headless)

    def log(self, msg: str) -> None:
        logger.info(f"[IsaacFrankaRobotAPI] {msg}")
        self._log_buffer.append(msg)  # Store in buffer for critic
    
    def clear_log_buffer(self) -> None:
        self._log_buffer = []
    
    def get_execution_logs(self) -> str:
        if not self._log_buffer:
            return ""
        return "\n".join(self._log_buffer)

    def close(self) -> None:
        if self._sim_app:
            self._sim_app.close()
            logger.info("Simulation closed")

def make_isaac_franka_robot(headless: bool = False) -> IsaacFrankaRobotAPI:
    return IsaacFrankaRobotAPI(headless=headless)