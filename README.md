# Robot Voyager 

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.0-green)](https://developer.nvidia.com/isaac-sim)


A **Voyager-style autonomous skill learning agent** for robotic manipulation in Isaac Sim.

Inspired by [MineDojo/Voyager](https://github.com/MineDojo/Voyager), this system:
- Uses an LLM to **generate robot skills** (Python code)
- Learns through **trial and error** in simulation
- Builds a **skill library**
- Follows a **curriculum** of progressively harder tasks
- Stores only **verified successful** skills

NOTE: we also implement a manual controll, and manual critic.

## Start

### 1. Setup Environment

```bash
# Clone/download this project (best to clone in stand alone in isaacsim)
cd robot_voyager

# Create virtual environment
conda create --name isaacsim

conda activate isaacsim

# Install dependencies
pip install -r requirements.txt

# Configure LLM
cp .env.example .env
# Edit .env with your LLM endpoint and API key
```



### 2. Run with Isaac Sim (Franka + RMPflow)

```bash
cd robot_voyager
# Full agent
C:\isaacsim\python.bat -m apps.run_voyager --max-tasks 10

# Only for testing  
C:\isaacsim\python.bat -m apps.run_agent --backend isaac_rmpflow_franka

```

### 3. Evaluate What Happened After Runs

Run evaluation without launching new Isaac sessions:

```bash
cd robot_voyager

# Evaluate latest run only
python -m apps.evaluate_runs --aggregate-only --num-runs 1

# Evaluate many past runs (last 10)
python -m apps.evaluate_runs --aggregate-only --num-runs 10

```

## Configuration

### Environment Variables (.env)

```bash
# First: Edit .env with your LLM endpoint and API key
# LLM endpoint (OpenAI-compatible)
LLM_BASE_URL=https://api.openai.com/v1
LLM_API_KEY=your-key
LLM_MODEL=llama

# Agent settings
AGENT_MAX_ATTEMPTS=3
AGENT_TIMEOUT_S=25
```

### Command Line Options

```bash
python -m apps.run_voyager --help

Options:
  --headless # Isaac Sim headless mode
  --max-attempts # Max attempts
```


## How It Works

### The Learning Loop

1. **Curriculum** provides the next task (example "pick up the cube")
2. **Skill Library** retrieves relevant existing skills via vector similarity
3. **Planner** asks LLM to generate Python code for the task
4. **Sandbox** executes the code on the robot
5. **Verifier** checks if the task succeeded (cube height > threshold)
6. **On Success**: Skill is saved to vector DB
7. **On Failure**: LLM-critic reflects on error and tries again (up to N attempts)

### Skills

Skills are Python modules with this structure:

```python
SKILL_METADATA = {
    "name": "pick_cube", # Skill name (func name)
    "description": "Pick up a cube from the table", # Describe to retrieve and save into Vector DB
}

def run(robot, **kwargs) -> bool:
    """Execute the skill. Return True on success."""
    robot.log("Starting pick_cube")
    
    cube_pos = robot.get_object_position("cube")
    
    robot.move_ee((cube_pos[0], cube_pos[1], cube_pos[2] + 0.15))
    robot.open_gripper()
    
    robot.move_ee((cube_pos[0], cube_pos[1], cube_pos[2] + 0.02))
    
    robot.close_gripper()
    
    robot.move_ee((cube_pos[0], cube_pos[1], cube_pos[2] + 0.20))
    
    robot.log("pick_cube complete")
    return True
```

### Robot API

All skills use this interface:

```python
# Motion
robot.move_ee(target_xyz, timeout_s=10, pos_tolerance=0.02) -> bool

# Environment
robot.list_objects() -> ["cube", "sphere", ...]
robot.get_object_position("cube") -> (x, y, z)
robot.get_target_position() -> (x, y, z)

```







## References

- [Voyager Paper](https://arxiv.org/abs/2305.16291) - Original Minecraft agent
- [MineDojo/Voyager](https://github.com/MineDojo/Voyager) - Reference implementation
- [Isaac Sim](https://developer.nvidia.com/isaac-sim) - NVIDIA robotics simulator

