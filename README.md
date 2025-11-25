# Voyager-Style Autonomous Robot Agent for Isaac Sim

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.0-green)](https://developer.nvidia.com/isaac-sim)


> An open-ended embodied agent that learns to control a robot through LLM-powered code generation and skill accumulation, inspired by the [Voyager project](https://voyager.minedojo.org/). This is a Project at my University

## Features

- **LLM Integration**: Claude or GPT for code generation
- **Skill Library**: SQLite-based storage for learned behaviors
- **Autonomous Learning**: Self-improvement through experience
- **Manual cmd**: Direct robot control without LLM
- **Comprehensive Logging**: Detailed logs for debugging and analysis
- **Modular Architecture**: Clean separation of concerns
- **Socket API**: Flexible client-server architecture



## Project Structure

```
voyager-robotic-isaac/
├── src/
│   ├── server/  
│   │   ├── main.py
│   │   ├── api_server.py
│   │   └── robot_controller.py
│   ├── manual_cmd/  
│   │   ├── manual_control.py
│   │   ├── isaac_client.py
│   │   └── skill_library.py
│   └── common/
│       ├── config.py
│       └── logger.py
├── logs/ 
├── data/ 
├── tests/ 
└── docs/ 
```

## Quick Start

### Prerequisites

- NVIDIA Isaac Sim 5.0
- Python 3.10+
- (Optional) Anthropic or OpenAI API key

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/JohnyTarbouch/voyager-robotic-isaac.git
cd voyager-robotic-isaac
```

2. **Install dependencies** (for LLM features)
```bash
pip install anthropic  # For Claude
# OR
pip install openai     # For GPT-4
```

3. **Set API key** (optional, for autonomous mode)
```bash
export ANTHROPIC_API_KEY='your-key-here'
# OR
export OPENAI_API_KEY='your-key-here'
```

### Running the Server

```bash
# Navigate to Isaac Sim directory
cd isaacsim

# Run the server (I just used windows)
python.bat C:\isaacsim\standalone_examples\voyager-robotic-isaac\src\server\main.py
```

Wait for: `READY FOR COMMANDS`

### Running the Client

In a new terminal:

```bash
cd isaacsim
python.bat C:\isaacsim\standalone_examples\voyager-robotic-isaac\src\client\manual_control.py
```

## Usage Examples

### Manual Control

```
Command: forward 2          # Move 2m forward
Command: left 90            # Turn left 90 deg
Command: square             # Execute square pattern
Command: save my_square     # Save as skill
Command: list               # View all skills
Command: exec my_square     # Execute saved skill
Command: stats              # Show statistics
```

## Commands

| Command | Description | Example |
|---------|-------------|---------|
| `forward [dist] [speed]` | Move forward | `forward 2 0.5` |
| `backward [dist] [speed]` | Move backward | `backward 1` |
| `left [angle] [speed]` | Turn left | `left 90` |
| `right [angle] [speed]` | Turn right | `right 45` |
| `square` | Execute square pattern | `square` |
| `save [name]` | Save last command | `save my_move` |
| `list` | List all skills | `list` |
| `exec [name]` | Execute skill | `exec my_move` |
| `delete [name]` | Delete skill | `delete my_move` |
| `stats` | Show statistics | `stats` |
| `state` | Get robot state | `state` |
| `help` | Show help | `help` |
| `quit` | Exit | `quit` |



### With LLM (TODO)

```bash
python -m src.client.agent

Agent> task move in a square pattern
Agent> task explore the environment
Agent> skills # Learned skills
```

## Logging

All operations are logged to `logs/` directory:

- `server.log` - Isaac Sim server events
- `client.log` - Client operations
- `commands.log` - Command execution details
- `skills.log` - Skill library operations

## Configuration

Edit `src/common/config.py` to customize:

- Server host and port
- Robot parameters
- Movement defaults
- Log levels and formats
- Database paths

Or use environment variables:

```bash
export ROBOT_SERVER_PORT=8888
export LOG_LEVEL=DEBUG
export LLM_PROVIDER=anthropic
```

## Development

### Project Layout

- **Server** (`src/server/`): Runs inside Isaac Sim, controls the robot
- **Manual controll** (`src/client/`): External Python scripts for control
- **Common** (`src/common/`): Shared configuration and utilities (logs)

### Adding New Commands

1. Add handler in `src/server/api_server.py`
2. Add client method in `src/client/isaac_client.py`

### Testing

```bash
# Run unit tests
pytest tests/

```



## Roadmap

- [ ] LLM agent implementation
- [ ] Vision integration (camera input)
- [ ] Obstacle avoidance
- [ ] Multi-robot coordination
- [ ] Curriculum learning

