"""Planner tests for skill generation validation behavior."""

from robot_voyager.agent.planner import Planner, ReflectionResult


class _StubLLM:
    def __init__(self, response_text: str):
        self._response_text = response_text
        self.last_messages = None

    def chat(self, messages, *args, **kwargs):
        self.last_messages = messages
        return self._response_text

    def chat_json(self, *args, **kwargs):
        return {}


def test_planner_accepts_skill_without_composition_when_skills_available():
    llm = _StubLLM(
        """```python
SKILL_METADATA = {"name": "demo_skill", "description": "demo", "tags": []}

def run(robot, **kwargs):
    robot.log("hello")
    return True
```"""
    )
    planner = Planner(llm)
    result = planner.propose_skill_code(
        task_name="demo",
        task_description="demo task",
        observation={},
        available_skills=[{"name": "open_gripper_skill", "description": "open", "tags": []}],
        attempt=1,
    )
    assert "def run" in result.code


def test_planner_accepts_skill_with_composition_when_skills_available():
    llm = _StubLLM(
        """```python
SKILL_METADATA = {"name": "demo_skill", "description": "demo", "tags": []}

def run(robot, **kwargs):
    return skills.call("open_gripper_skill")
```"""
    )
    planner = Planner(llm)
    result = planner.propose_skill_code(
        task_name="demo",
        task_description="demo task",
        observation={},
        available_skills=[{"name": "open_gripper_skill", "description": "open", "tags": []}],
        attempt=1,
    )
    assert "skills.call(" in result.code


def test_planner_allows_non_composed_code_when_no_skills_available():
    llm = _StubLLM(
        """```python
SKILL_METADATA = {"name": "demo_skill", "description": "demo", "tags": []}

def run(robot, **kwargs):
    robot.log("no composed skills yet")
    return True
```"""
    )
    planner = Planner(llm)
    result = planner.propose_skill_code(
        task_name="demo",
        task_description="demo task",
        observation={},
        available_skills=[],
        attempt=1,
    )
    assert "def run" in result.code


def test_planner_includes_last_error_and_reflection_in_retry_prompt():
    llm = _StubLLM(
        """```python
SKILL_METADATA = {"name": "demo_skill", "description": "demo", "tags": []}

def run(robot, **kwargs):
    return True
```"""
    )
    planner = Planner(llm)
    reflection = ReflectionResult(
        analysis="analysis",
        root_cause="grasp was too low",
        suggested_fixes=["increase approach height"],
        should_retry=True,
    )
    planner.propose_skill_code(
        task_name="demo",
        task_description="demo task",
        observation={},
        available_skills=[],
        attempt=2,
        last_error="timeout during move_ee",
        reflection=reflection,
    )

    assert llm.last_messages is not None
    user_prompt = llm.last_messages[1]["content"]
    assert "Previous attempt failed with error:" in user_prompt
    assert "timeout during move_ee" in user_prompt
    assert "Root cause: grasp was too low" in user_prompt
    assert "increase approach height" in user_prompt


def test_planner_includes_accepted_kwargs_for_available_skills():
    llm = _StubLLM(
        """```python
SKILL_METADATA = {"name": "demo_skill", "description": "demo", "tags": []}

def run(robot, **kwargs):
    return True
```"""
    )
    planner = Planner(llm)
    planner.propose_skill_code(
        task_name="demo",
        task_description="compose with existing skill",
        observation={},
        available_skills=[
            {
                "name": "pick_and_place_cube",
                "description": "Pick up a cube and place it",
                "tags": ["manipulation"],
                "accepted_kwargs": ["cube_name", "target_xyz"],
            }
        ],
        attempt=1,
    )

    assert llm.last_messages is not None
    user_prompt = llm.last_messages[1]["content"]
    assert "accepted_kwargs=['cube_name', 'target_xyz']" in user_prompt
