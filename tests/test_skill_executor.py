"""Tests for runtime validation of composed skill kwargs."""

from robot_voyager.agent.skill_executor import create_skill_context
from robot_voyager.agent.skill_library import Skill


_SKILL_CODE = """
SKILL_METADATA = {
    "name": "pick_and_place_cube",
    "description": "Pick and place helper",
    "tags": ["manipulation"]
}

def run(robot, **kwargs):
    robot.log("running pick_and_place_cube")
    return kwargs.get("cube_name") == "cube1" and kwargs.get("target_xyz") is not None
"""

_INNER_SKILL_CODE = """
SKILL_METADATA = {
    "name": "inner_skill",
    "description": "Inner composed skill",
    "tags": ["composition"]
}

def run(robot, **kwargs):
    robot.log("inner skill ran")
    return kwargs.get("value") == 42
"""

_OUTER_SKILL_CODE = """
SKILL_METADATA = {
    "name": "outer_skill",
    "description": "Outer skill that calls inner skill",
    "tags": ["composition"]
}

def run(robot, **kwargs):
    return skills.call("inner_skill", value=42)
"""


class _StubLibrary:
    def __init__(self):
        self._skill = Skill(
            name="pick_and_place_cube",
            description="Pick and place helper",
            tags=["manipulation"],
            code=_SKILL_CODE,
            accepted_kwargs=["cube_name", "target_xyz"],
        )

    def list(self):
        return [{"name": "pick_and_place_cube"}]

    def get(self, name):
        if name == "pick_and_place_cube":
            return self._skill
        return None


class _StubRobot:
    def log(self, msg: str) -> None:
        _ = msg


class _NestedStubLibrary:
    def __init__(self):
        self._skills = {
            "inner_skill": Skill(
                name="inner_skill",
                description="Inner composed skill",
                tags=["composition"],
                code=_INNER_SKILL_CODE,
                accepted_kwargs=["value"],
            ),
            "outer_skill": Skill(
                name="outer_skill",
                description="Outer skill",
                tags=["composition"],
                code=_OUTER_SKILL_CODE,
                accepted_kwargs=[],
            ),
        }

    def list(self):
        return [{"name": "inner_skill"}, {"name": "outer_skill"}]

    def get(self, name):
        return self._skills.get(name)


def test_skills_call_rejects_unknown_kwargs():
    context = create_skill_context(_StubLibrary(), _StubRobot())
    result = context["skills"].call(
        "pick_and_place_cube",
        cube_name="cube1",
        target_xy=(0.5, 0.0),
    )
    assert result is False


def test_skills_call_accepts_declared_kwargs():
    context = create_skill_context(_StubLibrary(), _StubRobot())
    result = context["skills"].call(
        "pick_and_place_cube",
        cube_name="cube1",
        target_xyz=(0.5, 0.0, 0.02),
    )
    assert result is True


def test_nested_composition_call_works():
    context = create_skill_context(_NestedStubLibrary(), _StubRobot())
    result = context["skills"].call("outer_skill")
    assert result is True
