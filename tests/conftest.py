import pytest

from humkit.robot import Robot


@pytest.fixture(scope="session")
def g1() -> Robot:
    return Robot.from_menagerie("unitree_g1", hanging=True, torque_control=True)
