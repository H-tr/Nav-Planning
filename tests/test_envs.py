"""The PyBullet environment loads each robot on its planar joints."""

import numpy as np
import pytest

from nav_planning import locobot, ridgeback

pytest.importorskip("pybullet")  # visualisation is an optional dependency


@pytest.mark.parametrize(
    "config", [locobot.locobot_robot_config, ridgeback.ridgeback_robot_config]
)
def test_pybullet_env(config):
    from nav_planning.envs.pybullet_env import PyBulletEnv

    env = PyBulletEnv(config, visualize=False)
    q = np.array([1.0, -0.5, 0.3, 0.2, -0.1])
    env.set_configuration(q)
    np.testing.assert_allclose(env.get_joint_states(), q)
    np.testing.assert_allclose(env.get_localization(), q[:3])
    rgbd = env.get_rgbd()
    assert rgbd["rgb"].shape == (config.camera.height, config.camera.width, 3)
