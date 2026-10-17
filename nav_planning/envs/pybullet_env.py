import time
from typing import Any

import numpy as np
import pybullet as pb

from nav_planning.envs.base_env import BaseEnv
from nav_planning.types import RobotConfig
from nav_planning.utils import pybullet_interface as vpb


class PyBulletEnv(BaseEnv):
    def __init__(
        self,
        config: RobotConfig,
        visualize: bool = True,
        viz_urdf_path: str | None = None,
    ):
        self.config = config
        urdf = viz_urdf_path if viz_urdf_path else config.urdf_path
        self.sim = vpb.PyBulletSimulator(urdf, config.joint_names, visualize=visualize)
        self.joint_names = config.joint_names

        # Find camera link index
        self.camera_link_idx = -1
        # Use skel_id (body id) to query joints
        num_joints = self.sim.client.getNumJoints(self.sim.skel_id)
        for i in range(num_joints):
            info = self.sim.client.getJointInfo(self.sim.skel_id, i)
            # info[12] is the child link name (bytes)
            if info[12].decode("utf-8") == config.camera.link_name:
                self.camera_link_idx = i
                break

        # Both robots' home pose: base at the origin, camera straight ahead.
        self.set_joint_states(np.zeros(len(self.joint_names)))

    def get_joint_states(self) -> np.ndarray:
        states = []
        for joint_idx in self.sim.joints:
            state = self.sim.client.getJointState(self.sim.skel_id, joint_idx)
            states.append(state[0])
        return np.array(states)

    def set_joint_states(self, config: np.ndarray):
        self.sim.set_joint_positions(np.asarray(config))

    def set_configuration(self, config: np.ndarray):
        """Apply a full 5-DOF config (base x, y, theta, pan, tilt).

        The base is on the URDF's virtual planar joints, so this is a
        joint-state update.
        """
        self.set_joint_states(config)

    def get_localization(self) -> np.ndarray:
        """Base pose ``[x, y, theta]``."""
        return self.get_joint_states()[:3]

    def get_rgbd(self):
        if self.camera_link_idx == -1:
            return None

        # Get link state
        ls = self.sim.client.getLinkState(self.sim.skel_id, self.camera_link_idx)
        pos = ls[0]
        orn = ls[1]

        # Calculate view matrix
        rot_mat = np.array(self.sim.client.getMatrixFromQuaternion(orn)).reshape(3, 3)

        # Optical frame: Z forward, Y down.
        forward = rot_mat[:, 2]
        up = -rot_mat[:, 1]

        view_matrix = self.sim.client.computeViewMatrix(
            cameraEyePosition=pos, cameraTargetPosition=pos + forward, cameraUpVector=up
        )

        width = self.config.camera.width
        height = self.config.camera.height
        fov = self.config.camera.fov
        aspect = width / height
        near = self.config.camera.near
        far = self.config.camera.far

        proj_matrix = self.sim.client.computeProjectionMatrixFOV(fov, aspect, near, far)

        img = self.sim.client.getCameraImage(width, height, view_matrix, proj_matrix)

        # img[2] is RGB, img[3] is Depth
        rgb = np.array(img[2], dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
        depth = np.array(img[3], dtype=np.float32).reshape(height, width)

        return {"rgb": rgb, "depth": depth}

    def get_obs(self) -> Any:
        return {
            "joint_states": self.get_joint_states(),
            "localization": self.get_localization(),
            "camera": self.get_rgbd(),
        }

    def step(self):
        self.sim.client.stepSimulation()

    def wait_key(self, key: str | int, message: str = "") -> None:
        """Block until *key* is pressed in the PyBullet GUI.

        Prints *message* to stdout, then polls keyboard events until the
        requested key is triggered.  Accepts either a single character
        (``"n"``) or a raw PyBullet key code (e.g. ``pb.B3G_RIGHT_ARROW``).
        Returns early if the GUI window is closed.
        """
        client = self.sim.client
        key_code = ord(key) if isinstance(key, str) else int(key)
        if message:
            print(message)
        while client.isConnected():
            keys = client.getKeyboardEvents()
            if key_code in keys and keys[key_code] & pb.KEY_WAS_TRIGGERED:
                break
            time.sleep(0.01)

    def wait_for_close(self) -> None:
        """Block until the user closes the PyBullet GUI window."""
        client = self.sim.client
        while client.isConnected():
            client.getKeyboardEvents()  # keeps the event queue alive
            time.sleep(0.05)

    def animate_path(
        self,
        path: np.ndarray,
        fps: float = 60.0,
        next_key: str | int | None = None,
    ) -> bool:
        """Interactively play back a full-DOF path, VAMP-style.

        Each row of *path* is handed to :meth:`set_configuration`.  The
        method blocks on the PyBullet GUI with these controls:

        * ``SPACE``             — toggle auto-play (loops end → start)
        * ``← / →``             — step one waypoint back / forward (while paused)
        * ``next_key`` (opt.)   — exit and return ``True`` — useful when
          the caller wants to drive a sequence of demos

        Exits when the user closes the GUI window (returns ``False``) or
        presses *next_key* if one is provided (returns ``True``).  The
        animation starts paused so the caller can read the banner before
        anything moves.

        Args:
          path: ``(N, dof)`` array of full-DOF configurations.
          fps:  playback frame rate while auto-playing.
          next_key: optional single-character string or raw PyBullet key
            code; pressing it exits the viewer.

        Returns:
          ``True`` if *next_key* was pressed, ``False`` otherwise.
        """
        client = self.sim.client
        dt = 1.0 / fps
        n = int(len(path))
        idx = 0
        playing = False

        left = pb.B3G_LEFT_ARROW
        right = pb.B3G_RIGHT_ARROW
        space = ord(" ")
        next_code: int | None = None
        if next_key is not None:
            next_code = ord(next_key) if isinstance(next_key, str) else int(next_key)

        parts = ["SPACE play/pause", "←/→ step"]
        if next_code is not None:
            parts.append(f"'{next_key}' next")
        parts.append("close window to exit")
        print("  " + "  |  ".join(parts))

        advanced_to_next = False
        while client.isConnected():
            self.set_configuration(path[idx])

            keys = client.getKeyboardEvents()
            if space in keys and keys[space] & pb.KEY_WAS_TRIGGERED:
                playing = not playing
            elif (
                next_code is not None
                and next_code in keys
                and keys[next_code] & pb.KEY_WAS_TRIGGERED
            ):
                advanced_to_next = True
                break
            elif not playing and left in keys and keys[left] & pb.KEY_WAS_TRIGGERED:
                idx = (idx - 1) % n
            elif not playing and right in keys and keys[right] & pb.KEY_WAS_TRIGGERED:
                idx = (idx + 1) % n
            elif playing:
                idx = (idx + 1) % n

            time.sleep(dt)

        return advanced_to_next

    def add_pointcloud(
        self, points: np.ndarray, lifetime: float = 0.0, pointsize: int = 3
    ):
        self.sim.draw_pointcloud(points, lifetime, pointsize)
