"""Minimal in-project replacement for ``vamp.pybullet_interface``.

Implements just enough of VAMP's PyBullet helper to satisfy the
visualization layer in this project: loading a URDF, looking up an
ordered list of joint indices, resetting joint positions, and drawing
point clouds.  The underlying
:class:`BulletClient`, the loaded body id (``skel_id``) and the joint
index list (``joints``) are exposed as attributes so callers can drop
into raw PyBullet for anything beyond this surface.
"""

from __future__ import annotations

from typing import List

import numpy as np
import pybullet as pb
from pybullet_utils.bullet_client import BulletClient


class _DisableRendering:
    """Suspend PyBullet's GUI rendering for the duration of the block."""

    def __init__(self, client: BulletClient) -> None:
        self.client = client

    def __enter__(self) -> "_DisableRendering":
        self.client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
        return self

    def __exit__(self, *_) -> None:
        self.client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)


class PyBulletSimulator:
    """Lightweight PyBullet wrapper used by :class:`PyBulletEnv`.

    Loads a URDF, resolves an ordered list of joint indices for the
    names supplied, and exposes the underlying :class:`BulletClient`
    so callers can issue arbitrary PyBullet calls.
    """

    def __init__(self, urdf: str, joints: List[str], visualize: bool = True) -> None:
        if visualize:
            self.client = BulletClient(connection_mode=pb.GUI)
            self.client.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
            self.client.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)
        else:
            self.client = BulletClient(connection_mode=pb.DIRECT)

        self.client.setRealTimeSimulation(0)
        self.urdf = urdf

        with _DisableRendering(self.client):
            self.skel_id = self.client.loadURDF(
                urdf,
                basePosition=(0, 0, 0),
                baseOrientation=(0, 0, 0, 1),
                useFixedBase=True,
                flags=pb.URDF_MAINTAIN_LINK_ORDER,
            )

        # Build a joint info table and pick out the requested joints in
        # the order the caller asked for.
        jtu = [
            [
                i.decode() if isinstance(i, bytes) else i
                for i in self.client.getJointInfo(self.skel_id, j)
            ]
            for j in range(self.client.getNumJoints(self.skel_id))
        ]
        jt = sorted(
            (ji for ji in jtu if ji[1] in joints),
            key=lambda ji: joints.index(ji[1]),
        )

        self.joints = [ji[0] for ji in jt]
        self.lows = [ji[8] for ji in jt]
        self.highs = [ji[9] for ji in jt]

    def set_joint_positions(self, positions) -> None:
        """Reset joint angles to *positions* (no dynamics)."""
        for joint, value in zip(self.joints, positions):
            self.client.resetJointState(self.skel_id, joint, value, targetVelocity=0)

    def draw_pointcloud(
        self, pc: np.ndarray, lifetime: float = 0.0, pointsize: int = 3
    ) -> None:
        """Render a point cloud as PyBullet debug points."""
        pc = np.asarray(pc)
        maxes = np.max(pc, axis=0)
        # Avoid division by zero on degenerate clouds.
        safe_maxes = np.where(maxes == 0, 1.0, maxes)
        colors = 0.8 * (pc / safe_maxes)
        with _DisableRendering(self.client):
            self.client.addUserDebugPoints(
                pc, colors, pointSize=pointsize, lifeTime=lifetime
            )
