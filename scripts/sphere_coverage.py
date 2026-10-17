#!/usr/bin/env python3
"""Measure how well the collision spheres cover the robot's meshes.

For every link of ``<robot>.urdf`` with collision geometry, points are
sampled on the surface of that geometry and tested against the link's
spheres in ``<robot>_spherized.urdf``.  Reported per link:

  covered   share of surface samples inside at least one sphere
  outside   deepest uncovered sample (m): how far the mesh pokes out
  inflate   how far the spheres' bounding box exceeds the geometry's (m)

    python scripts/sphere_coverage.py <resources> <robot> [--samples N]
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from fire import Fire
from trimesh.transformations import euler_matrix


def origin_matrix(node: ET.Element) -> np.ndarray:
    origin = node.find("origin")
    xyz = np.array(origin.get("xyz").split(), dtype=float)
    rpy = np.array(origin.get("rpy").split(), dtype=float)
    matrix = euler_matrix(*rpy, axes="sxyz")
    matrix[:3, 3] = xyz
    return matrix


def collision_mesh(collision: ET.Element, directory: Path) -> trimesh.Trimesh:
    geometry = collision.find("geometry")[0]
    if geometry.tag == "mesh":
        mesh = trimesh.load(directory / geometry.get("filename"), force="mesh")
        mesh.apply_scale(np.array(geometry.get("scale", "1 1 1").split(), dtype=float))
    elif geometry.tag == "box":
        mesh = trimesh.creation.box(np.array(geometry.get("size").split(), dtype=float))
    elif geometry.tag == "cylinder":
        mesh = trimesh.creation.cylinder(
            float(geometry.get("radius")), float(geometry.get("length")), sections=128
        )
    else:
        mesh = trimesh.creation.icosphere(4, float(geometry.get("radius")))
    return mesh.apply_transform(origin_matrix(collision))


def main(resources: str, robot: str, samples: int = 20000) -> None:
    directory = Path(resources).resolve() / robot
    links = ET.parse(directory / f"{robot}.urdf").getroot().findall("link")
    spheres = {
        link.get("name"): np.array(
            [
                [
                    *map(float, c.find("origin").get("xyz").split()),
                    float(c.find("geometry/sphere").get("radius")),
                ]
                for c in link.findall("collision")
            ]
        )
        for link in ET.parse(directory / f"{robot}_spherized.urdf")
        .getroot()
        .findall("link")
    }
    rng = np.random.default_rng(0)
    total, inside, count = 0, 0, 0
    print(
        f"{'link':40s} {'spheres':>7s} {'covered':>8s} {'outside':>8s} {'inflate':>8s}"
    )
    for link in links:
        collisions = link.findall("collision")
        if not collisions:
            continue
        mesh = trimesh.util.concatenate(
            [collision_mesh(c, directory) for c in collisions]
        )
        points = np.vstack(
            [mesh.vertices, trimesh.sample.sample_surface(mesh, samples, seed=rng)[0]]
        )
        s = spheres[link.get("name")]
        depth = np.min(
            np.linalg.norm(points[:, None, :] - s[None, :, :3], axis=2) - s[None, :, 3],
            axis=1,
        )
        lo = (s[:, :3] - s[:, 3:]).min(axis=0)
        hi = (s[:, :3] + s[:, 3:]).max(axis=0)
        inflate = max((mesh.bounds[0] - lo).max(), (hi - mesh.bounds[1]).max())
        covered = np.mean(depth <= 0.0)
        total += len(points)
        inside += int(np.sum(depth <= 0.0))
        count += len(s)
        print(
            f"{link.get('name'):40s} {len(s):7d} {100 * covered:7.2f}% "
            f"{max(depth.max(), 0.0):8.4f} {inflate:8.4f}"
        )
    print(f"{'total':40s} {count:7d} {100 * inside / total:7.2f}%")


if __name__ == "__main__":
    Fire(main)
