#!/usr/bin/env python3
"""Build the planning descriptions from the official robot descriptions.

``description`` expands each robot's official xacro (downloaded by
``scripts/download_assets.sh``) and writes, into
``nav_planning/resources/robot/<robot>/``:

  <robot>.urdf       the robot on three virtual planar joints
                     (base_x_joint, base_y_joint, base_theta_joint) with
                     every joint but the camera pan and tilt fixed.  Used
                     for visualisation, kinematics and as foam's input.
  <robot>.srdf       self-collision exclusions (see ``write_srdf``).
  <robot>_base.srdf  no exclusions: the base model is one rigid body.
  meshes/            every referenced mesh as STL.

``spheres`` runs after foam: it cuts ``<robot>_base.urdf`` out of the
spherized URDF — the links that do not move with the camera, with exactly
the same spheres; no planner uses it — and completes the SRDF (see
``spheres``).

    python scripts/build_robot_description.py description <assets> <resources>
    python scripts/build_robot_description.py spheres <resources> <robot>
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin
import pymeshlab
import trimesh
from fire import Fire

SCRIPT_DIR = Path(__file__).resolve().parent

# STL files stay under the repository's 1 MB file limit (50 bytes a face).
MAX_FACES = 20000
# Pan / tilt grid spacing for the self-collision survey (rad).
GRID_STEP = math.radians(1.0)
# The virtual base joints' range.  Planners narrow it with set_joint_limits.
BASE_XY_LIMIT = 10.0

ROBOTS = {
    "locobot": {
        # Arm-less LoCoBot on the iRobot Create 3, without the optional lidar.
        "xacro": (
            "interbotix_xslocobot_descriptions",
            "urdf/locobot.urdf.xacro",
        ),
        "mappings": {
            "robot_model": "locobot_base",
            "robot_name": "locobot",
            "arm_model": "mobile_base",
            "base_model": "create3",
            "use_lidar": "false",
        },
        "environment": {},
        "root": "locobot/base_link",
        # iRobot places the Create 3 base_link on the floor.
        "root_height": 0.0,
        "camera_joints": ("pan", "tilt"),
        "joint_limits": {},
    },
    "ridgeback": {
        "xacro": ("ridgeback_description", "urdf/ridgeback.urdf.xacro"),
        "mappings": {},
        # The description's payload hook carries our camera mast.
        "environment": {
            "RIDGEBACK_URDF_EXTRAS": str(
                SCRIPT_DIR / "ridgeback_camera_mast.urdf.xacro"
            )
        },
        "root": "base_link",
        # wheel_radius - axle_offset in ridgeback.urdf.xacro: base_link
        # sits this far above the floor.
        "root_height": 0.0759 - 0.0500,
        "camera_joints": ("ptu_pan", "ptu_tilt"),
        # The flir_ptu xacro's PTU-5 limits (+-178 deg, +31/-81 deg) match no
        # FLIR document; the PTU-5 user manual (v3.10, section 5.1.1) gives
        # the default user limits +-168 deg pan, -90/+30 deg tilt.
        "joint_limits": {
            "ptu_pan": (-math.radians(168.0), math.radians(168.0)),
            "ptu_tilt": (-math.radians(90.0), math.radians(30.0)),
        },
    },
}

# Official ROS packages and their location inside the downloaded assets.
PACKAGES = {
    "interbotix_xslocobot_descriptions": (
        "interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions"
    ),
    "irobot_create_description": "create3_sim/irobot_create_common/irobot_create_description",
    "ridgeback_description": "ridgeback/ridgeback_description",
    "lms1xx": "LMS1xx",
    "flir_ptu_description": "flir_ptu/flir_ptu_description",
    "realsense2_description": "realsense-ros/realsense2_description",
}


def make_ament_prefix(assets: Path) -> Path:
    """An ament prefix exposing the downloaded packages to xacro's $(find)."""
    prefix = assets / "ament"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True, exist_ok=True)
    for name, location in PACKAGES.items():
        (index / name).touch()
        link = prefix / "share" / name
        if not link.is_symlink():
            link.symlink_to(assets / location)
    return prefix


def expand_xacro(prefix: Path, spec: dict) -> ET.Element:
    import xacro

    os.environ["AMENT_PREFIX_PATH"] = str(prefix)
    os.environ.update(spec["environment"])
    package, path = spec["xacro"]
    doc = xacro.process_file(
        str(prefix / "share" / package / path), mappings=spec["mappings"]
    )
    return ET.fromstring(doc.toxml())


def mesh_source(prefix: Path, filename: str) -> Path:
    package, path = filename.removeprefix("package://").split("/", 1)
    return prefix / "share" / package / path


def export_mesh(source: Path, destination: Path) -> None:
    mesh = trimesh.load(source, force="mesh")
    if len(mesh.faces) > MAX_FACES:
        meshes = pymeshlab.MeshSet()
        meshes.add_mesh(pymeshlab.Mesh(mesh.vertices, mesh.faces))
        meshes.meshing_decimation_quadric_edge_collapse(
            targetfacenum=MAX_FACES, preservetopology=True, preservenormal=True
        )
        simple = meshes.current_mesh()
        mesh = trimesh.Trimesh(simple.vertex_matrix(), simple.face_matrix())
    mesh.export(destination)


def planning_urdf(robot: ET.Element, prefix: Path, spec: dict, out: Path) -> None:
    """Freeze, clean and re-root the expanded description in place."""
    # Simulation and control blocks are not part of the kinematic tree.
    for tag in ("gazebo", "transmission", "ros2_control"):
        for node in robot.findall(tag):
            robot.remove(node)
    # The Interbotix texture lives in a simulation package; keep its colour.
    for material in robot.iter("material"):
        for texture in material.findall("texture"):
            material.remove(texture)
            ET.SubElement(material, "color", rgba="0.1 0.1 0.1 1.0")

    # Only the camera moves relative to the base: wheels, wheel drops,
    # rockers and suspension are planned at their zero position.
    for joint in robot.findall("joint"):
        if joint.get("name") in spec["joint_limits"]:
            lower, upper = spec["joint_limits"][joint.get("name")]
            joint.find("limit").set("lower", str(lower))
            joint.find("limit").set("upper", str(upper))
        if joint.get("name") in spec["camera_joints"]:
            continue
        joint.set("type", "fixed")
        for child in ("axis", "limit", "dynamics", "mimic", "safety_controller"):
            for node in joint.findall(child):
                joint.remove(node)

    # foam reads both attributes of every collision origin.
    for collision in robot.iter("collision"):
        origin = collision.find("origin")
        if origin is None:
            origin = ET.SubElement(collision, "origin")
        origin.set("xyz", origin.get("xyz", "0 0 0"))
        origin.set("rpy", origin.get("rpy", "0 0 0"))

    meshes = out / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    for mesh in robot.iter("mesh"):
        source = mesh_source(prefix, mesh.get("filename"))
        name = f"{source.stem}.stl"
        if not (meshes / name).exists():
            export_mesh(source, meshes / name)
        mesh.set("filename", f"meshes/{name}")

    # The three virtual planar joints every subgroup plans over.
    robot.insert(0, ET.Element("link", name="world"))
    chain = [
        ("base_x_joint", "prismatic", "world", "base_x_link", "1 0 0", 0.0),
        ("base_y_joint", "prismatic", "base_x_link", "base_y_link", "0 1 0", 0.0),
        (
            "base_theta_joint",
            "revolute",
            "base_y_link",
            spec["root"],
            "0 0 1",
            spec["root_height"],
        ),
    ]
    for i, (name, kind, parent, child, axis, height) in enumerate(chain):
        joint = ET.Element("joint", name=name, type=kind)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=f"0 0 {height}", rpy="0 0 0")
        ET.SubElement(joint, "axis", xyz=axis)
        bound = BASE_XY_LIMIT if kind == "prismatic" else math.pi
        ET.SubElement(
            joint,
            "limit",
            lower=str(-bound),
            upper=str(bound),
            effort="100",
            velocity="1.0",
        )
        robot.insert(1 + 2 * i, joint)
        if child != spec["root"]:
            robot.insert(2 + 2 * i, ET.Element("link", name=child))


def base_urdf(robot: ET.Element, spec: dict) -> ET.Element:
    """Copy of ``robot`` without the links that move with the camera."""
    base = ET.fromstring(ET.tostring(robot))
    base.set("name", f"{robot.get('name')}_base")
    children = {}
    for joint in base.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    pan = next(
        j for j in base.findall("joint") if j.get("name") == spec["camera_joints"][0]
    )
    stack, removed = [pan], set()
    while stack:
        joint = stack.pop()
        base.remove(joint)
        link = joint.find("child").get("link")
        removed.add(link)
        stack.extend(children.get(link, []))
    for link in base.findall("link"):
        if link.get("name") in removed:
            base.remove(link)
    return base


def write_xml(root: ET.Element, path: Path, comment: str) -> None:
    ET.indent(root)
    with open(path, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write(f"<!-- {comment} -->\n")
        f.write(ET.tostring(root, encoding="unicode"))
        f.write("\n")


def geometry_points(geometry: pin.GeometryObject) -> np.ndarray:
    """Vertices of a collision geometry in its parent joint's frame."""
    shape = geometry.geometry
    if isinstance(shape, pin.coal.Box):
        h = shape.halfSide
        local = np.array(
            [
                [sx * h[0], sy * h[1], sz * h[2]]
                for sx in (-1, 1)
                for sy in (-1, 1)
                for sz in (-1, 1)
            ]
        )
    else:
        local = (
            trimesh.load(geometry.meshPath, force="mesh").vertices * geometry.meshScale
        )
    placement = geometry.placement
    return local @ placement.rotation.T + placement.translation


def write_srdf(urdf: Path, spec: dict, path: Path) -> None:
    """Self-collision exclusions, decided from the meshes.

    Collision geometry rides on three bodies: the base (everything up to
    the pan joint), the pan link and the tilting camera.  Geometry on the
    same body is never checked against itself.  A link pair on different
    bodies is disabled when it is

      Adjacent — the bodies share a joint (base-pan, pan-tilt);
      Never    — the meshes stay apart over a pan x tilt grid by more than
                 any point of the camera moves between grid neighbours, so
                 they can not touch anywhere in the joint range;
      Always   — the meshes touch at every grid point.

    The remaining pairs (base links that the tilted camera can reach) are
    collision-checked.
    """
    model, geom_model = pin.buildModelsFromUrdf(
        str(urdf), str(urdf.parent), geometry_types=pin.GeometryType.COLLISION
    )
    pan_id = model.getJointId(spec["camera_joints"][0])
    tilt_id = model.getJointId(spec["camera_joints"][1])
    link = [model.frames[g.parentFrame].name for g in geom_model.geometryObjects]
    joint = [g.parentJoint for g in geom_model.geometryObjects]

    adjacent, surveyed = set(), {}
    for i in range(geom_model.ngeoms):
        for j in range(i + 1, geom_model.ngeoms):
            if joint[i] == joint[j]:
                continue
            pair = tuple(sorted((link[i], link[j])))
            if (
                model.parents[joint[i]] == joint[j]
                or model.parents[joint[j]] == joint[i]
            ):
                adjacent.add(pair)
            else:
                geom_model.addCollisionPair(pin.CollisionPair(i, j))
                surveyed.setdefault(pair, []).append(len(geom_model.collisionPairs) - 1)

    # Largest displacement of a camera point between grid neighbours: the
    # tilting points lie within ``rho`` of the tilt axis origin, which is
    # ``d`` from the pan axis origin.
    rho = max(
        np.linalg.norm(geometry_points(g), axis=1).max()
        for g in geom_model.geometryObjects
        if g.parentJoint == tilt_id
    )
    d = np.linalg.norm(model.jointPlacements[tilt_id].translation)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    pans = np.linspace(
        lo[pan_id - 1],
        hi[pan_id - 1],
        math.ceil((hi[pan_id - 1] - lo[pan_id - 1]) / GRID_STEP) + 1,
    )
    tilts = np.linspace(
        lo[tilt_id - 1],
        hi[tilt_id - 1],
        math.ceil((hi[tilt_id - 1] - lo[tilt_id - 1]) / GRID_STEP) + 1,
    )
    reach = 0.5 * (pans[1] - pans[0]) * (d + rho) + 0.5 * (tilts[1] - tilts[0]) * rho

    data, geom_data = model.createData(), pin.GeometryData(geom_model)
    closest = np.full(len(geom_model.collisionPairs), np.inf)
    touching = np.zeros(len(geom_model.collisionPairs), dtype=int)
    q = pin.neutral(model)
    for pan_angle in pans:
        for tilt_angle in tilts:
            q[pan_id - 1], q[tilt_id - 1] = pan_angle, tilt_angle
            pin.computeDistances(model, data, geom_model, geom_data, q)
            pin.computeCollisions(geom_model, geom_data, False)
            for k in range(len(geom_model.collisionPairs)):
                closest[k] = min(closest[k], geom_data.distanceResults[k].min_distance)
                touching[k] += geom_data.collisionResults[k].isCollision()

    disabled = {pair: "Adjacent" for pair in adjacent}
    grid = len(pans) * len(tilts)
    for pair, ks in surveyed.items():
        if min(closest[k] for k in ks) > reach:
            disabled[pair] = "Never"
        elif all(touching[k] == grid for k in ks):
            disabled[pair] = "Always"
        print(
            f"  {pair}: closest {min(closest[k] for k in ks):.4f} m, "
            f"touching {max(touching[k] for k in ks)}/{grid} -> {disabled.get(pair, 'checked')}"
        )

    root = ET.Element("robot", name=model.name)
    for (a, b), reason in sorted(disabled.items()):
        ET.SubElement(root, "disable_collisions", link1=a, link2=b, reason=reason)
    write_xml(
        root, path, f"Self-collision exclusions, generated by {Path(__file__).name}"
    )


def description(assets: str, resources: str, robot: str | None = None) -> None:
    """Planning URDF, SRDFs and meshes of ``robot`` (default: every robot)."""
    prefix = make_ament_prefix(Path(assets).resolve())
    for name in [robot] if robot else ROBOTS:
        spec = ROBOTS[name]
        out = Path(resources).resolve() / name
        print(f"[{name}] expanding {spec['xacro'][1]}")
        urdf = expand_xacro(prefix, spec)
        urdf.set("name", name)
        planning_urdf(urdf, prefix, spec, out)
        write_xml(
            urdf,
            out / f"{name}.urdf",
            f"Generated by {Path(__file__).name} from the official description",
        )
        write_xml(
            ET.Element("robot", name=f"{name}_base"),
            out / f"{name}_base.srdf",
            "The base model is one rigid body: no self-collision pairs",
        )
        print(f"[{name}] surveying self-collisions")
        write_srdf(out / f"{name}.urdf", spec, out / f"{name}.srdf")


def spheres(resources: str, robot: str) -> None:
    """Finish the collision models once ``<robot>_spherized.urdf`` exists.

    Cuts ``<robot>_base.urdf`` out of the spherized URDF, and disables in
    ``<robot>.srdf`` every pair whose spheres overlap at the home pose (all
    joints zero) — reason ``Default``: the spheres enclose the meshes with
    some slack, so a checked pair whose meshes only come close would make
    the home pose invalid.  Such a pair is then not checked anywhere.
    """
    out = Path(resources).resolve() / robot
    spherized_path = out / f"{robot}_spherized.urdf"
    spherized = ET.parse(spherized_path).getroot()
    write_xml(
        base_urdf(spherized, ROBOTS[robot]),
        out / f"{robot}_base.urdf",
        f"Cut from {robot}_spherized.urdf by {Path(__file__).name}",
    )

    srdf_path = out / f"{robot}.srdf"
    model, geom_model = pin.buildModelsFromUrdf(
        str(spherized_path), str(out), geometry_types=pin.GeometryType.COLLISION
    )
    geom_model.addAllCollisionPairs()
    pin.removeCollisionPairs(model, geom_model, str(srdf_path))
    data, geom_data = model.createData(), pin.GeometryData(geom_model)
    pin.computeCollisions(model, data, geom_model, geom_data, pin.neutral(model), False)
    srdf = ET.parse(srdf_path).getroot()
    overlapping = set()
    for k, pair in enumerate(geom_model.collisionPairs):
        if geom_data.collisionResults[k].isCollision():
            a, b = (
                model.frames[geom_model.geometryObjects[g].parentFrame].name
                for g in (pair.first, pair.second)
            )
            overlapping.add(tuple(sorted((a, b))))
    for a, b in sorted(overlapping):
        print(f"  ({a}, {b}): spheres overlap at home -> Default")
        ET.SubElement(srdf, "disable_collisions", link1=a, link2=b, reason="Default")
    write_xml(
        srdf,
        srdf_path,
        f"Self-collision exclusions, generated by {Path(__file__).name}",
    )


if __name__ == "__main__":
    Fire({"description": description, "spheres": spheres})
