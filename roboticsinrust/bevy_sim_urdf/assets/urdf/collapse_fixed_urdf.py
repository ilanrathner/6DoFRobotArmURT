from __future__ import annotations

import argparse
import copy
import math
import os
import pathlib
import re
import sys
import zipfile
import xml.etree.ElementTree as ET


Vector = tuple[float, float, float]
Matrix = list[list[float]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collapse fixed joints in an Onshape-exported URDF.")
    parser.add_argument(
        "--input",
        required=True,
        help="Path to an Onshape URDF .zip or a .urdf file.",
    )
    parser.add_argument(
        "--output",
        default="assets/urdf/urdf_assembly_collapsed.urdf",
        help="Collapsed URDF output path.",
    )
    parser.add_argument(
        "--mesh-output-dir",
        default="assets/meshes_onshape",
        help="Directory where meshes from a zip input are extracted.",
    )
    parser.add_argument(
        "--keep-joints",
        default="",
        help="Comma-separated non-fixed joint names to keep. If omitted, all non-fixed joints are kept.",
    )
    parser.add_argument(
        "--drop-joints",
        default="",
        help="Comma-separated non-fixed joint names to collapse as fixed.",
    )
    parser.add_argument(
        "--robot-name",
        default="urt_arm_collapsed",
        help="Robot name in the generated URDF.",
    )
    return parser.parse_args()


def floats(value: str | None, default: Vector = (0.0, 0.0, 0.0)) -> Vector:
    if not value:
        return default
    parts = [float(part) for part in value.split()]
    if len(parts) != 3:
        raise ValueError(f"expected three floats, got: {value}")
    return parts[0], parts[1], parts[2]


def fmt(values: Vector) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def mat_identity() -> Matrix:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a: Matrix, b: Matrix) -> Matrix:
    out = [[0.0 for _ in range(4)] for _ in range(4)]
    for row in range(4):
        for col in range(4):
            out[row][col] = sum(a[row][idx] * b[idx][col] for idx in range(4))
    return out


def rot_from_rpy(rpy: Vector) -> Matrix:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def tf_from_xyz_rpy(xyz: Vector, rpy: Vector) -> Matrix:
    out = rot_from_rpy(rpy)
    out[0][3], out[1][3], out[2][3] = xyz
    return out


def tf_inverse(transform: Matrix) -> Matrix:
    out = mat_identity()
    for row in range(3):
        for col in range(3):
            out[row][col] = transform[col][row]
    translation = (transform[0][3], transform[1][3], transform[2][3])
    inv_t = rotate_vec(out, (-translation[0], -translation[1], -translation[2]))
    out[0][3], out[1][3], out[2][3] = inv_t
    return out


def rotate_vec(rotation_transform: Matrix, vector: Vector) -> Vector:
    return (
        rotation_transform[0][0] * vector[0]
        + rotation_transform[0][1] * vector[1]
        + rotation_transform[0][2] * vector[2],
        rotation_transform[1][0] * vector[0]
        + rotation_transform[1][1] * vector[1]
        + rotation_transform[1][2] * vector[2],
        rotation_transform[2][0] * vector[0]
        + rotation_transform[2][1] * vector[1]
        + rotation_transform[2][2] * vector[2],
    )


def mat_to_xyz_rpy(transform: Matrix) -> tuple[Vector, Vector]:
    xyz = (transform[0][3], transform[1][3], transform[2][3])
    r20 = transform[2][0]
    if abs(r20) < 1.0 - 1e-12:
        pitch = math.asin(-r20)
        roll = math.atan2(transform[2][1], transform[2][2])
        yaw = math.atan2(transform[1][0], transform[0][0])
    else:
        pitch = math.pi / 2 if r20 <= -1.0 else -math.pi / 2
        roll = 0.0
        yaw = math.atan2(-transform[0][1], transform[1][1])
    return xyz, (roll, pitch, yaw)


def origin_transform(element: ET.Element | None) -> Matrix:
    if element is None:
        return mat_identity()
    origin = element.find("origin")
    if origin is None:
        return mat_identity()
    return tf_from_xyz_rpy(floats(origin.attrib.get("xyz")), floats(origin.attrib.get("rpy")))


def set_origin(element: ET.Element, transform: Matrix) -> None:
    xyz, rpy = mat_to_xyz_rpy(transform)
    origin = element.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        element.insert(0, origin)
    origin.attrib["xyz"] = fmt(xyz)
    origin.attrib["rpy"] = fmt(rpy)


def load_urdf(input_path: pathlib.Path, mesh_output_dir: pathlib.Path) -> ET.Element:
    if input_path.suffix.lower() == ".zip":
        with zipfile.ZipFile(input_path) as archive:
            urdf_names = [name for name in archive.namelist() if name.lower().endswith(".urdf")]
            if len(urdf_names) != 1:
                raise SystemExit(f"Expected exactly one URDF in zip, found {len(urdf_names)}")

            mesh_output_dir.mkdir(parents=True, exist_ok=True)
            for name in archive.namelist():
                normalized = name.replace("\\", "/")
                if "/meshes/" not in normalized or normalized.endswith("/"):
                    continue
                target = mesh_output_dir / pathlib.PurePosixPath(normalized).name
                target.write_bytes(archive.read(name))

            return ET.fromstring(archive.read(urdf_names[0]))

    return ET.parse(input_path).getroot()


class UnionFind:
    def __init__(self, names: list[str]) -> None:
        self.parent = {name: name for name in names}

    def find(self, name: str) -> str:
        parent = self.parent[name]
        if parent != name:
            self.parent[name] = self.find(parent)
        return self.parent[name]

    def union(self, left: str, right: str) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def joint_parent(joint: ET.Element) -> str:
    return joint.find("parent").attrib["link"]  # type: ignore[union-attr]


def joint_child(joint: ET.Element) -> str:
    return joint.find("child").attrib["link"]  # type: ignore[union-attr]


def joint_origin(joint: ET.Element) -> Matrix:
    return origin_transform(joint)


def sanitize(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_").lower()
    return sanitized or "link"


def rewrite_mesh_filenames(link: ET.Element, mesh_output_dir: pathlib.Path, output_dir: pathlib.Path) -> None:
    relative_mesh_dir = pathlib.Path(
        os.path.relpath(mesh_output_dir.resolve(), output_dir.resolve())
    )
    for mesh in link.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if filename.startswith("package://"):
            mesh.attrib["filename"] = (relative_mesh_dir / pathlib.PurePosixPath(filename).name).as_posix()


def main() -> int:
    args = parse_args()
    input_path = pathlib.Path(args.input).resolve()
    output_path = pathlib.Path(args.output).resolve()
    mesh_output_dir = pathlib.Path(args.mesh_output_dir).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    keep_names = {name.strip() for name in args.keep_joints.split(",") if name.strip()}
    drop_names = {name.strip() for name in args.drop_joints.split(",") if name.strip()}

    root = load_urdf(input_path, mesh_output_dir)
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = root.findall("joint")
    moving_joints = [joint for joint in joints if joint.attrib.get("type") != "fixed"]
    moving_names = {joint.attrib["name"] for joint in moving_joints}

    if keep_names:
        unknown = keep_names - moving_names
        if unknown:
            raise SystemExit(f"Unknown keep joint(s): {', '.join(sorted(unknown))}")
        kept_moving = keep_names
    else:
        unknown = drop_names - moving_names
        if unknown:
            raise SystemExit(f"Unknown drop joint(s): {', '.join(sorted(unknown))}")
        kept_moving = moving_names - drop_names

    uf = UnionFind(list(links))
    for joint in joints:
        if joint.attrib.get("type") == "fixed" or joint.attrib["name"] not in kept_moving:
            uf.union(joint_parent(joint), joint_child(joint))

    components: dict[str, list[str]] = {}
    for link_name in links:
        components.setdefault(uf.find(link_name), []).append(link_name)

    child_links = {joint_child(joint) for joint in joints}
    root_links = [name for name in links if name not in child_links]
    root_link = root_links[0] if root_links else next(iter(links))

    parent_to_joints: dict[str, list[ET.Element]] = {}
    for joint in joints:
        parent_to_joints.setdefault(joint_parent(joint), []).append(joint)

    world_tf = {root_link: mat_identity()}
    pending = True
    while pending:
        pending = False
        for joint in joints:
            parent = joint_parent(joint)
            child = joint_child(joint)
            if parent in world_tf and child not in world_tf:
                world_tf[child] = mat_mul(world_tf[parent], joint_origin(joint))
                pending = True

    if len(world_tf) != len(links):
        missing = sorted(set(links) - set(world_tf))
        raise SystemExit(f"Could not compute world transforms for: {', '.join(missing[:8])}")

    incoming_kept: dict[str, list[ET.Element]] = {}
    for joint in moving_joints:
        if joint.attrib["name"] not in kept_moving:
            continue
        parent_component = uf.find(joint_parent(joint))
        child_component = uf.find(joint_child(joint))
        if parent_component != child_component:
            incoming_kept.setdefault(child_component, []).append(joint)

    component_rep: dict[str, str] = {}
    for component_root, members in components.items():
        if component_root == uf.find(root_link):
            component_rep[component_root] = root_link
        elif component_root in incoming_kept and incoming_kept[component_root]:
            component_rep[component_root] = joint_child(incoming_kept[component_root][0])
        else:
            component_rep[component_root] = sorted(members)[0]

    root_component = uf.find(root_link)
    component_name = {root_component: "base_link"}
    link_index = 1
    for joint in moving_joints:
        if joint.attrib["name"] not in kept_moving:
            continue
        component = uf.find(joint_child(joint))
        if component not in component_name:
            component_name[component] = f"link{link_index}"
            link_index += 1
    for component in components:
        if component not in component_name:
            component_name[component] = sanitize(component)

    new_robot = ET.Element("robot", {"name": args.robot_name})
    ET.SubElement(new_robot, "link", {"name": "root"})

    for component, members in components.items():
        new_link = ET.SubElement(new_robot, "link", {"name": component_name[component]})
        mass = 0.0
        for member in members:
            inertial = links[member].find("inertial")
            if inertial is not None and inertial.find("mass") is not None:
                try:
                    mass += float(inertial.find("mass").attrib.get("value", "0"))  # type: ignore[union-attr]
                except ValueError:
                    pass

        if mass > 0.0:
            inertial = ET.SubElement(new_link, "inertial")
            ET.SubElement(inertial, "mass", {"value": f"{mass:.9g}"})
            ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            inertia_value = max(mass * 1.0e-4, 1.0e-6)
            ET.SubElement(
                inertial,
                "inertia",
                {
                    "ixx": f"{inertia_value:.9g}",
                    "ixy": "0",
                    "ixz": "0",
                    "iyy": f"{inertia_value:.9g}",
                    "iyz": "0",
                    "izz": f"{inertia_value:.9g}",
                },
            )

        component_frame_inv = tf_inverse(world_tf[component_rep[component]])
        for member in members:
            link_tf = world_tf[member]
            for tag_name in ("visual", "collision"):
                for source in links[member].findall(tag_name):
                    cloned = copy.deepcopy(source)
                    new_origin = mat_mul(mat_mul(component_frame_inv, link_tf), origin_transform(source))
                    set_origin(cloned, new_origin)
                    rewrite_mesh_filenames(cloned, mesh_output_dir, output_path.parent)
                    new_link.append(cloned)

    root_joint = ET.SubElement(new_robot, "joint", {"name": "root_to_base", "type": "fixed"})
    ET.SubElement(root_joint, "parent", {"link": "root"})
    ET.SubElement(root_joint, "child", {"link": component_name[root_component]})
    ET.SubElement(root_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

    kept_joint_summaries: list[str] = []
    for joint in moving_joints:
        if joint.attrib["name"] not in kept_moving:
            continue
        parent_component = uf.find(joint_parent(joint))
        child_component = uf.find(joint_child(joint))
        if parent_component == child_component:
            continue

        parent_frame_inv = tf_inverse(world_tf[component_rep[parent_component]])
        child_frame = world_tf[component_rep[child_component]]
        new_origin_tf = mat_mul(parent_frame_inv, child_frame)

        cloned = copy.deepcopy(joint)
        cloned.find("parent").attrib["link"] = component_name[parent_component]  # type: ignore[union-attr]
        cloned.find("child").attrib["link"] = component_name[child_component]  # type: ignore[union-attr]
        set_origin(cloned, new_origin_tf)

        axis_element = cloned.find("axis")
        if axis_element is not None:
            axis = floats(joint.find("axis").attrib.get("xyz"))  # type: ignore[union-attr]
            original_joint_world = mat_mul(world_tf[joint_parent(joint)], joint_origin(joint))
            axis_world = rotate_vec(original_joint_world, axis)
            axis_new = rotate_vec(tf_inverse(child_frame), axis_world)
            axis_element.attrib["xyz"] = fmt(axis_new)

        new_robot.append(cloned)
        kept_joint_summaries.append(
            f"{cloned.attrib['name']}: {component_name[parent_component]} -> {component_name[child_component]}"
        )

    ET.indent(new_robot, space="  ")
    ET.ElementTree(new_robot).write(output_path, encoding="utf-8", xml_declaration=True)

    print(f"Input links: {len(links)}")
    print(f"Input joints: {len(joints)}")
    print(f"Kept moving joints: {len(kept_joint_summaries)}")
    print(f"Output links: {len(components) + 1} including fixed root")
    print(f"Output joints: {len(kept_joint_summaries) + 1} including root_to_base")
    print("")
    print("Kept joint graph:")
    for summary in kept_joint_summaries:
        print(f"  {summary}")
    print("")
    print("Collapsed components:")
    for component, members in sorted(components.items(), key=lambda item: component_name[item[0]]):
        print(f"  {component_name[component]}: {len(members)} source links, frame={component_rep[component]}")
    print("")
    print(f"Wrote: {output_path}")
    if input_path.suffix.lower() == ".zip":
        print(f"Extracted meshes: {mesh_output_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
