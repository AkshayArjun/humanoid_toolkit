"""Forward kinematics on the KinematicTree, written by hand to check against MuJoCo."""

from humkit.geometry.se3 import SE3
from humkit.geometry.so3 import SO3
from humkit.kinematics.tree import Joint, JointType, KinematicTree, Link
from humkit.types import FloatArray


def joint_transform(joint: Joint, q: FloatArray) -> SE3:
    a = joint.qpos_adr
    if joint.type is JointType.HINGE:
        rot = SO3.Exp(joint.axis * q[a])
    elif joint.type is JointType.BALL:
        rot = SO3(q[a : a + 4])
    elif joint.type is JointType.SLIDE:
        return SE3(SO3.identity(), joint.axis * q[a])
    else:
        raise ValueError(f"joint_transform does not handle {joint.type.name} joints")
    # rotate about the anchor, not the body origin
    return SE3(rot, joint.anchor - rot.act(joint.anchor))


def link_pose(link: Link, parent_pose: SE3, q: FloatArray) -> SE3:
    joint = link.joint
    if joint is None:
        return parent_pose @ link.offset
    if joint.type is JointType.FREE:
        a = joint.qpos_adr
        return SE3(SO3(q[a + 3 : a + 7]), q[a : a + 3])
    return parent_pose @ link.offset @ joint_transform(joint, q)


def fk_recursive(tree: KinematicTree, q: FloatArray) -> dict[str, SE3]:
    poses: dict[str, SE3] = {}

    def visit(link: Link, parent_pose: SE3) -> None:
        pose = link_pose(link, parent_pose, q)
        poses[link.name] = pose
        for child in link.children:
            visit(child, pose)

    visit(tree.root, SE3.identity())
    return poses


def fk_stack(tree: KinematicTree, q: FloatArray) -> dict[str, SE3]:
    poses: dict[str, SE3] = {}
    stack = [(tree.root, SE3.identity())]
    while stack:
        link, parent_pose = stack.pop()
        pose = link_pose(link, parent_pose, q)
        poses[link.name] = pose
        stack.extend((child, pose) for child in reversed(link.children))
    return poses


def fk_topological(tree: KinematicTree, q: FloatArray) -> dict[str, SE3]:
    poses: dict[str, SE3] = {}
    for link in tree:
        parent_pose = poses[link.parent.name] if link.parent else SE3.identity()
        poses[link.name] = link_pose(link, parent_pose, q)
    return poses
