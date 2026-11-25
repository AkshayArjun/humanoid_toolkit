"""Geometric Jacobians from the KinematicTree, written by hand to check against MuJoCo.

Rows are [linear; angular] in the world frame, matching MuJoCo's mj_jacBody and
the [rho, theta] twist order. Columns follow dq, so the matrix is 6 x nv.
"""

import numpy as np

from humkit.geometry.se3 import SE3
from humkit.kinematics.reference import fk_topological
from humkit.kinematics.tree import Joint, JointType, KinematicTree
from humkit.types import FloatArray

DOFS = {JointType.FREE: 6, JointType.BALL: 3, JointType.SLIDE: 1, JointType.HINGE: 1}


def num_dofs(tree: KinematicTree) -> int:
    return sum(DOFS[link.joint.type] for link in tree if link.joint is not None)


def body_jacobian(tree: KinematicTree, q: FloatArray, body: str) -> FloatArray:
    """Maps dq to the world-frame twist [v; w] of the body's origin."""
    poses = fk_topological(tree, q)
    point = poses[body].translation
    jac = np.zeros((6, num_dofs(tree)))

    link = tree[body]
    while link.parent is not None:
        if link.joint is not None:
            columns = joint_columns(link.joint, poses[link.name], point)
            a = link.joint.dof_adr
            jac[:, a : a + columns.shape[1]] = columns
        link = link.parent
    return jac


def joint_columns(joint: Joint, body_pose: SE3, point: FloatArray) -> FloatArray:
    """The Jacobian columns of one joint, for a point given in world coordinates."""
    rot = body_pose.rotation.to_matrix()
    anchor = body_pose.act(joint.anchor)

    if joint.type is JointType.HINGE:
        axis = rot @ joint.axis
        return rotation_columns(axis[:, None], anchor, point)
    if joint.type is JointType.SLIDE:
        return np.concatenate([rot @ joint.axis, np.zeros(3)])[:, None]
    if joint.type is JointType.BALL:
        return rotation_columns(rot, anchor, point)
    # FREE: translation dofs move along the world axes, rotation dofs turn about the body axes
    translation = np.vstack([np.eye(3), np.zeros((3, 3))])
    return np.hstack([translation, rotation_columns(rot, anchor, point)])


def rotation_columns(axes: FloatArray, anchor: FloatArray, point: FloatArray) -> FloatArray:
    """Columns for turning about each world axis (a column of `axes`) through `anchor`."""
    linear = np.cross(axes.T, point - anchor).T
    return np.vstack([linear, axes])
