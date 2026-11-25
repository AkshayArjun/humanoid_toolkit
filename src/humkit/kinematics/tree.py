"""The robot's kinematic tree, read from a compiled MjModel.

A robot is a tree of rigid bodies: every body has exactly one parent, and the
root is the world. MuJoCo stores this as a parent array, ``body_parentid``,
where ``body_parentid[i]`` is the index of body i's parent. MuJoCo also
guarantees ``body_parentid[i] < i``: a parent is always numbered before its
children. That is a topological order, so one pass over the bodies in index
order can link every child to an already-built parent.

Each link carries what forward kinematics needs:

    T_world_child = T_world_parent ∘ offset ∘ T_joint(q)

``offset`` is the body's fixed pose in its parent's frame (``body_pos``,
``body_quat``); ``T_joint`` is the motion of the body's joint, if it has one.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

from humkit.error import ModelLoadError
from humkit.geometry.se3 import SE3
from humkit.geometry.so3 import SO3
from humkit.types import FloatArray

if TYPE_CHECKING:
    import mujoco as mj

    from humkit.robot import Robot


class JointType(Enum):
    """Joint kinds. Values match MuJoCo's ``mjtJoint`` so ``JointType(jnt_type)`` works."""

    FREE = 0  # 6 DoF, qpos (x, y, z, qw, qx, qy, qz)
    BALL = 1  # 3 DoF, qpos is a quaternion
    SLIDE = 2  # 1 DoF translation along the axis
    HINGE = 3  # 1 DoF rotation about the axis


@dataclass(frozen=True)
class Joint:
    """A joint attaching a body to its parent.

    ``axis`` and ``anchor`` are in the child body's frame. ``qpos_adr`` and
    ``dof_adr`` index this joint's entries in ``q`` and ``dq``.
    """

    name: str
    type: JointType
    axis: FloatArray
    anchor: FloatArray
    qpos_adr: int
    dof_adr: int


@dataclass(eq=False)
class Link:
    """A node of the kinematic tree: one rigid body.

    ``eq=False`` keeps identity comparison and hashing: two links are the same
    only if they are the same object. ``parent`` and ``children`` are left out of
    the repr, or printing a link would recurse through the whole tree.
    """

    name: str
    index: int
    offset: SE3
    joint: Joint | None
    parent: Link | None = field(default=None, repr=False)
    children: list[Link] = field(default_factory=list, repr=False)

    @property
    def is_root(self) -> bool:
        return self.parent is None

    @property
    def is_leaf(self) -> bool:
        return not self.children


class KinematicTree:
    """The robot's links, stored in topological order (every parent before its children)."""

    def __init__(self, links: list[Link]) -> None:
        if not links or not links[0].is_root:
            raise ValueError("The first link must be the root (it has no parent)")
        self._links = links
        self._by_name = {link.name: link for link in links}

    @classmethod
    def from_model(cls, model: mj.MjModel) -> KinematicTree:
        """Build the tree in one pass over the bodies, in index order."""
        links: list[Link] = []
        for i in range(int(model.nbody)):
            parent_index = int(model.body_parentid[i])
            if i > 0 and parent_index >= i:
                raise ModelLoadError(f"Body {i} has parent {parent_index}; expected a lower index")
            link = Link(
                name=str(model.body(i).name),
                index=i,
                offset=SE3(SO3(model.body_quat[i]), model.body_pos[i]),
                joint=_read_joint(model, i),
            )
            if i > 0:
                parent = links[parent_index]
                link.parent = parent
                parent.children.append(link)
            links.append(link)
        return cls(links)

    @classmethod
    def from_robot(cls, robot: Robot) -> KinematicTree:
        model, _ = robot._mujoco_handles()
        return cls.from_model(model)

    @property
    def root(self) -> Link:
        return self._links[0]

    def __len__(self) -> int:
        return len(self._links)

    def __iter__(self) -> Iterator[Link]:
        """Links in topological order: each parent comes before its children."""
        return iter(self._links)

    def __getitem__(self, name: str) -> Link:
        try:
            return self._by_name[name]
        except KeyError:
            raise KeyError(f"No link named {name!r}") from None

    def render(self) -> str:
        """The tree as indented text, one link per line (depth-first, pre-order)."""
        lines: list[str] = []

        def visit(link: Link, depth: int) -> None:
            lines.append("  " * depth + link.name + _describe_joint(link.joint))
            for child in link.children:
                visit(child, depth + 1)

        visit(self.root, 0)
        return "\n".join(lines)


def _read_joint(model: mj.MjModel, body: int) -> Joint | None:
    count = int(model.body_jntnum[body])
    if count == 0:
        return None
    if count > 1:
        raise ModelLoadError(
            f"Body {model.body(body).name!r} has {count} joints; only one per body is supported"
        )
    j = int(model.body_jntadr[body])
    return Joint(
        name=str(model.joint(j).name),
        type=JointType(int(model.jnt_type[j])),
        axis=np.array(model.jnt_axis[j], dtype=np.float64),
        anchor=np.array(model.jnt_pos[j], dtype=np.float64),
        qpos_adr=int(model.jnt_qposadr[j]),
        dof_adr=int(model.jnt_dofadr[j]),
    )


def _describe_joint(joint: Joint | None) -> str:
    if joint is None:
        return ""
    if joint.type in (JointType.HINGE, JointType.SLIDE):
        axis = " ".join(f"{a:g}" for a in joint.axis)
        return f"  [{joint.type.name.lower()} {joint.name}, axis ({axis})]"
    return f"  [{joint.type.name.lower()} {joint.name}]"
