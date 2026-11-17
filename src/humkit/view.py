from pathlib import Path

import mujoco as mj
import mujoco.viewer as viewer

MENAGERIE_DIR = Path(__file__).resolve().parent.parent.parent.parent / "mujoco_menagerie"
print(f"Using mujoco_menagerie directory: {MENAGERIE_DIR}")
G1_SCENE = MENAGERIE_DIR / "unitree_g1" / "scene.xml"

if not G1_SCENE.exists():
    raise FileNotFoundError(
        f"Scene file not found: {G1_SCENE}. Clone mujoco_menagerie next to the humkit repo."
    )

model = mj.MjModel.from_xml_path(str(G1_SCENE))
data = mj.MjData(model)

print(f"Model loaded successfully from {G1_SCENE}.")

print(f"Number of generalized coordinates: {model.nq}")
print(f"Number of generalized velocities: {model.nv}")
print(f"Number of actuators: {model.nu}")


def print_joint_types():
    JOINT_TYPES = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
    BIAS_TYPES = {0: "none", 1: "affine", 2: "muscle"}

    print("# Day 1 notes: Unitree G1\n")
    print(f"- nq (position coords): {model.nq}")
    print(f"- nv (velocity coords): {model.nv}")
    print(f"- nu (actuators): {model.nu}\n")

    print("## Joints\n")
    print("| # | name | type | qpos index |")
    print("|---|---|---|---|")
    for i in range(model.njnt):
        j = model.joint(i)
        print(f"| {i} | {j.name} | {JOINT_TYPES[int(j.type[0])]} | {int(j.qposadr[0])} |")

    print("\n## Actuators\n")
    print("| # | name | drives joint | bias type | biasprm[:3] | ctrl range |")
    print("|---|---|---|---|---|---|")
    for i in range(model.nu):
        a = model.actuator(i)
        joint_name = model.joint(int(a.trnid[0])).name
        bias = BIAS_TYPES[int(model.actuator_biastype[i])]
        biasprm = model.actuator_biasprm[i, :3].round(2).tolist()
        ctrl_range = a.ctrlrange.round(2).tolist()
        print(f"| {i} | {a.name} | {joint_name} | {bias} | {biasprm} | {ctrl_range} |")


def viewer_loop():
    with viewer.launch_passive(model, data) as v:
        while v.is_running():
            mj.mj_step(model, data)
            v.sync()


def main():
    print_joint_types()
    viewer_loop()


if __name__ == "__main__":
    main()
