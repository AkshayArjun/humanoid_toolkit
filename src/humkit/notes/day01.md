Using mujoco_menagerie directory: /home/akshay/mujoco_menagerie
Model loaded successfully from /home/akshay/mujoco_menagerie/unitree_g1/scene.xml.
Number of generalized coordinates: 36
Number of generalized velocities: 35
Number of actuators: 29
# Day 1 notes: Unitree G1

- nq (position coords): 36
- nv (velocity coords): 35
- nu (actuators): 29

## Joints

| # | name | type | qpos index |
|---|---|---|---|
| 0 | floating_base_joint | free | 0 |
| 1 | left_hip_pitch_joint | hinge | 7 |
| 2 | left_hip_roll_joint | hinge | 8 |
| 3 | left_hip_yaw_joint | hinge | 9 |
| 4 | left_knee_joint | hinge | 10 |
| 5 | left_ankle_pitch_joint | hinge | 11 |
| 6 | left_ankle_roll_joint | hinge | 12 |
| 7 | right_hip_pitch_joint | hinge | 13 |
| 8 | right_hip_roll_joint | hinge | 14 |
| 9 | right_hip_yaw_joint | hinge | 15 |
| 10 | right_knee_joint | hinge | 16 |
| 11 | right_ankle_pitch_joint | hinge | 17 |
| 12 | right_ankle_roll_joint | hinge | 18 |
| 13 | waist_yaw_joint | hinge | 19 |
| 14 | waist_roll_joint | hinge | 20 |
| 15 | waist_pitch_joint | hinge | 21 |
| 16 | left_shoulder_pitch_joint | hinge | 22 |
| 17 | left_shoulder_roll_joint | hinge | 23 |
| 18 | left_shoulder_yaw_joint | hinge | 24 |
| 19 | left_elbow_joint | hinge | 25 |
| 20 | left_wrist_roll_joint | hinge | 26 |
| 21 | left_wrist_pitch_joint | hinge | 27 |
| 22 | left_wrist_yaw_joint | hinge | 28 |
| 23 | right_shoulder_pitch_joint | hinge | 29 |
| 24 | right_shoulder_roll_joint | hinge | 30 |
| 25 | right_shoulder_yaw_joint | hinge | 31 |
| 26 | right_elbow_joint | hinge | 32 |
| 27 | right_wrist_roll_joint | hinge | 33 |
| 28 | right_wrist_pitch_joint | hinge | 34 |
| 29 | right_wrist_yaw_joint | hinge | 35 |

## Actuators

| # | name | drives joint | bias type | biasprm[:3] | ctrl range |
|---|---|---|---|---|---|
| 0 | left_hip_pitch_joint | left_hip_pitch_joint | affine | [0.0, -500.0, -43.01] | [-2.53, 2.88] |
| 1 | left_hip_roll_joint | left_hip_roll_joint | affine | [0.0, -500.0, -39.59] | [-0.52, 2.97] |
| 2 | left_hip_yaw_joint | left_hip_yaw_joint | affine | [0.0, -500.0, -9.83] | [-2.76, 2.76] |
| 3 | left_knee_joint | left_knee_joint | affine | [0.0, -500.0, -15.85] | [-0.09, 2.88] |
| 4 | left_ankle_pitch_joint | left_ankle_pitch_joint | affine | [0.0, -500.0, -5.06] | [-0.87, 0.52] |
| 5 | left_ankle_roll_joint | left_ankle_roll_joint | affine | [0.0, -500.0, -4.56] | [-0.26, 0.26] |
| 6 | right_hip_pitch_joint | right_hip_pitch_joint | affine | [0.0, -500.0, -43.01] | [-2.53, 2.88] |
| 7 | right_hip_roll_joint | right_hip_roll_joint | affine | [0.0, -500.0, -39.59] | [-2.97, 0.52] |
| 8 | right_hip_yaw_joint | right_hip_yaw_joint | affine | [0.0, -500.0, -9.83] | [-2.76, 2.76] |
| 9 | right_knee_joint | right_knee_joint | affine | [0.0, -500.0, -15.85] | [-0.09, 2.88] |
| 10 | right_ankle_pitch_joint | right_ankle_pitch_joint | affine | [0.0, -500.0, -5.06] | [-0.87, 0.52] |
| 11 | right_ankle_roll_joint | right_ankle_roll_joint | affine | [0.0, -500.0, -4.56] | [-0.26, 0.26] |
| 12 | waist_yaw_joint | waist_yaw_joint | affine | [0.0, -500.0, -23.29] | [-2.62, 2.62] |
| 13 | waist_roll_joint | waist_roll_joint | affine | [0.0, -500.0, -37.34] | [-0.52, 0.52] |
| 14 | waist_pitch_joint | waist_pitch_joint | affine | [0.0, -500.0, -35.22] | [-0.52, 0.52] |
| 15 | left_shoulder_pitch_joint | left_shoulder_pitch_joint | affine | [0.0, -500.0, -16.73] | [-3.09, 2.67] |
| 16 | left_shoulder_roll_joint | left_shoulder_roll_joint | affine | [0.0, -500.0, -13.49] | [-1.59, 2.25] |
| 17 | left_shoulder_yaw_joint | left_shoulder_yaw_joint | affine | [0.0, -500.0, -10.12] | [-2.62, 2.62] |
| 18 | left_elbow_joint | left_elbow_joint | affine | [0.0, -500.0, -9.43] | [-1.05, 2.09] |
| 19 | left_wrist_roll_joint | left_wrist_roll_joint | affine | [0.0, -500.0, -4.55] | [-1.97, 1.97] |
| 20 | left_wrist_pitch_joint | left_wrist_pitch_joint | affine | [0.0, -500.0, -5.44] | [-1.61, 1.61] |
| 21 | left_wrist_yaw_joint | left_wrist_yaw_joint | affine | [0.0, -500.0, -4.87] | [-1.61, 1.61] |
| 22 | right_shoulder_pitch_joint | right_shoulder_pitch_joint | affine | [0.0, -500.0, -16.73] | [-3.09, 2.67] |
| 23 | right_shoulder_roll_joint | right_shoulder_roll_joint | affine | [0.0, -500.0, -13.49] | [-2.25, 1.59] |
| 24 | right_shoulder_yaw_joint | right_shoulder_yaw_joint | affine | [0.0, -500.0, -10.12] | [-2.62, 2.62] |
| 25 | right_elbow_joint | right_elbow_joint | affine | [0.0, -500.0, -9.43] | [-1.05, 2.09] |
| 26 | right_wrist_roll_joint | right_wrist_roll_joint | affine | [0.0, -500.0, -4.55] | [-1.97, 1.97] |
| 27 | right_wrist_pitch_joint | right_wrist_pitch_joint | affine | [0.0, -500.0, -5.44] | [-1.61, 1.61] |
| 28 | right_wrist_yaw_joint | right_wrist_yaw_joint | affine | [0.0, -500.0, -4.87] | [-1.61, 1.61] |
