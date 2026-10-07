# Robot Dynamics — ETH Zürich

Personal workspace for the **Robot Dynamics** course (D-MAVT, ETH Zürich, HS26): my MATLAB implementations of the weekly exercises, the official reference solutions, past midterms for exam practice, and a self-made MATLAB cheatsheet.

The exercises are built around the **ABB IRB120** 6-DoF industrial arm and use the exercise framework provided by the [Robotic Systems Lab (RSL)](https://rsl.ethz.ch).

## Repository structure

```
RobotDynamics/
├── Exercises/        # My implementations (edit the files in problems/)
│   ├── robotdynamics_exercise_1a/   # Forward kinematics
│   └── robotdynamics_exercise_1b/   # Differential kinematics (Jacobians)
├── Solutions/        # Official reference solutions for each exercise
├── Past Midterms/    # Midterm quizzes 2020–2024 (+ solutions where available)
└── Cheatsheet/       # LaTeX source + PDF of my MATLAB / Robot Dynamics cheatsheet
```

## Exercise progress

| Exercise | Topic | Implemented functions | Status |
|---|---|---|---|
| 1a | Forward kinematics | `jointToTransform01`…`56`, `getTransformI0`, `getTransform6E`, `jointToPosition`, `jointToRotMat`, `jointToQuat`, `quatMult`, `quatToRotMat`, `rotMatToQuat`, `rotVecWithQuat` | ✅ |
| 1b | Differential kinematics | `jointToPosJac`, `jointToRotJac` | ✅ |

## Running an exercise

Requires MATLAB (no extra toolboxes needed for the exercises). From inside an exercise folder, e.g. `Exercises/robotdynamics_exercise_1b`:

```matlab
init_workspace;      % add exercise, solution and visualization folders to the path
testviz;             % check the IRB120 visualization works
evaluate_problems;   % test your implementations in problems/ against the reference solutions
```

To inspect a configuration interactively:

```matlab
loadviz;                         % creates the abbRobot object
q = rand(6,1);                   % any 6x1 joint configuration
abbRobot.setJointPositions(q);
```

Reference solutions are shipped as protected `.p` files (and partly as `.m` files) under each exercise's `solutions/` folder.

## Past midterms

| Year | Midterm 1 | Midterm 2 |
|---|---|---|
| 2020 | `20_RD_Midterm_1` (+ solution) | `20_RD_Midterm_2` |
| 2022 | `22_RD_Midterm_1` (+ solution) | `22_RD_Midterm_2` (+ solution PDF) |
| 2023 | `midter2023` | — |
| 2024 | `24_RD Midterm 1` | `RobotDynamics_Quiz2_2024` |

Each midterm follows the same pattern: run `init_workspace`, fill in `problems/`, check with `evaluate_problems`.

## Cheatsheet

`Cheatsheet/RobotDynamics_MATLAB_Cheatsheet.pdf` summarizes MATLAB essentials for the exercise framework, rotation parametrizations (elementary rotations, Euler angles, angle–axis, unit quaternions), and homogeneous transformations / forward kinematics for the IRB120. The `.tex` sources are included for editing.

## Credits

Exercise templates, visualization, models and solutions © Robotic Systems Lab, ETH Zürich. This repository is for personal study.
