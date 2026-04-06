# Anygrasp pipeline testing

Try running following scripts to test the anygrasp installation.

```bash
cd /dependencies/anygrasp_sdk/grasp_detection
sudo python3 demo.py --checkpoint_path /dependencies/precompiled/weights/detection/checkpoint_detection.tar

cd /dependencies/anygrasp_sdk/grasp_tracking
sudo python3 demo.py --checkpoint_path /dependencies/precompiled/weights/tracking/checkpoint_tracking.tar
```

We need to use `sudo` with python3 because the internal file `.TimeRecord` cannot be created otherwise. Since this is run inside a devcontainer, it should be alright.

Expected outputs

```bash
# for grasp_detection
license passed: True, state: FvrLicenseState.PASSED
WARNING:root:Failed to import geometry msgs in rigid_transformations.py.
WARNING:root:Failed to import ros dependencies in rigid_transforms.py
WARNING:root:autolab_core not installed as catkin package, RigidTransform ros methods will be unavailable
[-0.3800217  -0.15607868  0.229     ] [0.3406217 0.2425703 0.61     ]
[0.64067411 0.48865464 0.47643372 0.46612111 0.36957011 0.32774115
 0.30404177 0.30038139 0.25854513 0.25687858 0.25486824 0.23989183
 0.23129246 0.22511464 0.2155883  0.20436358 0.20320499 0.18320534
 0.17407149 0.17206314]
grasp score: 0.6406741142272949

# for grasp_tracking
license passed: True, state: FvrLicenseState.PASSED
error: XDG_RUNTIME_DIR is invalid or not set in the environment.
Authorization required, but no authorization protocol specified

[Open3D WARNING] GLFW Error: Failed to detect any supported platform
[Open3D WARNING] GLFW initialized for headless rendering.
[Open3D WARNING] GLFW Error: OSMesa: Library not found
[Open3D WARNING] Failed to create window
0 [0]
new grasp_ids, reset filter!
1 [ 71 880 664 338 377]
2 [514 403 606  70 738]
3 [243 188 548 888 905]
4 [580 358 840  65 872]
5 [451 521 174 101 810]
6 [562 820 438 819 381]
7 [542 229 412 436 948]
8 [121 205 407 845 962]
9 [361 844 489 975 624]
10 [325 773 337 915 532]
11 [557 872 232 872 913]
12 [ 374  366 1018  366  397]
13 [ 61 121 325 121 784]
14 [ 62 508 246 508 328]
15 [364 400 518 400 647]
16 [ 125  917 1018  917  365]
17 [389 421 215 421 452]
18 [201 952 366 952 493]
19 [353 695 174 695 638]
20 [1001  751  919  751  434]
21 [ 985  564  551  564 1002]
22 [ 14 270 547 270 683]
23 [523 959 477 959 979]
24 [ 801  427 1011  427  187]
25 [918 519 881 519 945]
26 [896 858 310 858 613]
27 [470 300 370 300  53]
28 [171 522 967 522 864]
29 [673 816 596 816 757]

```