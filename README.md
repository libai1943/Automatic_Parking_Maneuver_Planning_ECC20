# Automatic Parking Maneuver Planning with Safe Travel Corridors

MATLAB implementation of the automatic parking maneuver planner proposed in:

> **B. Li, T. Acarman, X. Peng, Y. Zhang, X. Bian, and Q. Kong**,  
> “Maneuver Planning for Automatic Parking with Safe Travel Corridors: A Numerical Optimal Control Approach,”  
> *2020 European Control Conference (ECC)*, pp. 1993–1998, 2020.  
> DOI: **10.23919/ECC51009.2020.9143786**

This repository demonstrates a complete **search-and-optimization framework for autonomous parking**, combining:

- Hybrid A* for coarse kinematically feasible path search;
- a two-disc vehicle representation;
- **Safe Travel Corridors (STCs)** for compact collision-avoidance modeling;
- numerical optimal control for trajectory refinement; and
- AMPL + Ipopt for nonlinear programming.

**If you use this repository, the Safe Travel Corridor formulation, or the associated implementation in academic research, please cite the ECC 2020 paper above.**

---

## Overview

Automatic parking can be naturally formulated as an optimal control problem.

A parking trajectory should simultaneously satisfy:

- initial and terminal vehicle configurations;
- nonholonomic vehicle kinematics;
- velocity, acceleration, steering-angle and steering-rate limits;
- collision avoidance;
- forward and reverse maneuvers; and
- a specified performance objective.

The main difficulty is collision avoidance.

If every vehicle state is constrained directly against every obstacle at every discretization instant, the nonlinear program can become large and computationally expensive as the environment becomes more complicated.

This work addresses that difficulty using **Safe Travel Corridors (STCs)**.

The full planning framework is:

```text
Parking scenario
      |
      v
Dilated occupancy map
      |
      v
Hybrid A* search
      |
      v
Kinematically feasible coarse path
      |
      v
Path resampling + motion-profile initialization
      |
      v
Two-disc vehicle representation
      |
      v
Safe Travel Corridor construction
      |
      v
Reduced-scale optimal control problem
      |
      v
AMPL + Ipopt
      |
      v
Smooth, collision-free parking trajectory
```

The most important idea is that the complicated polygonal environment is handled **before** the final nonlinear optimization.

Once the STCs are generated, the final optimal-control problem only needs simple box constraints on two representative points of the vehicle.

---

# Safe Travel Corridors

Safe flight corridors are widely used in UAV trajectory planning: a point or approximately circular vehicle can be required to remain inside a sequence of collision-free convex regions.

Automatic parking is more difficult because a passenger vehicle is neither a point nor a circle.

A long rectangular vehicle can rotate substantially during a parking maneuver, so constraining only its geometric center is insufficient.

The method therefore approximates the vehicle footprint using **two overlapping discs**:

```text
        vehicle footprint
  +---------------------------+
  |                           |
  |      ( rear )  ( front )  |
  |         ○          ○      |
  |                           |
  +---------------------------+
```

The center of each disc is attached rigidly to the vehicle.

For a vehicle pose

```text
(x, y, theta)
```

the implementation computes two representative centers:

```text
rear representative point:
(xr, yr)

front representative point:
(xf, yf)
```

according to the vehicle heading.

A separate collision-free rectangular corridor is then constructed around each representative point at every trajectory discretization instant.

Conceptually:

```text
rear disc center  ∈ rear safe box

front disc center ∈ front safe box
```

Therefore, the final nonlinear optimizer does not need to reason directly about all obstacle polygons.

It only enforces:

```text
xr_min <= xr <= xr_max
yr_min <= yr <= yr_max

xf_min <= xf <= xf_max
yf_min <= yf <= yf_max
```

at each discretization point.

This is the essential purpose of the Safe Travel Corridor formulation.

---

# Why STCs Matter

Suppose the parking environment contains many polygonal obstacles.

A direct collision-avoidance formulation may require constraints involving:

```text
number of trajectory nodes
        ×
number of obstacles
        ×
vehicle geometry
```

The resulting optimization problem can grow rapidly as more obstacles are added.

With STCs, obstacle geometry is processed when the corridors are constructed.

The final optimal-control problem instead sees a fixed number of corridor bounds per trajectory node:

```text
rear center: 4 bounds
front center: 4 bounds
```

or eight scalar corridor values per discretization instant.

Therefore, the dimension of the collision-avoidance portion of the final NLP is effectively independent of the number of obstacles in the original environment.

This reduction is the main computational motivation behind the method.

---

# Complete Planning Pipeline

The main entry is:

```matlab
RunMe
```

The current implementation executes:

```matlab
InitParams();

[x, y, theta, path_length, completeness_flag] = ...
    SearchHybridAStarPath();

[x, y, theta, v, a, phy, w, tf] = ...
    ResamplePath(x, y, theta);

[~, ~, xr, yr, xf, yf] = ...
    SpecifyLocalBoxes(x, y, theta);

WriteInitialGuess(...);
WriteBoundaryValues();

!ampl rr.run
```

If optimization succeeds:

```matlab
Statics();
Dynamics();
```

are called to visualize the final result.

---

# Stage 1 — Scenario and Parameter Initialization

## `InitParams.m`

This file defines the vehicle, search, optimization, and environment parameters.

### Vehicle kinematic limits

The default values are:

```matlab
vehicle_v_max   = 2.5;   % maximum speed
vehicle_a_max   = 1.0;   % maximum acceleration magnitude
vehicle_phy_max = 0.7;   % maximum steering angle
vehicle_w_max   = 0.5;   % maximum steering-angle rate
```

The minimum turning radius is derived from:

```text
wheelbase
+
maximum steering angle
```

using the kinematic bicycle model.

### Vehicle geometry

The default passenger-vehicle dimensions are:

```matlab
wheelbase      = 2.8 m
front overhang = 0.96 m
rear overhang  = 0.929 m
vehicle width  = 1.942 m
```

The total vehicle length is therefore obtained from:

```text
rear overhang + wheelbase + front overhang
```

The two-disc representation is also initialized here.

The disc radius is chosen according to one half of the vehicle representation, and the longitudinal locations of the rear and front disc centers are stored through:

```matlab
vehicle_geometrics_.r2x
vehicle_geometrics_.f2x
```

---

## Environment

The default environment covers:

```text
x ∈ [-20, 20]
y ∈ [-20, 20]
```

The packaged demonstration loads:

```text
Case1.mat
```

which contains:

```text
initial vehicle configuration
terminal vehicle configuration
polygonal obstacles
```

The paper itself evaluates the method over a much larger collection of benchmark parking scenarios; this repository is a compact demonstration implementation of the planning framework.

---

# Stage 2 — Dilated Occupancy Map

## `CreateDilatedCostmap.m`

Hybrid A* requires fast collision checking.

The polygonal obstacles are first rasterized into a binary occupancy map.

The map is subsequently dilated according to the radius of the two-disc vehicle representation.

The implementation uses morphological dilation:

```matlab
strel(...)
imdilate(...)
```

so that collision checking can be performed efficiently using the representative disc centers.

The basic idea is:

```text
original obstacles
      |
      v
occupancy grid
      |
      v
inflate obstacles by vehicle-disc radius
      |
      v
dilated cost map
```

Once the map has been dilated, a representative point that lies in a free cell corresponds to a disc that does not intersect the original obstacle geometry.

---

# Stage 3 — Hybrid A* Coarse Path Search

## `SearchHybridAStarPath.m`

This is the main coarse-path search routine.

The Hybrid A* state is:

```text
(x, y, theta)
```

rather than only:

```text
(x, y)
```

which allows the search to preserve nonholonomic vehicle behavior.

The default discretization is:

```matlab
resolution_x     = 0.2;
resolution_y     = 0.2;
resolution_theta = 0.2;
```

with:

```matlab
simulation_step = 0.7;
max_iter        = 500;
```

---

## Motion Primitives

The search expands six basic vehicle motions:

```text
forward + left steering
forward + zero steering
forward + right steering

reverse + left steering
reverse + zero steering
reverse + right steering
```

In the source code these correspond to combinations of:

```text
v   ∈ {forward, reverse}
phi ∈ {-phi_max, 0, +phi_max}
```

Each candidate motion is propagated using the kinematic bicycle model.

The implementation numerically integrates:

```text
dx/dt     = v cos(theta)
dy/dt     = v sin(theta)
dtheta/dt = v tan(phi) / L
```

for the specified simulation distance.

This means the coarse Hybrid A* solution already respects the main nonholonomic geometry of the vehicle.

---

# Hybrid A* Collision Checking

For every candidate state, the rear and front representative disc centers are calculated:

```text
(xr, yr)
(xf, yf)
```

Both centers are checked against the dilated occupancy map.

Thus, although the search remains computationally lightweight, it considers the physical extent of the vehicle more accurately than a single-point model.

---

# Hybrid A* Heuristic

The heuristic combines two complementary estimates.

### Nonholonomic estimate

A Reeds–Shepp connection is used to estimate the distance to the goal while accounting for:

- minimum turning radius; and
- forward/reverse driving.

### Obstacle-aware holonomic estimate

A conventional 2-D A* search estimates the route length while considering the obstacle map.

The Hybrid A* heuristic combines these pieces so that it simultaneously reflects:

```text
vehicle turning capability
+
environment topology
```

The implementation also periodically attempts a direct Reeds–Shepp connection to the target, which can accelerate termination when the current search state is suitably located.

---

# Stage 4 — From Path to Trajectory

## `ResamplePath.p`

The Hybrid A* output is a geometric path:

```text
x
y
theta
```

The final optimal-control problem requires a complete trajectory initialization, including:

```text
x
y
theta
v
a
phi
omega
tf
```

`ResamplePath.p` converts the coarse path into the time-parameterized representation required by the optimizer and generates the associated initial motion profiles.

The implementation is distributed as a MATLAB protected `.p` file.

Its role in the complete pipeline is:

```text
Hybrid A* geometric path
        |
        v
resampling / motion initialization
        |
        v
full optimal-control initial guess
```

---

# Stage 5 — Constructing the Safe Travel Corridors

## `SpecifyLocalBoxes.m`

This function implements the most characteristic part of the ECC 2020 method.

For every trajectory sample, it first computes:

```matlab
xr = x + r2x * cos(theta);
yr = y + r2x * sin(theta);

xf = x + f2x * cos(theta);
yf = y + f2x * sin(theta);
```

corresponding to the two vehicle-disc centers.

A separate axis-aligned free-space box is then constructed around each center.

---

## Corridor Expansion

Starting from a representative point, the algorithm expands the local box in four directions:

```text
up
left
down
right
```

using the default expansion increment:

```matlab
optimization_.unit_step = 0.03;
```

and a maximum expansion distance:

```matlab
optimization_.max_step = 10;
```

Expansion in one direction stops when additional growth would violate collision-free space.

The resulting box is therefore locally enlarged until it approaches nearby obstacle boundaries.

---

## Handling an Invalid Initial Center

Because the coarse trajectory is only an initialization, a representative center may occasionally be located at a position where a valid local box cannot immediately be constructed.

The implementation therefore tries small perturbations in four directions using increments of:

```text
0.01 m
```

until a valid corridor seed is found.

The local box is then expanded from the adjusted point.

---

# Corridor Data Passed to the Optimizer

For each trajectory node, eight values are written into:

```text
CC
```

representing:

```text
rear box:
xmin
xmax
ymin
ymax

front box:
xmin
xmax
ymin
ymax
```

These values become the complete collision-avoidance interface between the geometric environment and the final optimal-control problem.

This separation is important:

```text
polygonal obstacles
        |
        v
geometric preprocessing
        |
        v
Safe Travel Corridors
        |
        v
simple bound constraints in NLP
```

---

# Stage 6 — Numerical Optimal Control

## `NLP.mod`

The final parking maneuver is formulated as a nonlinear optimal-control problem using AMPL.

The default discretization is:

```text
Nfe = 100
```

The optimization variables include:

```text
tf       terminal time

x, y     vehicle position
theta    vehicle heading

v        longitudinal velocity
a        longitudinal acceleration

phy      steering angle
w        steering-angle rate

xr, yr   rear representative center
xf, yf   front representative center
```

---

# Objective

The optimization minimizes:

```text
tf
```

so the resulting maneuver is a minimum-time trajectory subject to the imposed vehicle and safety constraints.

---

# Vehicle Kinematics

The optimization explicitly enforces the discrete kinematic bicycle model:

```text
x
y
theta
v
phi
```

together with acceleration and steering-rate dynamics.

Thus, unlike a purely geometric path smoother, the final result is generated subject to vehicle motion constraints.

---

# Boundary Conditions

The parking maneuver begins at:

```text
(x0, y0, theta0)
```

and terminates at:

```text
(xf, yf, thetaf)
```

The source code additionally imposes zero initial and terminal values for the motion-related quantities:

```text
velocity
acceleration
steering angle
steering-angle rate
```

so the demonstration represents a parking maneuver from rest to rest.

---

# Mechanical Bounds

The final NLP imposes:

```text
|v|   <= 2.5
|a|   <= 1.0
|phi| <= 0.7
|w|   <= 0.5
```

using the default configuration.

Negative velocity is permitted, which is essential for parking maneuvers requiring reverse motion.

---

# Collision-Avoidance Constraints in the NLP

The final collision constraints are remarkably compact.

For every discretization point:

```text
rear representative center ∈ rear STC
front representative center ∈ front STC
```

which is implemented through:

```text
CC[i,1] <= xr[i] <= CC[i,2]
CC[i,3] <= yr[i] <= CC[i,4]

CC[i,5] <= xf[i] <= CC[i,6]
CC[i,7] <= yf[i] <= CC[i,8]
```

Notice that the original obstacle polygons no longer appear explicitly in `NLP.mod`.

That is precisely the role of the STC preprocessing stage.

---

# Stage 7 — AMPL and Ipopt

## `WriteInitialGuess.m`

Writes the initialized optimization variables into:

```text
initial_guess0.INIVAL
```

including:

```text
x
y
theta

xr
yr
xf
yf

v
a
phi
w
tf
```

This supplies Ipopt with the Hybrid-A*-based warm start.

---

## `WriteBoundaryValues.m`

Writes the six two-point boundary quantities into:

```text
BV
```

namely:

```text
initial x
initial y
initial heading

terminal x
terminal y
terminal heading
```

---

## `rr.run`

This is the AMPL execution script.

It performs:

```text
load NLP.mod
      |
      v
load initial_guess0.INIVAL
      |
      v
select Ipopt
      |
      v
solve
      |
      v
write opti_flag.txt
```

`RunMe.m` then checks:

```text
opti_flag
```

before displaying the optimized result.

---

# Ipopt Configuration

The supplied `ipopt.opt` uses:

```text
max_cpu_time = 100 s
tol          = 1e-8
mu_strategy  = adaptive
linear_solver = ma57
print_level  = 0
```

The archived package therefore uses the HSL MA57 sparse linear solver with Ipopt.

---

# Visualization

After a successful optimization:

```matlab
Statics();
Dynamics();
```

are called.

### `Statics.p`

Produces a static visualization of the final parking trajectory and vehicle footprints.

### `Dynamics.p`

Visualizes the parking maneuver dynamically.

Both routines are distributed as MATLAB protected `.p` files.

---

# Main Files and Functions

| File | Purpose |
|---|---|
| `RunMe.m` | Main entry; executes the complete planning pipeline |
| `Case1.mat` | Packaged parking scenario |
| `InitParams.m` | Vehicle, environment, Hybrid A*, and optimization parameters |
| `CreateDilatedCostmap.m` | Rasterizes obstacles and constructs the inflated occupancy map |
| `SearchHybridAStarPath.m` | Searches a kinematically feasible coarse parking path |
| `ResamplePath.p` | Converts the coarse path into a trajectory initialization |
| `SpecifyLocalBoxes.m` | Constructs rear/front Safe Travel Corridors |
| `WriteInitialGuess.m` | Writes the NLP warm start |
| `WriteBoundaryValues.m` | Writes initial and terminal vehicle configurations |
| `NLP.mod` | Final minimum-time optimal-control formulation |
| `rr.run` | AMPL/Ipopt execution script |
| `ipopt.opt` | Ipopt solver configuration |
| `Statics.p` | Static trajectory visualization |
| `Dynamics.p` | Dynamic maneuver visualization |
| `Arrow.p` | Plotting utility |

---

# Repository Structure

```text
Automatic_Parking_Maneuver_Planning_ECC20/
│
├── RunMe.m
│
├── Case1.mat
│
├── InitParams.m
├── CreateDilatedCostmap.m
├── SearchHybridAStarPath.m
├── ResamplePath.p
├── SpecifyLocalBoxes.m
│
├── WriteInitialGuess.m
├── WriteBoundaryValues.m
│
├── NLP.mod
├── rr.run
├── ipopt.opt
│
├── Statics.p
├── Dynamics.p
├── Arrow.p
│
├── ampl.exe
├── ampl.lic
├── ipopt.exe
├── libhsl.dll
├── libipoptfort.dll
├── ampltabl_64.dll
│
└── LICENSE
```

---

# Quick Start

Clone the repository:

```shell
git clone https://github.com/libai1943/Automatic_Parking_Maneuver_Planning_ECC20.git
cd Automatic_Parking_Maneuver_Planning_ECC20
```

Open MATLAB and set the repository root as the current working directory.

Then run:

```matlab
RunMe
```

The script will:

```text
1. load the parking scenario
2. initialize planner parameters
3. generate the dilated map
4. run Hybrid A*
5. construct an initial trajectory
6. generate Safe Travel Corridors
7. formulate the NLP input
8. call AMPL + Ipopt
9. visualize the optimized parking maneuver
```

---

# Requirements

The archived implementation is primarily intended for a **Windows + MATLAB + AMPL/Ipopt** environment.

You will need:

```text
MATLAB
AMPL
Ipopt
```

The MATLAB installation should also provide the functionality used by:

```matlab
robotics.ReedsSheppConnection
```

and the image-processing functions:

```matlab
strel
imdilate
```

The latter are used to construct the dilated occupancy map.

---

# AMPL License

The repository is an archived research implementation and contains AMPL-related executable/license files used by the original demonstration.

Users should obtain and use their **own valid AMPL license** in accordance with current AMPL licensing terms.

The solver is invoked from MATLAB through:

```matlab
!ampl rr.run
```

so the AMPL executable must be accessible from the repository directory or through the system path.

---

# A Good Way to Study the Code

For readers interested in understanding the method rather than only running the demo, a useful reading order is:

```text
RunMe.m
   ↓
InitParams.m
   ↓
SearchHybridAStarPath.m
   ↓
CreateDilatedCostmap.m
   ↓
SpecifyLocalBoxes.m
   ↓
NLP.mod
   ↓
rr.run
```

The most important technical connection to understand is:

```text
Hybrid A* path
      ↓
two representative vehicle discs
      ↓
local free-space boxes
      ↓
fixed-size STC constraints
      ↓
minimum-time optimal control
```

Once this connection is clear, the overall architecture of the method becomes straightforward.

---

# Citation

If you use this repository, its source code, the Safe Travel Corridor formulation, or the associated automatic-parking planning framework in your research, **please cite**:

> **B. Li, T. Acarman, X. Peng, Y. Zhang, X. Bian, and Q. Kong**,  
> “Maneuver Planning for Automatic Parking with Safe Travel Corridors: A Numerical Optimal Control Approach,”  
> in *2020 European Control Conference (ECC)*, pp. 1993–1998, 2020.  
> DOI: **10.23919/ECC51009.2020.9143786**

## BibTeX

```bibtex
@inproceedings{li2020stc,
  title={Maneuver Planning for Automatic Parking with Safe Travel Corridors: A Numerical Optimal Control Approach},
  author={Li, Bai and Acarman, Tankut and Peng, Xiaoyan and Zhang, Youmin and Bian, Xuepeng and Kong, Qi},
  booktitle={2020 European Control Conference (ECC)},
  pages={1993--1998},
  year={2020},
  organization={IEEE},
  doi={10.23919/ECC51009.2020.9143786}
}
```

---

# License

This repository is released under the **GNU General Public License v3.0 (GPL-3.0)**.

See `LICENSE` for details.

---

Copyright © 2020 Bai Li.
