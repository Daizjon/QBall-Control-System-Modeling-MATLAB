# QBall 6-DOF Dynamics & Optimal Control Design (MATLAB)

## Overview

This project models, linearizes, and stabilizes a 6-degree-of-freedom (6-DOF) quadrotor platform (QBall) using state-space control methods in MATLAB.

The system includes nonlinear rigid-body dynamics, actuator bandwidth modeling, and full multi-axis coupling. A Linear Quadratic Regulator (LQR) controller is designed to stabilize the system and evaluate closed-loop performance.



## System Modeling

The QBall platform consists of:

- 4 counter-rotating propellers
- 6 degrees of freedom:
  - Translational: X, Y, Z
  - Rotational: Roll, Pitch, Yaw
- Coupled nonlinear rigid-body dynamics

### Nonlinear Equations of Motion

Key modeled dynamics include:

- Roll / Pitch:
  J θ̈ = F L

- Z-axis:
  M Z̈ = 4F cos(r)cos(p) − Mg

- X-axis:
  M Ẍ = 4F sin(p)

- Y-axis:
  M Ÿ = −4F sin(r)

- Yaw:
  Jᵧ θ̈ᵧ = Kᵧ Δτ



## Actuator Dynamics

Motor thrust is modeled as a first-order system:

F = K (ω / (s + ω)) u

Actuator state representation:

v̇ = ω(u − v)

This incorporates motor bandwidth and actuator lag into the control model.



## Linearization & State-Space Formulation

The nonlinear system is linearized about a small-angle equilibrium point.

For each axis, state-space models are derived:

ẋ = Ax + Bu

States include:

- Position / angle
- Velocity
- Actuator state (v)
- Integral state (s) for reference tracking



## System Analysis

- Verified controllability using `ctrb()`
- Verified observability using `obsv()`
- Evaluated eigenstructure of linearized system
- Constructed augmented state model for integral control



## Control Design

An optimal state-feedback controller was designed using Linear Quadratic Regulation (LQR):

u = −Kx + ref

Design steps:

1. Select weighting matrices Q and R
2. Solve Algebraic Riccati Equation
3. Compute optimal gain K using `lqr(A,B,Q,R)`

Closed-loop system:

ẋ = (A − BK)x + Bref



## Results

- Stabilized multi-input multi-output (MIMO) system
- Demonstrated closed-loop eigenvalue shift
- Evaluated response across all 6 DOF
- Validated stability under optimal feedback control



## Technical Focus Areas

- Nonlinear Dynamics Modeling
- State-Space Linearization
- MIMO Control Systems
- LQR Optimal Control
- Controllability & Observability Analysis
- MATLAB-Based Simulation


