# Project Proposal

## Title
**Evaluating Grasp Quality Metrics Under Real-World Uncertainty**

## 1. Motivation
Analytical grasp-quality metrics rely on assumptions about contact geometry, friction, and physical interactions. However, these assumptions may not hold accurately in real-world robotic manipulation.

We aim to investigate whether more sophisticated physics-based models lead to more reliable grasp predictions and execution.

## 2. Main Research Question
**Under what conditions do analytical grasp-quality metrics fail to predict real-world grasp stability?**

**Hypothesis:** Incorporating contact mechanics and uncertainty modeling improves grasp-quality prediction and robustness, but the benefits depend on real-world conditions.

## 3. Methodology
We will develop a robotic grasping pipeline using a **UR7e robotic arm, Robotiq gripper, and Intel RealSense RGB-D camera**.

1. **Perception:** RGB-D → Point Cloud → Surface Normals.
2. **Grasp Generation:** Generate feasible grasp candidates based on contact geometry and gripper constraints.
3. **Grasp Evaluation:** Compare three scoring methods:
   - **A — Geometry-Based:** Contact geometry and normal alignment.
   - **B — Nominal Physics-Based:** Friction cones and grasp wrench analysis.
   - **C — Uncertainty-Aware Physics:** Contact mechanics with friction and perception uncertainty.
4. **Execution:** Inverse Kinematics → Motion Planning → Grasp → Lift.
5. **Validation:** Compare predicted grasp quality against actual hardware performance.

All three methods share the same perception, candidate generation, and execution pipeline.

## 4. Experiments
We will use controlled, 3D-printed objects to investigate:

- **Perception Uncertainty:** How do surface-normal estimation errors affect grasp ranking?
- **Friction Uncertainty:** How does friction coefficient mismatch affect prediction reliability?
- **External Wrench (Stretch Goal):** Does accounting for center-of-mass variation improve grasp-quality prediction?

**Evaluation Metrics:** Grasp success rate, slip resistance, and correlation between predicted grasp quality and observed stability.

## 5. Expected Contributions
1. An integrated RGB-D robotic grasp planning and execution system.
2. A systematic comparison of geometric, physics-based, and uncertainty-aware grasp-quality metrics.
3. Experimental insights into when analytical grasp models succeed or fail under real-world uncertainty.

## 6. Project Scope
- **Must-have:** Complete grasping pipeline, Methods A and B, and real-world evaluation.
- **Research Extension:** Method C and uncertainty analysis.
- **Stretch Goal:** Center-of-mass / external wrench analysis.
