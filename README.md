# Mobile Robot Path Planning for Chemical Laboratory

A MATLAB-based mobile robot path planning project for laboratory environments.  
This repository implements and compares several classical path planning algorithms, including:

- Rapidly-exploring Random Tree (RRT)
- RRT* (Optimal RRT)
- A* Search Algorithm

The project includes map generation, path smoothing, robot animation, and operation visualization for laboratory robot navigation.

---

## Project Features

- Classical path planning algorithms
- Laboratory environment map generation
- Path smoothing and optimization
- Robot motion animation
- Operation process visualization
- MATLAB simulation framework

---

## Repository Structure

```text
Path planning/
│
├── README.md
│
├── lab_robot_rrt.m
│
├── rrt_plan.m
├── rrt_star_plan.m
├── a_star_plan.m
│
├── smooth_path.m
│
├── create_lab_map.m
├── plot_lab_map.m
│
├── animate_robot.m
├── show_operation_animation.m
│
└── get_operation_delay.m
```

---

## File Description

### Main Script

- `lab_robot_rrt.m`  
  Main program for laboratory robot path planning simulation.

### Path Planning Algorithms

- `rrt_plan.m`  
  Standard RRT path planning.

- `rrt_star_plan.m`  
  RRT* path planning with rewiring optimization.

- `a_star_plan.m`  
  Grid-based A* path planning.

### Environment Modeling

- `create_lab_map.m`  
  Generate laboratory map and obstacle environment.

- `plot_lab_map.m`  
  Visualize laboratory environment.

### Path Optimization

- `smooth_path.m`  
  Path smoothing and trajectory refinement.

### Visualization

- `animate_robot.m`  
  Robot motion animation.

- `show_operation_animation.m`  
  Visualization of robot operation process.

### Utility Function

- `get_operation_delay.m`  
  Operation timing and delay control.

---

## Algorithms

### RRT
Rapidly-exploring Random Tree is a sampling-based algorithm for feasible path generation in complex environments.

### RRT*
RRT* extends RRT using a rewiring strategy to improve path optimality.

### A*
A* is a heuristic graph search algorithm commonly used for shortest path planning.

---

## Usage

Run the main MATLAB script:

```matlab
lab_robot_rrt
```

Users can:

- Compare different planning algorithms
- Modify laboratory maps
- Adjust planning parameters
- Visualize robot trajectories
- Observe robot operation animations

---

## Example Results

### RRT Path Planning
<img width="1396" height="1121" alt="RRT" src="https://github.com/user-attachments/assets/35379f1d-552a-4047-9b11-f4604212f111" />

### RRT* Path Planning
<img width="1396" height="1121" alt="RRTStar" src="https://github.com/user-attachments/assets/ab05a32e-0836-4aa8-8aad-28952a611027" />

### A* Path Planning
<img width="1396" height="1121" alt="AStar" src="https://github.com/user-attachments/assets/3cfccea3-abc6-4a8a-a2ca-2a328e70ccda" />

---

## Future Work

Possible future extensions include:

- Dynamic obstacle avoidance
- Informed RRT*
- Multi-objective path optimization
- Curvature-constrained trajectory planning
- Multi-robot coordination
- Real-world robot deployment

---

## Research Applications

This project can be extended for research in:

- Mobile robot navigation
- Intelligent optimization algorithms
- Autonomous laboratory systems
- Robotic inspection and measurement
- Robot trajectory optimization

---

## License

This project is intended for academic learning and research purposes.
