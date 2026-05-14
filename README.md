# Mobile Robot Path Planning for Chemical Laboratory

A MATLAB-based path planning project for mobile robots in chemical laboratory environments.  
This repository demonstrates and compares several classical path planning algorithms, including:

- Rapidly-exploring Random Tree (RRT)
- RRT* (Optimal RRT)
- A* Search Algorithm

The project focuses on obstacle avoidance and feasible path generation for autonomous mobile robots operating in laboratory-like environments.

---

## Project Features

- Implementation of classical path planning algorithms
- Obstacle collision checking
- Path visualization and comparison
- MATLAB-based simulation environment
- Suitable for robotics and algorithm learning

---

## Repository Structure

```text
RRT-Path-Planning/
│
├── README.md
├── main.m
│
├── planners/
│   ├── rrt.m
│   ├── rrt_star.m
│   └── astar.m
│
├── collision/
│   ├── isCollisionFree.m
│   └── lineSegmentIntersect.m
│
├── visualization/
│   ├── drawTree.m
│   └── drawPath.m
│
├── maps/
│   ├── map1.png
│   └── map2.png
│
├── utils/
│   ├── distance.m
│   └── sampling.m
│
└── results/
```

---

## Algorithms

### 1. RRT
Rapidly-exploring Random Tree is a sampling-based path planning algorithm that incrementally expands a search tree toward random samples.

### 2. RRT*
RRT* improves upon RRT by introducing a rewiring mechanism to asymptotically optimize path quality.

### 3. A*
A* is a heuristic graph search algorithm widely used for grid-based shortest path planning.

---

## Usage

Run the main script in MATLAB:

```matlab
main
```

Users can:

- Select different planning algorithms
- Modify map environments
- Adjust planning parameters
- Visualize generated paths

---

## Example Results

### RRT Path Planning
<img width="1396" height="1121" alt="RRT" src="https://github.com/user-attachments/assets/35379f1d-552a-4047-9b11-f4604212f111" />

### RRT* Path Planning
<img width="1396" height="1121" alt="RRTStar" src="https://github.com/user-attachments/assets/ab05a32e-0836-4aa8-8aad-28952a611027" />

### A* Path Planning
<img width="1396" height="1121" alt="AStar" src="https://github.com/user-attachments/assets/3cfccea3-abc6-4a8a-a2ca-2a328e70ccda" />

---

## Future Improvements

Possible future extensions include:

- Dynamic obstacle avoidance
- Informed RRT*
- Hybrid path planning
- Curvature-constrained planning
- Multi-objective optimization
- Real robot deployment

---

## Research Direction

This repository can serve as a foundation for:

- Mobile robot navigation
- Intelligent optimization algorithms
- Autonomous laboratory systems
- Robotic inspection and measurement planning

---

## License

This project is intended for academic learning and research purposes.
