# A-Star-Algorithms
## Discrete Structure (CO1007) — A* Algorithms Assignment

<p align="center">
  <img src="https://img.shields.io/badge/C++-17-blue?logo=c%2B%2B" />
  <img src="https://img.shields.io/badge/Concepts-A*%20Search%20%7C%20Graphs%20%7C%20Pathfinding-lightgrey" />
  <img src="https://img.shields.io/badge/Status-Completed-brightgreen" />
</p>

---

## Overview

This project is a C++ implementation of multiple pathfinding problems using the A* search algorithm and related graph-search techniques.

The program demonstrates how one search framework can be adapted to different environments, including social networks, weighted drone routes, warehouse grids, and emergency evacuation maps.

The assignment is designed to reinforce core concepts from discrete structures and programming fundamentals, including:

* Graph representation using adjacency matrices
* Linked-list path reconstruction
* Heuristic search
* Shortest-path reasoning
* Grid-based navigation
* Validation of reachable and unreachable cases

---

## Project Structure

```text
.
├── main.cpp
├── Algo.cpp
├── Algo.h
├── PathNode.cpp
├── PathNode.h
├── 2550201.pdf
└── README.md
```

---

## Core Modules

### Task 1 — Degrees of Separation in a Social Network

* Represents people as graph vertices
* Represents friendships as unweighted edges
* Computes the minimum number of social connections from a start person to a target person
* Uses a reverse-BFS-style heuristic to guide path reconstruction

Example output node format:

```text
Node: 0 | f: 2.00 | g: 0.00 | h: 2.00
```

---

### Task 2 — Drone Delivery in 2D Space

* Models drone waypoints as graph vertices with 2D coordinates
* Uses weighted edges to represent travel cost between waypoints
* Supports three heuristic modes:

  * Mode 1: Manhattan distance
  * Mode 2: Euclidean distance
  * Mode 3: Chebyshev distance

The module compares how different heuristic choices affect the selected route.

---

### Task 3 — Warehouse Robot Navigation

* Models a warehouse as a 2D grid
* Uses `0` for open cells and `1` for blocked cells
* Allows movement in 8 directions:

  * Up, Down, Left, Right
  * Up-Left, Up-Right, Down-Left, Down-Right

Movement costs:

```text
Straight move  = 1.0
Diagonal move  = 1.5
```

The returned path stores the movement directions as linked-list nodes.

---

### Task 4 — Evacuation Route Planning

* Converts a floor-plan grid into a weighted adjacency matrix
* Searches for a safe path from a start location to an exit
* Supports Manhattan and Chebyshev heuristic modes
* Handles special cases such as:

  * unreachable exits
  * blocked start or target cells
  * start position already equal to the exit

Unlike Task 3, this task returns coordinate-based path nodes.

---

## Requirements

* C++17 compatible compiler
* Standard C++ libraries only
* No external packages required

Tested compile command:

```bash
g++ -std=c++17 -Wall -Wextra -pedantic -o main main.cpp Algo.cpp PathNode.cpp
```

---

## How to Run

### Compile

```bash
g++ -std=c++17 -Wall -Wextra -pedantic -o main main.cpp Algo.cpp PathNode.cpp
```

### Run

```bash
./main
```

The test scenarios are already written directly inside `main.cpp`, so no external input file is required.

---

## Output Format

Each discovered route is printed as a linked list of `PathNode` objects.

```text
Solution Path:
Node: <name> | f: <total_estimated_cost> | g: <actual_cost> | h: <heuristic_cost>
```

Where:

* `name` is the person ID, coordinate, or movement direction
* `g` is the actual accumulated cost from the start node
* `h` is the estimated cost to the goal
* `f = g + h`

If no valid route exists, the program prints:

```text
No path found.
```

---

## Key Algorithms

* A* search
* Reverse BFS heuristic construction
* Graph traversal
* Adjacency matrix representation
* Grid-to-graph conversion
* Linked-list path reconstruction
* Heuristic comparison using Manhattan, Euclidean, and Chebyshev distances

---

## Complexity

| Module                    | Main Idea                         | Complexity                              |
| ------------------------- | --------------------------------- | --------------------------------------- |
| Task 1 — Social path       | Reverse BFS + path reconstruction | O(V²) with adjacency matrix scanning    |
| Task 2 — Drone path        | A* over weighted graph            | O(V²) with linear frontier selection    |
| Task 3 — Warehouse path    | A* over grid cells                | O((mn)²) in the current vector frontier |
| Task 4 — Evacuation path   | Grid-to-graph + A* search         | O((mn)²) due to matrix-based scanning   |

`V` is the number of graph vertices, while `m × n` is the grid size.

---

## Limitations

* Fixed maximum size of 100 vertices or grid-related entries
* Linear search is used instead of a priority queue
* Test cases are hardcoded in `main.cpp`
* Some modules use adjacency matrices, which are less memory-efficient for sparse graphs
* The project focuses on algorithm demonstration rather than interactive user input

---

## Author

Hắc Minh Quân  
Student ID: 2550201

---

## Notes

This project demonstrates the ability to:

* Translate theoretical A* concepts into working C++ code
* Design reusable path nodes for multiple search problems
* Compare heuristic strategies across different environments
* Handle both successful and unreachable pathfinding scenarios
* Connect discrete structures theory with practical simulation problems

---
