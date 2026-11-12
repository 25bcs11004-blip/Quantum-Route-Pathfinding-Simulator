<div align="center">

# ⚛️ Quantum Route Pathfinding Simulator
### Dynamic Multi-Criteria Grid Navigation • Dijkstra & A* • Grover-Inspired Monte Carlo Selection

[![Language](https://img.shields.io/badge/Language-C%2B%2B20-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/20)
[![Compiler](https://img.shields.io/badge/Compiler-GCC_14.2.0-4EAA25?style=for-the-badge&logo=gnu&logoColor=white)](https://gcc.gnu.org/)
[![Dependencies](https://img.shields.io/badge/Dependencies-C%2B%2B_STL_Only-238636?style=for-the-badge)](#4-technology-stack--system-requirements)
[![Grid](https://img.shields.io/badge/Grid-35%C3%9721_(735_Cells)-8957e5?style=for-the-badge)](#2-system-architecture--grid-design)
[![Course](https://img.shields.io/badge/Course-Advanced_Data_Structures-d29922?style=for-the-badge)](#10-academic-credits)

<p align="center">
  <b>A high-performance, zero-dependency C++20 command-line pathfinding laboratory that models a dynamic 2D weighted environment (distance, traffic congestion, and environmental risk) with O(α(V)) Disjoint Set Union (DSU) pre-checks, O(log V) iterative Segment Tree traffic analytics, and a classical Grover-style Monte Carlo trajectory selector.</b>
</p>

</div>

---

## 📌 Table of Contents
1. [Project Overview](#-1-project-overview)
2. [System Architecture & Visual Flowcharts](#-2-system-architecture--visual-flowcharts)
3. [Mathematical Model & Quantum-Inspired Selection](#-3-mathematical-model--quantum-inspired-selection)
4. [Advanced Data Structures & Complexity Analysis](#-4-advanced-data-structures--complexity-analysis)
5. [Technology Stack & Build Instructions](#-5-technology-stack--build-instructions)
6. [Interactive CLI Command Reference](#-6-interactive-cli-command-reference)
7. [Environment Presets](#-7-environment-presets)
8. [Experimental Benchmarks & Terminal Captures (Seed 2024)](#-8-experimental-benchmarks--terminal-captures-seed-2024)
9. [Academic Credits](#-9-academic-credits)

---

## 🚀 1. Project Overview

Conventional shortest-path routines typically optimize a single static edge weight (such as geometric distance). In real-world navigation, logistics, emergency response, and robotics, the shortest geometric path is rarely optimal when **traffic congestion**, **environmental hazards**, and **dynamic roadblocks** are present.

The **Quantum Route Pathfinding Simulator** addresses this by modeling a changing **35 × 21 grid (735 cells)** as an implicit weighted graph where an autonomous agent navigates from a source cell to a destination cell under multi-criteria weights:

* **Multi-Criteria Composite Routing:** Balances geometric step distance ($d \in \{1, \sqrt{2}\}$), per-cell traffic congestion factors ($f_j$), and environmental hazard ratings ($\text{risk}_j$) controlled by user-tunable weights $(w_{\text{dist}}, w_{\text{time}}, w_{\text{risk}})$.
* **Instant Disconnected-Graph Rejection via DSU:** Uses a **Disjoint Set Union (DSU)** structure with path compression and union-by-rank to verify source-to-goal reachability in $O(\alpha(V))$ time before launching any graph search.
* **Real-Time Traffic Analytics via Segment Tree:** Maintains per-cell traffic multipliers in an **iterative array-based Segment Tree**, supporting $O(\log V)$ point updates, range-sum delay queries, and range-maximum bottleneck queries.
* **Admissible A* vs. Dijkstra Profiling:** Implements a unified priority-queue search engine that benchmarks Dijkstra against $A^*$ (averaging over 25 runs in microseconds) using admissible lower-bound heuristics (Manhattan, Chebyshev, or Euclidean) with strict diagonal corner-cutting prevention.
* **Quantum-Inspired Monte Carlo Route Selection:** Generates up to 12 diverse $A^*$ candidate trajectories, evaluates their robustness across **48 randomized log-normal Monte Carlo traffic scenarios**, and applies **Grover-style phase inversion and amplitude diffusion** to select the most resilient route. *(Note: Implemented via classical floating-point arithmetic as an algorithmic analogy—no qubits or quantum hardware are used.)*

---

## 🏗️ 2. System Architecture & Visual Flowcharts

### 2.1 Overall System Architecture
The command-line user interacts with the `Sim` controller through textual commands. `Sim` owns the `World` state, cost weights, `mt19937` random generator, agent coordinates, and active route plan.

```mermaid
flowchart LR
    User(["👤 Command-Line User<br/>(commands / settings)"]) -->|input| Sim["⚙️ Command Interpreter<br/>Sim Controller"]
    Sim -->|updates / queries| World["🌐 World / Grid Graph<br/>walls • traffic • risk"]
    World -->|graph links| DSU["🔗 DSU Connectivity<br/>Pre-Check O(α(V))"]
    World -->|traffic values| SegTree["📊 Segment Tree<br/>Traffic Analytics O(log V)"]
    Sim -->|run / replan| Search["🔍 Dijkstra + A*<br/>Min-Priority Queue"]
    Search -->|search trace| Renderer["🖥️ Terminal Renderer<br/>ANSI Route + Statistics"]
    Search -->|candidate paths| Quantum["⚛️ Candidate Evaluation<br/>48 Monte Carlo + Q-Inspired"]
    Quantum -->|selected route| Renderer
    Sim -->|show / step / live| Renderer
