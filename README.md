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

## 🎮 Interactive Actions: What You Can Do, What It Does & How It Works

When you launch `./quantum_route`, you enter an interactive command loop (`quantum-route>`) operating on a **35 × 21 grid (735 cells, indexed `0` to `734` where `index = row * 35 + col`)**. Below is every action you can perform, its exact visual/functional effect, and how the underlying C++20 data structures execute it.

### 1. Pathfinding, Benchmarking & Agent Navigation
| Action / Command | What You Can Do | What It Will Do (Visual & Output Effect) | How It Works Internally (`main.cpp`) |
| :--- | :--- | :--- | :--- |
| **`run`** | Trigger a full multi-algorithm benchmark and route calculation from the current agent cell to the goal. | 1. Prints DSU component count and reachability.<br/>2. Animates **Dijkstra** expansion, **A\*** expansion, and **Quantum-inspired superposition** on the ANSI grid.<br/>3. Prints a side-by-side table of execution time ($\mu\text{s}$), popped/expanded nodes, composite cost, distance, time, and risk.<br/>4. Displays the 12-path Quantum register table and locks in the winning route plan. | Calls `g.rebuildDSU()` and checks `g.dsu.connected(agent, goal)` in $O(\alpha(V))$. If disconnected, aborts immediately. Otherwise, runs `profile(false)` (Dijkstra) and `profile(true)` ($A^*$) 25 times each using a `std::priority_queue` min-heap to measure average microseconds, then invokes `quantumSelect()` (12 noisy $A^*$ runs + 48 Monte Carlo scenarios + Grover diffusion). |
| **`step`** | Move the autonomous delivery agent forward by **1 grid cell** along the optimal path. | Moves `@` (Agent) to the next cell on the planned route, adds that step's composite cost to the cumulative `travelled` total, redraws the grid, and prints `DELIVERED!` when the agent reaches `G` (Goal). | Checks if `planVer != ver` (i.e., whether obstacles, traffic, risk, or weights changed since the last plan). If stale, automatically calls `replan()` to verify DSU connectivity and run a fresh $A^*$ search from the agent's current cell before stepping to `plan[1]`. |
| **`live [n]`**<br/>*(e.g., `live 50`)* | Run a continuous, real-time dynamic traffic simulation for `n` ticks (default `15`, max `500`). | Every tick, traffic fluctuates randomly across the map, the route dynamically adapts around new jams in real time, and the agent advances 1 step per frame until delivery (`DELIVERED under live traffic`) or disconnection. | In each loop iteration $k \in [0, n)$, selects 8 random non-wall cells and perturbs their traffic factor in $[1.0, 8.0]$ via `g.traffic.set()` in $O(\log V)$. Increments world version `touch()`, triggers `replan()` ($A^*$), advances `agent = plan[1]`, and renders the frame in-place using ANSI cursor-up escape codes. |
| **`reset`** | Reset the agent back to the starting coordinates without losing map edits. | Moves `@` back to `S` (Start), clears the active route overlay, resets cumulative travelled cost to `0.00`, and redraws the grid. | Sets `agent = start`, `plan.clear()`, `travelled = 0`, increments `ver` via `touch()`, and calls `show("agent reset")`. |

---

### 2. Dynamic Topology & Obstacle Manipulation (DSU-Backed)
| Action / Command | What You Can Do | What It Will Do (Visual & Output Effect) | How It Works Internally (`main.cpp`) |
| :--- | :--- | :--- | :--- |
| **`wall <idx>`**<br/>**`unwall <idx>`**<br/>*(e.g., `wall 37`)* | Turn any open cell `idx` (`0..734`, except Agent/Goal) into an impassable obstacle (`XX` / `##`), or clear an existing wall. | Immediately places or removes a red blocked cell on the grid, prints the updated number of DSU connected components, and invalidates any existing route passing through that cell. | Sets `g.wall[idx] = 1` (or `0`), calls `g.rebuildDSU()` which resets 735 singleton sets and unites all valid adjacent non-wall cell pairs using union-by-rank and path compression, and increments `ver` so the next `step` or `run` recalculates the path. |
| **`block <u> <v>`**<br/>**`unblock <u> <v>`**<br/>*(e.g., `block 36 37`)* | Sever or restore the specific road connection (edge) between two adjacent cells `u` and `v` without blocking the cells themselves. | Cuts the link between `u` and `v`, prints `Edge u-v BLOCKED`, displays the new DSU component count, and states whether `connected(agent, goal)` is `true` or `false`. | Validates that `u` and `v` are adjacent ($\max(\vert{}dx\vert{}, \vert{}dy\vert{}) = 1$). Computes a canonical 64-bit undirected edge key $\min(u,v) \cdot 735 + \max(u,v)$ and inserts/erases it in `std::unordered_set<long long> cutEdges` in $O(1)$ expected time. Rebuilds DSU so neighbor generator `eachNb()` ignores that edge. |
| **`src <idx>`**<br/>**`dst <idx>`**<br/>*(e.g., `dst 700`)* | Relocate the starting point (`S` / `@`) or destination goal (`G`) to any valid non-wall cell `0..734`. | Moves `S` or `G` on the terminal map, clears the previous route plan, and prepares the simulator to route between the new endpoints. | Validates `!g.wall[idx]`, updates `start = agent = idx` (or `goal = idx`), clears `plan`, increments `ver`, and renders the updated grid. |

---

### 3. Real-Time Traffic Congestion & Segment Tree Analytics
| Action / Command | What You Can Do | What It Will Do (Visual & Output Effect) | How It Works Internally (`main.cpp`) |
| :--- | :--- | :--- | :--- |
| **`traffic <idx> <factor>`**<br/>*(e.g., `traffic 108 6.5`)* | Set the traffic congestion multiplier (`0.1` to `100.0`) on a single cell `idx`. | Updates the cell's color intensity on the grid (orange/red for heavy traffic `~`), prints the point update confirmation, and outputs the global `Network delay index`. | Executes `g.traffic.set(idx, factor)` on the iterative Segment Tree: writes `factor` to leaf `idx + N` and bit-shifts up the tree (`i >>= 1`) calling `pull(i)` to update parent sums and maxima in $O(\log 735)$ time. |
| **`traffic <u> <v> <factor>`**<br/>*(e.g., `traffic 36 37 4.5`)* | Set the traffic congestion multiplier (`0.1` to `100.0`) across a road segment connecting adjacent cells `u` and `v`. | Updates both endpoint cells in the Segment Tree, prints `O(log 735)` confirmation, and recalculates the total network delay index. | Verifies adjacency between `u` and `v`, performs two $O(\log V)$ Segment Tree point updates (`setf(u, f)` and `setf(v, f)`), and queries `g.traffic.sum(0, 735) - 735` for the global network delay. |
| **`query <l> <r>`**<br/>*(e.g., `query 35 40`)* | Perform an instant statistical traffic audit over any contiguous range of cell indices `[l, r]`. | Prints the **Total Delay** ($\sum f_i - \text{len}$), **Average Traffic Factor**, and **Maximum Bottleneck Factor** across cells `l` through `r`. | Calls `g.traffic.sum(l, r + 1)` and `g.traffic.max(l, r + 1)` on the iterative Segment Tree over the half-open interval $[l, r+1)$ in $O(\log V)$ time without scanning individual cells. |
| **`risk <idx> <value>`**<br/>*(e.g., `risk 142 15`)* | Assign an environmental hazard rating (`0.0` to `20.0`) to cell `idx`. | Marks the cell with a magenta `!` hazard indicator on the map (`!` if both risky and congested) and penalizes routes that enter cell `idx`. | Stores `g.risk[idx] = static_cast<float>(value)` in the 1D `std::vector<float> risk` array ($O(1)$) and increments the world version `ver`. |

---

### 4. Multi-Criteria Weights, Heuristics, Presets & Quantum Tuning
| Action / Command | What You Can Do | What It Will Do (Visual & Output Effect) | How It Works Internally (`main.cpp`) |
| :--- | :--- | :--- | :--- |
| **`weights <wd> <wt> <wr>`**<br/>*(e.g., `weights 1 3 5`)* | Change the optimization priority between **Distance** ($w_d$), **Travel Time/Traffic** ($w_t$), and **Hazard Risk** ($w_r$). | Shifts route selection behavior (e.g.,

---

##  1. Project Overview

Conventional shortest-path routines typically optimize a single static edge weight (such as geometric distance). In real-world navigation, logistics, emergency response, and robotics, the shortest geometric path is rarely optimal when **traffic congestion**, **environmental hazards**, and **dynamic roadblocks** are present.

The **Quantum Route Pathfinding Simulator** addresses this by modeling a changing **35 × 21 grid (735 cells)** as an implicit weighted graph where an autonomous agent navigates from a source cell to a destination cell under multi-criteria weights:

* **Multi-Criteria Composite Routing:** Balances geometric step distance ($d \in \{1, \sqrt{2}\}$), per-cell traffic congestion factors ($f_j$), and environmental hazard ratings ($\text{risk}_j$) controlled by user-tunable weights $(w_{\text{dist}}, w_{\text{time}}, w_{\text{risk}})$.
* **Instant Disconnected-Graph Rejection via DSU:** Uses a **Disjoint Set Union (DSU)** structure with path compression and union-by-rank to verify source-to-goal reachability in $O(\alpha(V))$ time before launching any graph search.
* **Real-Time Traffic Analytics via Segment Tree:** Maintains per-cell traffic multipliers in an **iterative array-based Segment Tree**, supporting $O(\log V)$ point updates, range-sum delay queries, and range-maximum bottleneck queries.
* **Admissible A* vs. Dijkstra Profiling:** Implements a unified priority-queue search engine that benchmarks Dijkstra against $A^*$ (averaging over 25 runs in microseconds) using admissible lower-bound heuristics (Manhattan, Chebyshev, or Euclidean) with strict diagonal corner-cutting prevention.
* **Quantum-Inspired Monte Carlo Route Selection:** Generates up to 12 diverse $A^*$ candidate trajectories, evaluates their robustness across **48 randomized log-normal Monte Carlo traffic scenarios**, and applies **Grover-style phase inversion and amplitude diffusion** to select the most resilient route. *(Note: Implemented via classical floating-point arithmetic as an algorithmic analogy—no qubits or quantum hardware are used.)*

---

##  2. System Architecture & Visual Flowcharts

### 2.1 Overall System Architecture
The command-line user interacts with the `Sim` controller through textual commands. `Sim` owns the `World` state, cost weights, `mt19937` random generator, agent coordinates, and active route plan.

```mermaid
flowchart LR
    User([" Command-Line User<br/>(commands / settings)"]) -->|input| Sim["⚙️ Command Interpreter<br/>Sim Controller"]
    Sim -->|updates / queries| World["🌐 World / Grid Graph<br/>walls • traffic • risk"]
    World -->|graph links| DSU["🔗 DSU Connectivity<br/>Pre-Check O(α(V))"]
    World -->|traffic values| SegTree[" Segment Tree<br/>Traffic Analytics O(log V)"]
    Sim -->|run / replan| Search[" Dijkstra + A*<br/>Min-Priority Queue"]
    Search -->|search trace| Renderer[" Terminal Renderer<br/>ANSI Route + Statistics"]
    Search -->|candidate paths| Quantum["⚛️ Candidate Evaluation<br/>48 Monte Carlo + Q-Inspired"]
    Quantum -->|selected route| Renderer
    Sim -->|show / step / live| Renderer
