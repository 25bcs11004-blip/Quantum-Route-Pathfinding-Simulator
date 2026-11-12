# Quantum Route Pathfinding Simulator

**Dynamic Multi-Criteria Grid Pathfinding System Comparing Classical Graph Search (**$\text{Dijkstra}$ **&** $A^{*}$**) Against Grover-Inspired Monte Carlo Candidate Route Selection**

Overview •
[Architecture](#-system-architecture--design) •
[Mathematical Model](#-mathematical-formulation--quantum-inspired-pipeline) •
[Flowcharts](#-algorithmic-flowcharts) •
[Benchmarks](#-experimental-benchmarks--visualizations-seed-2024) •
[Quick Start](#-quick-start--build-instructions) •
[CLI Reference](#-interactive-cli-command-reference)

## 📌 Project Overview

The **Quantum Route Pathfinding Simulator** is a self-contained $\text{C++20}$ command-line application that models a dynamically changing two-dimensional environment as an **implicit weighted grid graph** ($35 \times 21 = 735$ indexed cells). An autonomous agent must navigate from a source cell to a destination while avoiding blocked cells and severed edges, simultaneously optimizing for **movement distance**, **traffic congestion**, and **environmental hazard risk**.

### ✨ Key Technical Highlights

* **Multi-Criteria Composite Edge Costing:** Evaluates step costs dynamically using user-tunable priority weights $(w_{\text{dist}}, w_{\text{time}}, w_{\text{risk}})$ across 4-direction ($\text{Manhattan}$) and 8-direction ($\text{Chebyshev}$ / $\text{Euclidean}$) movement policies with strict diagonal corner-cutting prevention.

* $\mathcal{O}(\alpha(V))$ **Connectivity Pre-Check via DSU:** Utilizes a **Disjoint Set Union (DSU)** structure with path compression and union-by-rank to verify source-to-goal reachability in near-constant time before attempting expensive priority-queue graph traversals.

* $\mathcal{O}(\log V)$ **Real-Time Traffic Analytics via Segment Tree:** Maintains per-cell traffic congestion factors in an **iterative array-based Segment Tree** ($2N$ nodes) supporting logarithmic point updates, range-sum delay queries, and range-maximum bottleneck queries.

* **Classical Grover-Inspired Candidate Selection:** Generates up to $M \le 12$ unique candidate trajectories via perturbed $A^{*}$ searches, evaluates route robustness across $S = 48$ log-normal **Monte Carlo** traffic scenarios, and applies classical **Grover phase inversion and mean diffusion** arithmetic to collapse onto the most resilient route. *(Note: All computations use classical floating-point vectors as an educational mathematical analogy; no qubits or quantum hardware are used).*

* **Live Dynamic Replanning & ANSI Terminal Animation:** Tracks world topology/weight mutations via a monotonic version counter, automatically triggering $A^{*}$ replanning during step-by-step navigation or multi-tick live traffic fluctuations.

## 🏗️ System Architecture & Design

### 1. Overall System Architecture

The simulator is organized around the `Sim` controller, which orchestrates user CLI commands, the `World` implicit graph state, `DSU` connectivity verification, `SegTree` traffic analytics, the shared `Dijkstra` / $A^{*}$ search engine, the `quantumSelect` Monte Carlo evaluator, and the ANSI terminal `Overlay` renderer.

```
flowchart LR
    User["Command-Line User\n(Commands / Settings)"] -->|input| Sim["Command Interpreter\nSim Controller"]
    Sim -->|updates / queries| World["World / Grid Graph\nWalls • Traffic • Risk"]
    World -->|graph links| DSU["DSU Connectivity\nPre-Check O(α(V))"]
    World -->|traffic values| SegTree["Iterative Segment Tree\nTraffic Analytics O(log V)"]
    Sim -->|run / replan| Search["Dijkstra + A*\nMin-Priority Queue"]
    Search -->|search trace| Renderer["Terminal Renderer\nANSI Route + Statistics"]
    Search -->|candidate paths| Quantum["Candidate Evaluation\n48x Monte Carlo + Grover"]
    Quantum -->|selected route| Renderer
    Sim -->|show / step / live| Renderer

    style User fill:#1e293b,stroke:#3b82f6,stroke-width:2px,color:#f8fafc
    style Sim fill:#1e293b,stroke:#3b82f6,stroke-width:2px,color:#f8fafc
    style World fill:#1e293b,stroke:#60a5fa,stroke-width:2px,color:#f8fafc
    style DSU fill:#1e1b4b,stroke:#8b5cf6,stroke-width:2px,color:#f8fafc
    style SegTree fill:#1e1b4b,stroke:#8b5cf6,stroke-width:2px,color:#f8fafc
    style Search fill:#0f172a,stroke:#10b981,stroke-width:2px,color:#f8fafc
    style Quantum fill:#31102f,stroke:#ec4899,stroke-width:2px,color:#f8fafc
    style Renderer fill:#1e293b,stroke:#f59e0b,stroke-width:2px,color:#f8fafc

```

### 2. Row-Major Implicit Grid Representation

Rather than storing an explicit adjacency list, the $35 \times 21$ grid maps each 2D coordinate $(x, y)$ to a 1D index $i = y \times 35 + x \in [0, 734]$. Legal neighbors are generated on the fly using 4-directional or 8-directional offset arrays.

```
flowchart LR
    subgraph Row_Progression ["1D Row-Major Memory Layout (N = 735)"]
        R0["Row 0\nIndices 0 – 34"] -->|next row| R1["Row 1\nIndices 35 – 69"]
        R1 -->|next row| R2["Row 2\nIndices 70 – 104"]
    end
    subgraph Coordinate_Mapping ["Implicit Graph Coordinate Mapping"]
        Map["Cell (x, y)\nIndex = y × 35 + x"] -->|mapping| Grid["35 Columns × 21 Rows\n735 Total Cells"]
        Grid -->|default endpoints| Endpoints["Start = 36 (x=1, y=1)\nGoal = 698 (x=33, y=19)"]
    end

```

## 🧮 Mathematical Formulation & Quantum-Inspired Pipeline

### 1. Composite Multi-Criteria Step Cost

Entering a target cell $j$ with geometric step distance $d \in \{1, \sqrt{2}\}$ incurs a composite cost:

$$
\text{StepCost}(j, d) = w_{\text{dist}} \cdot d + w_{\text{time}} \cdot d \cdot f_j \cdot \eta_j + w_{\text{risk}} \cdot \text{risk}_j
$$

where:

* $d = 1.0$ for orthogonal moves and $d = \sqrt{2} \approx 1.4142$ for diagonal moves (when `dir8 on` is enabled and neither adjacent orthogonal cell is a wall).

* $f_j \in [0.1, 100.0]$ is the baseline traffic factor stored at leaf $j$ of the Segment Tree.

* $\eta_j = \exp(\sigma Z)$ with $Z \sim \mathcal{N}(0, 1)$ is multiplicative log-normal traffic noise applied during stochastic candidate generation and Monte Carlo evaluation ($\eta_j = 1.0$ during deterministic search).

* $\text{risk}_j \in [0, 20]$ is the environmental hazard rating of cell $j$.

### 2. Admissible $A^{*}$ Heuristic Scaling

To guarantee that $A^{*}$ never overestimates the remaining cost to the goal $(t_x, t_y)$, the simulator scans all open cells to find the minimum possible cost per unit distance $u_{\min}$:

$$
u_{\min} = \min_{i \in \text{Open}} \left( w_{\text{dist}} + w_{\text{time}} \cdot f_i \cdot \eta_i + \frac{w_{\text{risk}} \cdot \text{risk}_i}{\sqrt{2}} \right)
$$

$$
h(v) = d_{\text{geom}}(v, t) \times u_{\min}
$$

where $d_{\text{geom}}(v, t)$ is **Manhattan** distance ($\Delta x + \Delta y$) in 4-dir mode, **Chebyshev** distance ($\max(\Delta x, \Delta y)$) in 8-dir mode, or **Euclidean** distance ($\sqrt{\Delta x^2 + \Delta y^2}$) when `heur euclid` is active.

### 3. Grover-Inspired Candidate Route Selection

1. **Candidate Superposition (**$N_C = 12$**):** Executes 1 deterministic baseline $A^{*}$ search and 11 noisy $A^{*}$ searches ($\sigma_{\text{gen}} = 1.8\sigma$), deduplicating identical trajectories to yield $M \le 12$ distinct candidate routes.

2. **Monte Carlo Evaluation (**$N_S = 48$**):** Evaluates every candidate route $i \in \{0, \dots, M-1\}$ across 48 independent log-normal traffic realizations $\eta = \exp(\sigma Z)$, computing sample mean $\mu_i$, standard deviation $\sigma_i$, and risk-adjusted score:
   

   $$
   \text{Score}_i = \mu_i + 0.5\sigma_i
   $$

3. **Uniform Amplitude Initialization:** Initializes classical floating-point amplitudes uniformly across the $M$ unique candidates:
   

   $$
   a_i^{(0)} = \frac{1}{\sqrt{M}}
   $$

4. **Grover Oracle & Mean Diffusion:** Identifies the oracle winner $i^{*} = \arg\min_i \text{Score}_i$. For $K = \max\left(1, \text{round}\left(\frac{\pi}{4}\sqrt{M}\right)\right)$ iterations, applies phase inversion followed by inversion about the mean $\bar{a}$:
   

   $$
   a_{i^{*}} \leftarrow -a_{i^{*}}, \qquad \bar{a} = \frac{1}{M}\sum_{j=0}^{M-1} a_j, \qquad a_i \leftarrow 2\bar{a} - a_i
   $$

5. **Measurement Collapse:** Normalizes squared amplitudes into selection probabilities $p_i = \frac{a_i^2}{\sum_j a_j^2}$ and collapses onto $\arg\max_i p_i$.

## 🔄 Algorithmic Flowcharts

```
flowchart TD
    Start(["START"]) --> Seed["Read optional seed;\nconstruct simulator"]
    Seed --> InitCity["Generate default city;\nrebuild DSU"]
    InitCity --> ReadCmd[/"Read command from\nterminal"/]
    ReadCmd --> Valid{"Command valid?"}
    Valid -->|no| ReadCmd
    Valid -->|yes| Exec["Execute matching\ncommand handler"]
    Exec --> Quit{"Quit requested?"}
    Quit -->|no| ReadCmd
    Quit -->|yes| EndNode(["END"])

```

## 📊 Data Structures & Complexity Analysis

Let $V = 735$ cells ($35 \times 21$), $E = \mathcal{O}(V)$ bounded-degree grid edges, $M \le 12$ unique candidate paths of length $L$, and $S = 48$ Monte Carlo scenarios.

| Data Structure / Stage | Project Role | Core Operation | Time Complexity | Space Complexity | 
| ----- | ----- | ----- | ----- | ----- | 
| **`std::vector`** | Implicit grid state, $g$-scores, `parent[]`, `closed[]` | Direct index lookup | $\mathcal{O}(1)$ | $\mathcal{O}(V)$ | 
| **Implicit Grid (`eachNb`)** | On-demand 4-dir / 8-dir neighbor & corner-cut filter | Neighbor expansion | $\mathcal{O}(1)$ per cell | $\mathcal{O}(V)$ | 
| **`std::unordered_set`** | Order-independent blocked edges (`cutEdges`) | Edge cut insert / lookup | $\mathcal{O}(1)$ expected | $\mathcal{O}(B)$ | 
| **Disjoint Set Union (`DSU`)** | Pre-search source-to-goal reachability check | `find` / `unite` | $\mathcal{O}(\alpha(V))$ | $\mathcal{O}(V)$ | 
| **Iterative `SegTree`** | Per-cell traffic factors, network delay & range queries | `set` / `sum` / `max` | $\mathcal{O}(\log V)$ | $\mathcal{O}(V)$ | 
| **Min-Heap (`priority_queue`)** | Frontier ordering for $\text{Dijkstra}$ ($g$) and $A^{*}$ ($g+h$) | Push / Pop unsettled node | $\mathcal{O}((V+E)\log V)$ | $\mathcal{O}(E)$ | 
| **Monte Carlo + Grover Stage** | 12x $A^{*}$ generation, 48x scenario scoring, diffusion | Candidate selection | $\mathcal{O}(12 V \log V + S M L + M\sqrt{M})$ | $\mathcal{O}(M \cdot L)$ | 

## 🧪 Experimental Benchmarks & Visualizations (Seed `2024`)

All benchmarks were verified with **GCC 14.2.0** (`g++ -std=c++20 -O2 main.cpp -o quantum_route`) using deterministic seed `2024` across all four synthetic terrain presets (`city`, `maze`, `field`, `disaster`).

### 1. Preset Performance Comparison Table

| Environment Preset | Dijkstra Time ($\mu\text{s}$) | $A^{*}$ Time ($\mu\text{s}$) | Quantum Collapse ($\mu\text{s}$) | Dijkstra Expanded | $A^{*}$ Expanded | Optimal Composite Cost | $A^{*}$ Node Reduction | Superposed Paths ($M$) | 
| ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | 
| **🏙️ City** | $7.9\ \mu\text{s}$ | $5.9\ \mu\text{s}$ | $1386.1\ \mu\text{s}$ | $295$ | $164$ | $110.00$ | $-44.4\%$ | $3$ ($1\text{ iter}$) | 
| **🧩 Maze** | $6.1\ \mu\text{s}$ | $7.5\ \mu\text{s}$ | $1447.4\ \mu\text{s}$ | $239$ | $236$ | $231.00$ | $-1.3\%$ | $8$ ($2\text{ iters}$) | 
| **🌾 Field** | $17.7\ \mu\text{s}$ | $7.2\ \mu\text{s}$ | $1763.4\ \mu\text{s}$ | $584$ | $147$ | $100.00$ | $-74.8\%$ | $12$ ($3\text{ iters}$) | 
| **🌋 Disaster** | $16.0\ \mu\text{s}$ | $12.3\ \mu\text{s}$ | $1500.8\ \mu\text{s}$ | $473$ | $352$ | $167.27$ | $-25.6\%$ | $12$ ($3\text{ iters}$) | 

> **Key Observation:** Across all four presets, $A^{*}$ with our scaled lower-bound heuristic achieved **100% optimal composite cost agreement** with $\text{Dijkstra}$ while reducing expanded nodes by up to **74.8%** in open terrain (`field`) and **44.4%** in urban corridors (`city`).

### 2. Captured Terminal Outputs

## 🚀 Quick Start & Build Instructions

### Prerequisites

* Any $\text{C++20}$ **compatible compiler** (`GCC 10+`, `Clang 11+`, or `MSVC 19.29+`).

* **Zero external libraries** required (100% standard C++ STL).

* An ANSI-capable terminal (Windows Terminal, PowerShell, macOS Terminal, or Linux bash) for color-coded overlays.

### 1. Clone the Repository

```
git clone https://github.com/25bcs11004-blip/Quantum-Route-Pathfinding-Simulator.git
cd Quantum-Route-Pathfinding-Simulator

```

### 2. Compile the Source Code

```
g++ -std=c++20 -O2 QuantumRoute/main.cpp -o quantum_route

```

*(If `main.cpp` is in your root directory, run `g++ -std=c++20 -O2 main.cpp -o quantum_route`).*

### 3. Run the Simulator

```
# Launch with default deterministic seed (2024)
./quantum_route

# Or specify a custom RNG seed
./quantum_route 42

```

## 💻 Interactive CLI Command Reference

Once inside the `quantum-route>` prompt, use the following commands to manipulate the world, query data structures, and benchmark algorithms:

| Category | Command Syntax | Description & Complexity | 
| ----- | ----- | ----- | 
| **Execution** | `run` | Rebuilds DSU, checks connectivity, profiles $\text{Dijkstra}$ vs. $A^{*}$ (25x avg), runs Quantum candidate collapse, and prints comparison tables. | 
| **Navigation** | `step` | Advances the agent 1 node along the active plan (triggers automatic $A^{*}$ replanning if the world version changed). | 
| **Live Sim** | `live [n]` | Runs `n` ticks (default `15`, max `500`) of fluctuating random traffic, replanning and moving the agent at each tick. | 
| **Topology** | `wall <idx>` | `unwall <idx>` | Blocks or frees cell `<idx>` ($0 \dots 734$), rebuilds DSU, and increments world version. | 
| **Edge Cuts** | `block <u> <v>` | `unblock <u> <v>` | Cuts or restores the edge between adjacent cells `u` and `v` in `cutEdges` ($\mathcal{O}(1)$ hash lookup) and rebuilds DSU. | 
| **Traffic** | `traffic <idx> <factor>` | Updates cell `<idx>` traffic factor ($0.1 \dots 100.0$) via Segment Tree point update in $\mathcal{O}(\log V)$. | 
| **Edge Traffic** | `traffic <u> <v> <factor>` | Updates traffic congestion on both endpoints of adjacent edge `u-v` in $\mathcal{O}(\log V)$. | 
| **Analytics** | `query <l> <r>` | Executes an inclusive Segment Tree range query on cells $[l, r]$ returning **total delay**, **average factor**, and **max factor** in $\mathcal{O}(\log V)$. | 
| **Risk** | `risk <idx> <value>` | Sets environmental hazard level ($0 \dots 20$) on cell `<idx>`. | 
| **Priorities** | `weights <w_dist> <w_time> <w_risk>` | Adjusts multi-criteria routing weights (must be $\ge 0$ and not all zero). | 
| **Presets** | `preset city|maze|field|disaster` | Generates a new synthetic environment and ensures source-to-goal connectivity. | 
| **Endpoints** | `src <idx>` | `dst <idx>` | Relocates the start/agent cell or destination goal cell. | 
| **Movement** | `dir8 on|off` | Toggles between 4-directional ($\text{Manhattan}$) and 8-directional ($\text{Chebyshev}$, diagonal cost $\sqrt{2}$) movement. | 
| **Heuristic** | `heur manh|euclid` | Switches $A^{*}$ geometric distance between Manhattan/Chebyshev and Euclidean distance. | 
| **Quantum** | `quantum on|off` | `sigma <0..1>` | Enables/disables the Grover-inspired Monte Carlo stage or tunes traffic noise $\sigma$ (default `0.3`). | 
| **Display** | `anim on|off|<ms>` | `color on|off` | Toggles live search animations, sets frame delay in ms, or switches to plain ASCII mode. | 
| **System** | `show` | `reset` | `help` | `quit` | Redraws the current grid, resets agent to start, displays command help, or exits. | 

## 📂 Recommended Repository Structure

To make all images in this `README.md` render automatically on GitHub, upload the PNG files from the **Visual Assets Studio** into an `assets/` folder in the root of your repository:

```
Quantum-Route-Pathfinding-Simulator/
├── QuantumRoute/
│   └── main.cpp                                    # Complete C++20 Simulator Source (610 lines)
├── assets/
│   ├── 00_github_repo_banner.png                   # Repository Header Banner
│   ├── 01_system_architecture.png                  # Figure 5.1: Overall System Architecture
│   ├── 02_row_major_grid_representation.png        # Figure 5.2: Row-Major Grid Representation
│   ├── 03_program_execution_flowchart.png          # Flowchart 9.1: Overall Program Execution
│   ├── 04_dsu_connectivity_flowchart.png           # Flowchart 9.4: DSU Connectivity Verification
│   ├── 05_environment_generation_flowchart.png     # Flowchart 9.3: Environment Generation
│   ├── 06_benchmark_output_seed2024.png            # Figure 10.1: Captured Preset Benchmarks
│   ├── 07_city_preset_cli_grid.png                 # Figure 10.2: City Preset CLI Grid
│   ├── 08_dijkstra_terminal_visualization.png      # Figure 10.3: Dijkstra Search Visualization
│   ├── 09_live_traffic_simulation_output.png       # Figure 10.4: Live Traffic Simulation Output
│   ├── 10_command_tests_dsu_segtree.png            # Figure 10.5: DSU & Segment Tree Command Tests
│   └── 11_quantum_grover_pipeline.png              # Quantum-Inspired Candidate Selection Pipeline
├── docs/
│   └── ADS_Project_Report.pdf                      # Full 40-Page Academic Project Report
└── README.md                                       # Repository Documentation

```

## 👥 Authors & Academic Acknowledgement

Submitted in partial fulfillment of the requirements for **Advanced Data Structures (ADS)** at the **University Institute of Engineering (UIE), Chandigarh University**:

* **Ansh** — *UID: 25BCS11004* (Section: 802-A)

* **Aryan Vaid** — *UID: 25BCS10791* (Section: 802-A)

* **Faculty Supervisor:** **Md. Tauseef Sir**, Department of UIE, Chandigarh University

## 📚 References

1. **Cormen, T. H., Leiserson, C. E., Rivest, R. L., & Stein, C.** (2022). *Introduction to Algorithms* (4th ed.). MIT Press.

2. **Hart, P. E., Nilsson, N. J., & Raphael, B.** (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.

3. **C++ Standard Library Reference** (`C++20`): <https://en.cppreference.com/>