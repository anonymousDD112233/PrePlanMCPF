# Agent Exclusion Strategies for Multi-Agent Combinatorial Path Finding

This repository contains the official code accompanying our **AAAI 2027** submission:  
> **Agent Exclusion Strategies for Multi-Agent Combinatorial Path Finding**

This paper examines whether all available agents should participate in Multi-Agent Combinatorial Path Finding (MCPF) under stochastic execution delays. 
It introduces PAES, a setting in which agents may be excluded before planning, and SEER, an adaptive method that selects which agents to exclude using both delay probabilities and travel distances to improve solvability, runtime, and service quality.

This repository includes full source code, benchmark data, and experiment scripts to enable **full reproducibility** of the results presented in the paper.

---

## Repository Structure

- **Agent_Goal_locations_files/** – Agent and goal locations files for experiments.  
- **ExperimentalResults/** – Processed experimental results from all experiments.  
- **Maps/** – Benchmark maps used in experiments.
- **FindConflict.py** – Detects conflicts between agents’ paths.  
- **GenerateInstances.py** – Generates agent and goal locations for maps.
- **GraphG.py** – Builds the grid graph from a map.
- **HeuristicAllocation.py** – Generates k-best agent-to-goal sequence allocations via greedy enumeration.
- **LowLevelPlan.py** – Computes individual agent paths under constraints.  
- **NodeStateClasses.py** – Defines data structures for nodes, states, and constraints.  
- **Robust_Planner.py** – Main planner implementation (Robust CBSS under SST).
- **RunTests.py** – Experiment executions reported in the paper.  
- **Run_Simulation.py** – Runs the online execution (simulation of plan execution).  
- **Verify.py** – Verifies solution robustness using simulations.

---

## Requirements & Installation

### Python
- **Python** ≥ 3.10  
- Recommended: **Ubuntu 20.04+** (tested on Ubuntu 24.04, AMD EPYC 7702P, 16 cores)  

---

## Randomization & Seeds
All randomized components are initialized with fixed seeds for reproducibility:
- **Verify.py**: `seed = 47`
- **FindConflict.py**: `seed = 42`
- **Run_Simulation.py**: `seed = 44`

Other components are fully deterministic given identical inputs.
