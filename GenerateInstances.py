import os
import random
import networkx as nx


def read_map_file(file_path):
    with open("OurResearch.domain/" + file_path, 'r') as f:
        lines = f.readlines()

    map_start_index = lines.index("map\n") + 1
    map_lines = lines[map_start_index:]

    currMap = []
    rows, cols = 0, 0
    for line in map_lines:
        cols = len(line.strip())
        rows += 1
        for char in line.strip():
            currMap += [0] if char == "." else [1]
    print(rows, cols, flush=True)
    return {"Rows": rows, "Cols": cols, "Map": currMap}


def build_free_cell_graph(mapAndDim):
    G = nx.Graph()
    rows, cols = mapAndDim["Rows"], mapAndDim["Cols"]
    for idx in range(rows * cols):
        if mapAndDim["Map"][idx] == 0:
            G.add_node(idx)
    for idx in G.nodes:
        r, c = divmod(idx, cols)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                nidx = nr * cols + nc
                if mapAndDim["Map"][nidx] == 0:
                    G.add_edge(idx, nidx)
    return G


def create_locations_for_agents_And_Goals(NumAgents, NumGoals, MapAndDim, G_template,
                                          max_attempts=100000):
    free_cells = [i for i, v in enumerate(MapAndDim["Map"]) if v == 0]
    for _ in range(max_attempts):
        chosen_agents = random.sample(free_cells, NumAgents)
        S = set(chosen_agents)

        H = G_template.copy()
        H.remove_nodes_from(S)

        if (H.number_of_nodes() > 0 and nx.is_connected(H)
                and all(any(nb not in S for nb in G_template.neighbors(v)) for v in S)):
            chosen_goals = random.sample(list(set(free_cells) - S), NumGoals)
            return chosen_agents, chosen_goals

    raise RuntimeError(f"Failed to place {NumAgents} agents after {max_attempts} attempts")


SEED = 42
random.seed(SEED)

mapsList = ["random-32-32-20.map", "room-32-32-4.map", "maze-32-32-2.map", "ht_chantry.map", "lak303d.map", "den520d.map"]
output_dir = "Agent_Goal_locations_files"
os.makedirs(output_dir, exist_ok=True)

for map_name in mapsList:
    mapAndDim = read_map_file(map_name)
    G_template = build_free_cell_graph(mapAndDim)
    print(f"{map_name}: {G_template.number_of_nodes()} free cells", flush=True)

    for instance in range(1, 501):
        if instance % 100 == 0:
            print(f"Instance {instance}", flush=True)
        AgentsPositions, GoalsLocations = create_locations_for_agents_And_Goals(
            150, 300, mapAndDim, G_template)

        base = map_name.split('.')[0]
        agents_file = os.path.join(output_dir, f"{base}_Map_Agent_Locs_instance_{instance}.txt")
        goals_file = os.path.join(output_dir, f"{base}_Map_Goal_Locs_instance_{instance}.txt")

        with open(agents_file, "w") as f:
            for item in AgentsPositions:
                f.write(f"{item}\n")

        with open(goals_file, "w") as f:
            for item in GoalsLocations:
                f.write(f"{item}\n")
