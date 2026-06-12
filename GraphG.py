import math
from collections import defaultdict

import networkx as nx

class GraphG:
    def __init__(self, mapAndDim, goalsLoc):
        self.rows = mapAndDim["Rows"]
        self.cols = mapAndDim["Cols"]
        self.grid = mapAndDim["Map"]  # 0 = free, 1 = blocked
        self.goalsLoc = goalsLoc
        self.G = self.build_graph()
        self.dictDistance = defaultdict(lambda: math.inf)

    def build_graph(self) -> nx.Graph:
        G = nx.Graph()
        for idx in range(self.rows * self.cols):
            if self.grid[idx] == 0:
                G.add_node(idx)

        for idx in G.nodes:
            r, c = divmod(idx, self.cols)
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    nidx = nr * self.cols + nc
                    if self.grid[nidx] == 0:
                        G.add_edge(idx, nidx)
        return G

    def CalcAllDistancesFromGoals(self):
        for goal in self.goalsLoc:
            distDictFromGoal = nx.single_source_shortest_path_length(self.G, goal)
            for loc, dist in distDictFromGoal.items():
                self.dictDistance[(loc, goal)] = dist


    def all_goals_reachable_by_at_least_one_agent(self, active_agentsLoc, inactive_agentsLoc):
        G_without_inactive_agents = self.G.copy()
        G_without_inactive_agents.remove_nodes_from(inactive_agentsLoc)
        for goal in self.goalsLoc:
            reachable = nx.node_connected_component(G_without_inactive_agents, goal)
            if not reachable.intersection(active_agentsLoc):
                return False

        return True


