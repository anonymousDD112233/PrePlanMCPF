import math
import os
import random
import csv
import ast
import networkx as nx
import sys
import time

from GraphG import GraphG
from Robust_Planner import run_robust_planner_with_timeout
from Run_Simulation import Run_Simulation


def seed_from_range(key: str):
    start, _ = map(int, key.split("_"))
    return 10000 + start


####################################################### Create Map #################################################################################
def create_map(map_name):
    with open(f"OurResearch.domain/{map_name}.map", "r") as file:
        lines = file.readlines()
    map_lines = lines[lines.index("map\n") + 1:]
    currMap = [0 if char == "." else 1 for line in map_lines for char in line.strip()]
    rows, cols = len(map_lines), len(map_lines[0].strip())
    numOfFreeCells = currMap.count(0)
    return {"Rows": rows, "Cols": cols, "Map": currMap,
            "ObsRatio": 1 - numOfFreeCells / (rows * cols), "FreeCells": numOfFreeCells}


####################################################### Global Variables ######################################################################
mapName = sys.argv[1]
mapAndDim = create_map(mapName)
num_of_agents, goalsList = int(sys.argv[2]), [int(x) for x in sys.argv[3].split("-")]
instanceRange = sys.argv[4]
seedPerInstanceForDelay = seed_from_range(instanceRange)
howToRemove = sys.argv[5]
configStr = f"M={mapName}--A={num_of_agents}--G={sys.argv[3]}--I={instanceRange}--R={howToRemove}"

verifyAlpha = 0.05
max_planning_time = 60
desired_safe_prob = 0.7
max_instance_time = 300

####################################################### Write the header of a CSV file ############################################################
if not os.path.exists("Output_files"):
    os.makedirs("Output_files")

columns = ["Map", "Map Width", "Map Height", "Map Obstacle Ratio", "Desired Safe prob", "Number of agents",
           "Number of goals", "Number of removed agents", "How To Remove", "Agent Density", "Instance",
           "Offline Runtime", "Online Runtime", "Choose Remove Time", "Runtime",
           "SGAT", "Num Of Replan", "Online SST", "Offline SST", "Number of Expands", "Min Safe Prob"]

with open(f"Output_files/Output_{configStr}.csv", mode="w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=columns)
    writer.writeheader()


####################################################### Read locs from file #################################################################################
def read_locs_from_file(num_of_instance, num_of_goals):
    base = f"Agent_Goal_locations_files/{mapName.split('.')[0]}_Map"
    def read_first_n(path, n):
        with open(path, "r") as f:
            return [ast.literal_eval(line.strip()) for _, line in zip(range(n), f)]
    return (read_first_n(f"{base}_Agent_Locs_instance_{num_of_instance}.txt", num_of_agents),
            read_first_n(f"{base}_Goal_Locs_instance_{num_of_instance}.txt", num_of_goals))


def run_Test(AgentLocations, GoalLocations, DelaysProbDictExecution, InactiveAgents, GraphObj):
    randGen = random.Random(44)
    start_time = time.time()
    minSafeProb = math.inf

    ####################################### Offline Planning #############################################
    print("Calc Plan")
    p, OfflineTime, countExpand = run_robust_planner_with_timeout(
        AgentLocations, GoalLocations, desired_safe_prob, DelaysProbDictExecution, mapAndDim, verifyAlpha,
        max_planning_time, InactiveAgents, GraphObj)

    if p is None:
        print("Plan is None!\n--------------------------------------------------------------------------")
        return 60.0, None, None, None, None, None, countExpand, None, None

    plan_paths, Offline_SST, currSafeProb, inactive_agents = p
    minSafeProb = min(minSafeProb, round(currSafeProb, 4))
    OnlineTime, numOfReplans, timestep, Online_SST = 0, 0, 0, 0
    SGAT = len(GoalLocations) * OfflineTime

    while True:
        if time.time() - start_time >= max_instance_time:
            return (round(OfflineTime, 3), None, 300.0, numOfReplans, None, Offline_SST,
                    round(countExpand / (numOfReplans + 1), 3), minSafeProb, None)

        #################################### Run Simulation #############################################
        s = Run_Simulation(plan_paths, DelaysProbDictExecution, AgentLocations, GoalLocations, randGen, timestep,
                           Online_SST, inactive_agents)

        if s.runSimulation():
            print("Pass\n--------------------------------------------------------------------------")
            return (round(OfflineTime, 3), round(OnlineTime, 3), round(OfflineTime, 3) + round(OnlineTime, 3),
                    numOfReplans, s.TST, Offline_SST, round(countExpand / (numOfReplans + 1), 3), minSafeProb,
                    s.TST + SGAT)

        AgentLocations, GoalLocations = s.AgentLocations, s.remainGoals
        GraphObj = GraphG(mapAndDim, GoalLocations)

        ########################################## Re-planning ############################################
        print("Calc New Plan")
        p, replan_time, currCountExpand = run_robust_planner_with_timeout(
            AgentLocations, GoalLocations, desired_safe_prob, DelaysProbDictExecution, mapAndDim, verifyAlpha,
            max_planning_time, InactiveAgents, GraphObj)

        countExpand += currCountExpand
        if p is None:
            print("Plan is None!\n--------------------------------------------------------------------------")
            return (round(OfflineTime, 3), None, None, numOfReplans + 1, None, Offline_SST,
                    round(countExpand / (numOfReplans + 2), 3),
                    minSafeProb, None)

        plan_paths, _, currSafeProb, _ = p
        minSafeProb = min(minSafeProb, round(currSafeProb, 4))
        OnlineTime += replan_time
        numOfReplans += 1
        timestep = s.timestep
        Online_SST = s.TST
        SGAT += (len(GoalLocations) * replan_time)


def compute_smart_remove_agents(AgentsLocations, GoalsLocations, delaysProbDict, graphObj):
    graphObj.CalcAllDistancesFromGoals()
    n = len(AgentsLocations)
    cur_loc = list(AgentsLocations)
    cur_cost = [0.0] * n
    goal_count = [0] * n
    remaining = list(GoalsLocations)

    while remaining:
        best_total, best_a, best_gi = math.inf, None, None
        for a in range(n):
            speed = 1.0 - delaysProbDict[a]
            if speed <= 0:
                continue
            for gi, goal in enumerate(remaining):
                d = graphObj.dictDistance[(cur_loc[a], goal)]
                if not math.isfinite(d):
                    continue
                total = cur_cost[a] + d / speed
                if total < best_total:
                    best_total, best_a, best_gi = total, a, gi
        if best_a is None:
            break
        goal = remaining.pop(best_gi)
        cur_loc[best_a] = goal
        cur_cost[best_a] = best_total
        goal_count[best_a] += 1

    return {a for a in range(n) if goal_count[a] == 0}


def run_ordered_removal_loop(AgentsLocations, GoalsLocations, delaysProbDictForExecution, order, choose_time_per_row, instance, records, num_of_goals):
    for HowManyRemove in [1, num_of_agents * 20 // 100, num_of_agents * 40 // 100, num_of_agents * 60 // 100, num_of_agents * 80 // 100, num_of_agents - 1]:
        print(f"M: {mapName}, A: {num_of_agents}, G: {num_of_goals}, I: {instance}, Mode: {howToRemove}, R: {HowManyRemove}", flush=True)
        graphObj = GraphG(mapAndDim, GoalsLocations)
        numComponentsBefore = nx.number_connected_components(graphObj.G)
        removedLocs = [AgentsLocations[order[i]] for i in range(HowManyRemove)]
        for loc in removedLocs:
            mapAndDim["Map"][loc] = 1
            graphObj.G.remove_node(loc)
        assert nx.number_connected_components(graphObj.G) == numComponentsBefore, \
            f"Removal changed components! I={instance}, R={HowManyRemove}, Mode={howToRemove}"
        inactiveAgents = {order[i] for i in range(HowManyRemove)}
        result = run_Test(AgentsLocations, GoalsLocations, delaysProbDictForExecution, inactiveAgents, graphObj)
        records.append(build_record(instance, result, HowManyRemove, num_of_goals, choose_time_per_row))
        for loc in removedLocs:
            mapAndDim["Map"][loc] = 0


####################################################### run Tests #################################################################################

def run_instances():
    randGenForLocs = random.Random(seedPerInstanceForDelay)
    randGenForRandomOrder = random.Random(seedPerInstanceForDelay + 1)

    startInstance, endInstance = map(int, instanceRange.split("_"))
    for instance in range(startInstance, endInstance + 1):
        for num_of_goals in goalsList:
            records = []
            AgentsLocations, GoalsLocations = read_locs_from_file(instance, num_of_goals)
            delaysProbDictForExecution = {a: round(randGenForLocs.uniform(0.5, 0.95), 3) for a in range(len(AgentsLocations))}
            for a in randGenForLocs.sample(range(len(AgentsLocations)), num_of_agents // 10):
                delaysProbDictForExecution[a] = 0

            if howToRemove == "NoExclusion":
                print(f"M: {mapName}, A: {num_of_agents}, G: {num_of_goals}, I: {instance}, Mode: NoExclusion, R: 0", flush=True)
                graphObj = GraphG(mapAndDim, GoalsLocations)
                result = run_Test(AgentsLocations, GoalsLocations, delaysProbDictForExecution, set(), graphObj)
                records.append(build_record(instance, result, 0, num_of_goals, 0.0))

            elif howToRemove in ("DelayExclusion", "RandomExclusion"):
                t0 = time.time()
                if howToRemove == "DelayExclusion":
                    order = sorted(range(len(AgentsLocations)), key=lambda a: (-delaysProbDictForExecution[a], a))
                else:
                    order = list(range(len(AgentsLocations)))
                    randGenForRandomOrder.shuffle(order)
                choose_time_per_row = time.time() - t0
                run_ordered_removal_loop(AgentsLocations, GoalsLocations, delaysProbDictForExecution,
                                         order, choose_time_per_row, instance, records, num_of_goals)

            elif howToRemove == "SEER":
                graphObj = GraphG(mapAndDim, GoalsLocations)
                t0 = time.time()
                inactiveAgents = compute_smart_remove_agents(
                    AgentsLocations, GoalsLocations, delaysProbDictForExecution, graphObj
                )
                choose_time = time.time() - t0
                num_removed = len(inactiveAgents)
                print(f"M: {mapName}, A: {num_of_agents}, G: {num_of_goals}, I: {instance}, Mode: SEER, R: {num_removed}", flush=True)
                numComponentsBefore = nx.number_connected_components(graphObj.G)
                removedLocs = [AgentsLocations[a] for a in inactiveAgents]
                for loc in removedLocs:
                    mapAndDim["Map"][loc] = 1
                    graphObj.G.remove_node(loc)
                assert nx.number_connected_components(graphObj.G) == numComponentsBefore, \
                    f"Removal changed components! I={instance}, Mode=SEER, R={num_removed}"
                result = run_Test(AgentsLocations, GoalsLocations, delaysProbDictForExecution, inactiveAgents, graphObj)
                records.append(build_record(instance, result, num_removed, num_of_goals, choose_time))
                for loc in removedLocs:
                    mapAndDim["Map"][loc] = 0

            with open(f"Output_files/Output_{configStr}.csv", mode="a", newline="", encoding="utf-8") as file:
                writerRecord = csv.writer(file)
                writerRecord.writerows(records)


def build_record(instance, result, num_removed, num_of_goals, choose_remove_time=0.0):
    (offlineRuntime, onlineRuntime, runtime, needReplan, sstOnline, sstOffline, countExpand, minSafeProb, sgat) = result

    choose_remove_time = round(choose_remove_time, 3)
    total_runtime = round(runtime + choose_remove_time, 3) if runtime is not None else None

    row = [mapName, mapAndDim["Cols"], mapAndDim["Rows"], mapAndDim["ObsRatio"], desired_safe_prob, num_of_agents,
           num_of_goals, num_removed, howToRemove, num_of_agents / mapAndDim["FreeCells"], instance,
           offlineRuntime, onlineRuntime, choose_remove_time, total_runtime,
           sgat, needReplan, sstOnline, sstOffline, countExpand, minSafeProb]

    return row


run_instances()
