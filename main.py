from utils import *
import Grids
import MSVC
import MAPF
import json
import os
import matplotlib.pyplot as plt
import time
import random
import statistics






def test_unlabeled():
    graph=Grids.Grid8Connected("./maps/den312d.map")
    agent=600
    starts, goals = generate_random_instance(graph, agent)
    # solver=MAPF.TSWAP(graph,starts,goals)
    solver=MAPF.FlowBasedUMRPP(graph,starts,goals)
    print("begin to solve")
    solution=solver.solveWeighted()
    
    print("makespan=",solution.getMakespan())


def test_unlabeled2():
    graph=Grids.Grid("./maps/orz201d.map")
    agent=180
    starts, goals = generate_random_instance(graph, agent)
    # solver=MAPF.TSWAP(graph,starts,goals)
    # solver=MAPF.FlowBasedUMRPP(graph,starts,goals)
    print("begin to solve")
    # solution=solver.solveWeighted()
    solver=MSVC.UNPP(graph,starts,goals)
    solver.load_data("./data/4connected/orz201d_VS.json")
    solver.read_distance_table("./data/distance/orz201d_dist.dat")
    solver.unlabeled_incomplete()
    print(solver.intermediate_data,solver.getLowerBoundMakespan(),solver.getLowerBoundSOC())
    
    paths=[]
    # solver.distance_optimal_formation(starts,goals,paths)
    # paths=MAPF.Paths(paths)
    # print(paths.countConflict())
    
    
    
    # print("makespan=",solution.getMakespan())

def test_min_cost():
    from scipy.optimize import linear_sum_assignment
    from scipy.sparse import csr_matrix

    # Generate a random cost matrix
    # cost_matrix = np.random.rand(360, 360)
    n = 360
    m = 360
    density = 0.01
    data = np.random.rand(int(n * m * density))
    row = np.random.randint(0, n, len(data))
    col = np.random.randint(0, m, len(data))
    cost_matrix = csr_matrix((data, (row, col)), shape=(n, m))

    # Solve the assignment problem using linear_sum_assignment
    

    # Solve the assignment problem using linear_sum_assignment
    t0=time.time()
    # row_ind, col_ind = linear_sum_assignment(cost_matrix)
    row_ind, col_ind = linear_sum_assignment(cost_matrix.toarray())
    t1=time.time()
    print(f"Comp time: {t1-t0}")

    # Print the total cost and the assigned rows and columns
    total_cost = cost_matrix[row_ind, col_ind].sum()
    print(f"Total cost: {total_cost}")
    print(f"Assigned rows: {row_ind}")
    print(f"Assigned columns: {col_ind}")

def precompute_distance(map_name:str):
    map_file_name="./maps/"+map_name+".map"
    graph=Grids.Grid(map_file_name)
    outputJson="./data/distance/"+map_name+"_dist.dat"
    msvc=MSVC.MSVC()
    msvc.preprocessDistanceMatrix(graph,outputJson)


def grid_experiment():
    grid_len=[5,10,15,20,25,30,35,40,45,50]
    # grid_len=[5]
    data_dict=dict()
    vsize=[]
    count=50
    time_data=[]
    ratio_data=[]
    for m in grid_len:
        graph=Grids.Grid(m,m,[])
        vm=0
        time_sum=0
        per_best=0
        ratio_sum=0
        for k in range(count):
            msvc=MSVC.MSVC()
            t0=time.time()
        # sol=msvc.findMaxPathDominatedVertexSetWithBinarySearch(graph)
            sol=msvc.findMaximalPathDominatedSet(graph)
            
            # sol=msvc.findMaximumPathDominatedSet(graph,-1)
            t1=time.time()
            # ratio_sum+=msvc.evaluate_ratio(graph,sol,None
            if len(sol)>vm:
                vm=len(sol)
                per_best=msvc.evaluate_ratio(graph,sol,None)
            # vm=max(len(sol),vm)
            
            # print(t1-t0,len(sol))
            # exit(0)
            time_sum+=t1-t0
        vsize.append(vm)
        ratio_data.append(per_best)
        time_data.append(time_sum/count)

    data_dict=dict()
    data_dict["grid_len"]=grid_len
    data_dict["vsize"]=vsize
    data_dict["time"]=time_data
    data_dict["ratio"]=ratio_data
    with open("./data/4connected/grid_experiment_improved.json", "w") as f:
        json.dump(data_dict,f)


def evaluate_map_maxial_vertices(map_name:str):
    map_file_name="./maps/"+map_name+".map"
    graph=Grids.Grid(map_file_name)
    v_size=len(graph.getV())

    max_size_found=0
    curr_best=[]
    edge_size=graph.countEdges()
    size_data=[]
    num_trials=20
    sum_time=0
    outputJson="./data/4connected/"+map_name+"_VS.json"
    for i in range(num_trials):
        msvc=MSVC.MSVC()
        t0=time.time()
        sol=msvc.findMaximalPathDominatedSet(graph)
        t1=time.time()
        sum_time+=t1-t0
        if len(sol)>max_size_found:
            max_size_found=len(sol)

            curr_best=sol
        size_data.append(len(sol))
        per_best=msvc.evaluate_ratio(graph,curr_best,None)
        try:
            data_dict=dict()
            data_dict["name"]=map_name
            data_dict["|V|"]=v_size
            data_dict["|E|"]=edge_size
            data_dict["per"]=per_best
            data_dict["maximalSetSize"]=len(curr_best)
            data_dict["avg"]=statistics.mean(size_data)
            data_dict["err"]=statistics.stdev(size_data)
            data_dict["time"]=sum_time/num_trials
            data_dict["maximal_vSet"]=[(n.pos.x,n.pos.y) for n in curr_best]


            with open(outputJson, "w") as outfile:
                json.dump(data_dict, outfile, indent=4)
        except:
            pass


def test_pbs():
    graph=Grids.Grid("./maps/orz201d.map")
    starts,goals=generate_random_instance(graph,140)
    solver=MAPF.PBS(graph,starts,goals)
    # solver=MAPF.HCA(graph,starts,goals,60000)
    # solver=MAPF.ECBS(graph,starts,goals)
    solver.solve()


def check_wf():
    graph=Grids.Grid("./maps/den312d.map")
    with open("./data/4connected/den312d_VS.json","r") as f:
        wf_set=json.load(f)["maximal_vSet"]
        np.random.shuffle(wf_set)
        wf_nodes=[graph.getNode(x,y) for (x,y) in wf_set]
        msvc=MSVC.MSVC()
        flag=msvc.OnePointCheckIfPathConnected(wf_nodes,graph)
        print(flag)

def check_unpp_wf2(pType="orz201d"):
    graph=Grids.Grid("./maps/"+pType+".map")
    k=50
    agents=180
    for i in range(k):
        with open("./data/4connected/orz201d_VS.json","r") as f:
            wf_set=json.load(f)["maximal_vSet"]
            # np.random.shuffle(wf_set)
            # wf_nodes=[graph.getNode(x,y) for (x,y) in wf_set]
            # starts=random.sample(wf_set,agents)
            # goals=random.sample(wf_set,agents)
            starts=wf_set[:agents]    
            goals=wf_set[agents:2*agents]
            starts=[graph.getNode(x,y) for (x,y) in starts]
            goals=[graph.getNode(x,y) for (x,y) in goals]
            solver=MAPF.PIBT(graph,starts,goals)
            # solver=MAPF.HCA(graph,starts,goals,60000)
            solver.solve()
            sol=solver.getSolution()
            print(sol.getMakespan())
            

            # solver=MSVC.UNPP(graph,starts,goals)
            # solver.read_distance_table("./data/distance/"+pType+"_dist.dat")
            # paths=MAPF.Paths(agents)
            # solver.prioritized_planner(starts,goals,paths)
            print("i=",i)
    




def test():
    m=49
    n=49
    graph=Grids.Grid("./maps/orz201d.map")
    # graph=Grids.Grid(24,24,[])
    # graph=Grids.Grid("./maps/30x30.map")
    msvc=MSVC.MSVC()
    # sol=msvc.findMaxPathDominatedVertexSetWithBinarySearch(graph)
    # sol=msvc.findMaximalPathDominatedSet(graph)

    sol=msvc.findMaximumPathDominatedSet(graph,-1)

    print(msvc.evaluate_ratio(graph,sol,sol[0]))
    sol=[(n.pos.x,n.pos.y) for n in sol]
    print(sol)
    print(len(sol))
    # Create a 5x5 tilemap with some tiles in black and others in green
    tilemap=[[0 for i in range(n)] for i in range(m)]
    for x,y in sol:
        tilemap[x][y]=1
    tilemap=np.array(tilemap)
    # Define the x and y coordinates of the center of each tile
    x = np.arange(0, m+0.1, 1) 
    y = np.arange(0, n+0.1, 1) 

    # Create a 2D plot with a square aspect ratio
    fig, ax = plt.subplots(figsize=(n, m))
    ax.set_aspect('equal')

    # Plot the tiles in black or green based on their values
    for i in range(m):
        for j in range(n):
            if tilemap[i, j] == 0:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='white'))
            else:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='green',edgecolor="red"))

            if graph.existNode(i,j)==False:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor='black'))

    # Set the x and y limits and tick positions
    ax.set_xlim(0, n)
    ax.set_ylim(0, m)
    ax.set_xticks(y)
    ax.set_yticks(x)



    # plt.savefig("orz201d.svg",bbox_inches="tight",pad_inches=0.05)
    # Show the plot
    plt.show()
if __name__ == "__main__":

    # test()
    grid_experiment()
    # test32x32()
    # generate_json("60x60")
    # generate_short_instance_json(15)
    # generate_density_instance_json()
    # generate_density_instance_json()
    # check_wf()
    # test_unlabeled()
    # evaluate_map_maxial_vertices("Shanghai_0_256")
    # precompute_distance("hrt002d")
    # check_unpp_wf2()
    # generate_gauss_json("hrt002d")
#
    # test_pbs()
    # test_unlabeled2()
    # test_min_cost()