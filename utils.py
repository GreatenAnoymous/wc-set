
import Grids
import MAPF
import MSVC
import time
from common import *
import scipy
import lbap
import json
import math
import random

def generate_random_instance(graph, num_agents):
    starts = graph.getV()
    goals = graph.getV()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    return starts[:num_agents], goals[:num_agents]


def load_instance_json(graph, agents, index, pType="third_density"):
    # file_name="./instances/half_dense/agents"+str(agents)+"_"+str(index)+".json"
    file_name = "./instances/"+pType+"/agents" + \
        str(agents)+"_"+str(index)+".json"
    # file_name="./instances/corner_dense/agents"+str(agents)+"_"+str(index)+".json"
    with open(file_name, "r") as f:
        data_dict = json.load(f)
        starts = data_dict['starts']
        goals = data_dict["goals"]
        starts = [graph.getNode(s[0], s[1]) for s in starts]
        goals = [graph.getNode(g[0], g[1]) for g in goals]
    return starts, goals


def benchmarkMRPP(graph, function_generate_starts_goals, num_agents: list, num_cases=20, file_name=None, function_arg=None, pType="lak103d"):
    mkpn_data = []
    soc_data = []
    rate_data = []
    comp_time_data = []
    nodes_data = []
    agents_data = []
    time_limit=60000
    itemList = ['num_agents', 'mkpn', 'soc',
                'runtime', 'success_rate', 'nodes_expanded']
    for agent in num_agents:
        mkpn_sum = 0
        soc_sum = 0
        rate_sum = 0
        count = 0
        time_sum = 0
        nodes_sum = 0

        for k in range(num_cases):
            # print(agent,k)
            if function_arg is None:
                starts, goals = function_generate_starts_goals(graph, agent)
            else:
                starts, goals = function_generate_starts_goals(
                    graph, agent, k, pType)
            # solver=MAPF.PIBT_COMPLETE(graph,starts,goals)
            # solver=MAPF.LacamSolver(graph,starts,goals)
            # solver=MAPF.HCA(graph,starts,goals,time_limit)
            solver=MAPF.PIBT(graph,starts,goals)
            # solver=MAPF.ECBS(graph,starts,goals)
            # solver=MAPF.PushAndSwap(graph,starts,goals)
            t1 = time.time()
            try:
                solver.solve()
                # solver.solveInstance()
                solution = solver.getSolution()
            except:
                print("failed")
                solution=MAPF.Plan()
                
            t2 = time.time()
            #+t4-t3
            
            if solution.size() > 1 and (t2-t1) < time_limit/1000 and solver.getLowerBoundMakespan() != 0:
                mkpn_lb = solver.getLowerBoundMakespan()
                # mkpn_sum+=(solution.getMakespan()+mkpn1+mkpn2)/mkpn_lb
                mkpn_sum += (solution.getMakespan())/mkpn_lb
                soc_lb = solver.getLowerBoundSOC()
                soc_sum += (solution.getSOC())/soc_lb
                # soc_sum+=(solution.getSOC()+(mkpn1+mkpn2)*agent)/soc_lb
                try:
                    nodes_sum += solver.getNumExpansions()
                except:
                    pass
                rate_sum += 1
                count += 1
                time_sum += t2-t1
                # print(mkpn_lb,solution.getMakespan(),soc_lb,solution.getSOC())
                # print(mkpn_sum,soc_sum,rate_sum)
            else:
                # time_sum+=time_limit
                if solver.getLowerBoundMakespan() == 0:
                    rate_sum += 1
                try:
                    nodes_sum += solver.getNumExpansions()
                except:
                    pass
            # exit(0)
        print("agents=", agent, "mkpn=", mkpn_sum/num_cases, "soc=", soc_sum/num_cases,
            "rate=", rate_sum/num_cases, "nodes_expanded=", nodes_sum/num_cases)
        if count != 0:
            mkpn_data.append(mkpn_sum/count)
            soc_data.append(soc_sum/count)
        else:
            mkpn_data.append(np.nan)
            soc_data.append(np.nan)
        if count==0:
            comp_time_data.append(np.nan)
        else:
            comp_time_data.append(time_sum/count)
        rate_data.append(rate_sum/num_cases)
        nodes_data.append(nodes_sum/num_cases)
        agents_data.append(agent)
        dataList = [agents_data, mkpn_data, soc_data,
                    comp_time_data, rate_data, nodes_data]
        if file_name is not None:
            write_multiple_csv(file_name, itemList, dataList)


def generate_gauss(num_robots, graph=None,sigma=5):
    used = set()
    starts = []
    k = 0
    center=random.choice(graph.getV())
    while k < num_robots:
        x = np.random.normal(center.pos.x, sigma)
        y = np.random.normal(center.pos.y, sigma)
        x, y = int(x), int(y)
        if graph is not None and graph.existNode(x, y) == False:
            continue
        if (x, y) not in used:
            used.add((x, y))
            starts.append((x, y))
            k += 1
    return starts


def benchmarkMRPP3( m_sides, num_cases=10,file_name=None, pType="d10"):
    mkpn_data = []
    soc_data = []
    rate_data = []
    comp_time_data = []
    nodes_data = []
    agents_data = []
    time_limit=60000
    itemList = ['num_agents', 'mkpn', 'soc',
                'runtime', 'success_rate', 'nodes_expanded']
    for m in m_sides:
        mkpn_sum = 0
        soc_sum = 0
        rate_sum = 0
        count = 0
        time_sum = 0
        nodes_sum = 0
        agent=int(m*m/3)
        graph=Grids.Grid(m,m,[])
        for k in range(num_cases):
            print(agent,k)
         
            starts, goals = load_instance_json(graph,agent,k,pType)

            # solver=MAPF.HCA(graph,starts,goals,time_limit)
            # solver=MAPF.PIBT(graph,starts,goals)
            # solver=MAPF.ECBS(graph,starts,goals)
            # solver=MAPF.PushAndSwap(graph,starts,goals)
            # solver=MAPF.PIBT_COMPLETE(graph,starts,goals)
            solver=MAPF.LacamSolver(graph,starts,goals)
            t1 = time.time()
            # solver.solve()
            solver.solveInstance()
            t2 = time.time()
            #+t4-t3
            solution = solver.getSolution()
            if solution.size() > 1 and (t2-t1) < time_limit/1000 and solver.getLowerBoundMakespan() != 0:
                mkpn_lb = solver.getLowerBoundMakespan()
                # mkpn_sum+=(solution.getMakespan()+mkpn1+mkpn2)/mkpn_lb
                mkpn_sum += (solution.getMakespan())/mkpn_lb
                soc_lb = solver.getLowerBoundSOC()
                soc_sum += (solution.getSOC())/soc_lb
                # soc_sum+=(solution.getSOC()+(mkpn1+mkpn2)*agent)/soc_lb
                try:
                    nodes_sum += solver.getNumExpansions()
                except:
                    pass
                rate_sum += 1
                count += 1
                time_sum += t2-t1
                # print(mkpn_lb,solution.getMakespan(),soc_lb,solution.getSOC())
                # print(mkpn_sum,soc_sum,rate_sum)
            else:
                # time_sum+=time_limit
                if solver.getLowerBoundMakespan() == 0:
                    rate_sum += 1
                try:
                    nodes_sum += solver.getNumExpansions()
                except:
                    pass
            # exit(0)
        print("agents=", agent, "mkpn=", mkpn_sum/num_cases, "soc=", soc_sum/num_cases,
            "rate=", rate_sum/num_cases, "nodes_expanded=", nodes_sum/num_cases)
        if count != 0:
            mkpn_data.append(mkpn_sum/count)
            soc_data.append(soc_sum/count)
        else:
            mkpn_data.append(np.nan)
            soc_data.append(np.nan)
        comp_time_data.append(time_sum/count)
        rate_data.append(rate_sum/num_cases)
        nodes_data.append(nodes_sum/num_cases)
        agents_data.append(agent)
        dataList = [agents_data, mkpn_data, soc_data,
                    comp_time_data, rate_data, nodes_data]
        if file_name is not None:
            write_multiple_csv(file_name, itemList, dataList)




def benchmarkMRPP4( distance, num_cases=20,file_name=None,m=100, pType="m100"):
    def load_dist_instance_json(graph, d, index, pType="third_density"):
            # file_name="./instances/half_dense/agents"+str(agents)+"_"+str(index)+".json"
        file_name = "./instances/"+pType+"/dist" + \
            str(d)+"_"+str(index)+".json"
        # file_name="./instances/corner_dense/agents"+str(agents)+"_"+str(index)+".json"
        with open(file_name, "r") as f:
            data_dict = json.load(f)
            starts = data_dict['starts']
            goals = data_dict["goals"]
            starts = [graph.getNode(s[0], s[1]) for s in starts]
            goals = [graph.getNode(g[0], g[1]) for g in goals]
        return starts, goals
    mkpn_data = []
    soc_data = []
    rate_data = []
    comp_time_data = []
    nodes_data = []
    agents_data = []
    time_limit=300000
    itemList = ['dist', 'mkpn', 'soc',
                'runtime', 'success_rate', 'nodes_expanded']
    for d in distance:
        mkpn_sum = 0
        soc_sum = 0
        rate_sum = 0
        count = 0
        time_sum = 0
        nodes_sum = 0
        # agent=int(m*m/3)
        graph=Grids.Grid(m,m,[])
        for k in range(num_cases):
            print(d,k)
         
            starts, goals = load_dist_instance_json(graph,d,k,pType)
            solver=MAPF.PIBT_COMPLETE(graph,starts,goals)
            # solver=MAPF.HCA(graph,starts,goals,time_limit)
            # solver=MAPF.PIBT(graph,starts,goals)
            # solver=MAPF.ECBS(graph,starts,goals)
            # solver=MAPF.PushAndSwap(graph,starts,goals)
            t1 = time.time()
            solver.solve()
            t2 = time.time()
            #+t4-t3
            solution = solver.getSolution()
            if solution.size() > 1 and (t2-t1) < time_limit/1000 and solver.getLowerBoundMakespan() != 0:
                mkpn_lb = solver.getLowerBoundMakespan()
                # mkpn_sum+=(solution.getMakespan()+mkpn1+mkpn2)/mkpn_lb
                mkpn_sum += (solution.getMakespan())/mkpn_lb
                soc_lb = solver.getLowerBoundSOC()
                soc_sum += (solution.getSOC())/soc_lb
                # soc_sum+=(solution.getSOC()+(mkpn1+mkpn2)*agent)/soc_lb
                try:
                    nodes_sum += solver.getNumExpansions()
                except:
                    pass
                rate_sum += 1
                count += 1
                time_sum += t2-t1
                # print(mkpn_lb,solution.getMakespan(),soc_lb,solution.getSOC())
                # print(mkpn_sum,soc_sum,rate_sum)
            else:
                # time_sum+=time_limit
                if solver.getLowerBoundMakespan() == 0:
                    rate_sum += 1
                try:
                    nodes_sum += solver.getNumExpansions()
                except:
                    pass
            # exit(0)
        print("dist=", d, "mkpn=", mkpn_sum/num_cases, "soc=", soc_sum/num_cases,
            "rate=", rate_sum/num_cases, "nodes_expanded=", nodes_sum/num_cases)
        if count != 0:
            mkpn_data.append(mkpn_sum/count)
            soc_data.append(soc_sum/count)
        else:
            mkpn_data.append(np.nan)
            soc_data.append(np.nan)
        comp_time_data.append(time_sum/count)
        rate_data.append(rate_sum/num_cases)
        nodes_data.append(nodes_sum/num_cases)
        agents_data.append(d)
        dataList = [agents_data, mkpn_data, soc_data,
                    comp_time_data, rate_data, nodes_data]
        if file_name is not None:
            write_multiple_csv(file_name, itemList, dataList)

def generate_gauss(num_robots, graph=None,sigma=5):
    used = set()
    starts = []
    k = 0
    center=random.choice(graph.getV())
    while k < num_robots:
        x = np.random.normal(center.pos.x, sigma)
        y = np.random.normal(center.pos.y, sigma)
        x, y = int(x), int(y)
        if graph is not None and graph.existNode(x, y) == False:
            continue
        if (x, y) not in used:
            used.add((x, y))
            starts.append((x, y))
            k += 1
    return starts

def benchmarkMRPP2(graph, function_generate_starts_goals, num_agents: list, num_cases=20, file_name=None, function_arg=None, pType="lak103d"):
    mkpn_data = []
    soc_data = []
    rate_data = []
    comp_time_data = []
    nodes_data = []
    agents_data = []
    time_limit=120000
    itemList = ['num_agents', 'mkpn', 'soc',
                'runtime', 'success_rate', 'nodes_expanded']
    for agent in num_agents:
        mkpn_sum = 0
        soc_sum = 0
        rate_sum = 0
        count = 0
        time_sum = 0
        nodes_sum = 0

        for k in range(num_cases):
            # print(agent,k)
            if function_arg is None:
                starts, goals = function_generate_starts_goals(graph, agent)
            else:
                starts, goals = function_generate_starts_goals(
                    graph, agent, k, pType+"_gauss")

            solver=MSVC.UNPP(graph,starts,goals)
            solver.load_data("./data/4connected/"+pType+"_VS.json")
            solver.read_distance_table("./data/distance/"+pType+"_dist.dat")
            t1 = time.time()
            # solver.solve()
            try:
                solver.unlabeled_complete()
            except:
                pass
            t2 = time.time()
            #+t4-t3
            solution = solver.getSolution()
            if solution.size() > 1 and (t2-t1) < time_limit and solver.getLowerBoundMakespan() != 0:
                mkpn_lb = solver.getLowerBoundMakespan()
                # mkpn_sum+=(solution.getMakespan()+mkpn1+mkpn2)/mkpn_lb
                # mkpn_sum += (solution.getMakespan())/mkpn_lb
                mkpn_sum+=(solver.intermediate_data[0]+solver.intermediate_data[1]+solver.intermediate_data[2])/mkpn_lb
                soc_lb = solver.getLowerBoundSOC()
                # soc_sum += (solution.getSOC())/soc_lb
                soc_sum+=(solver.intermediate_data[4]+(solver.intermediate_data[0]+solver.intermediate_data[2])*agent)/soc_lb
                nodes_sum += 0
                rate_sum += 1
                count += 1
                time_sum += t2-t1
                # print(mkpn_lb,solution.getMakespan(),soc_lb,solution.getSOC())
                # print(mkpn_sum,soc_sum,rate_sum)
            else:
                if solver.getLowerBoundMakespan() == 0:
                    rate_sum += 1
                nodes_sum += 1
            # exit(0)
        print("agents=", agent, "mkpn=", mkpn_sum/num_cases, "soc=", soc_sum/num_cases,
            "rate=", rate_sum/num_cases, "nodes_expanded=", nodes_sum/num_cases)
        if count != 0:
            mkpn_data.append(mkpn_sum/count)
            soc_data.append(soc_sum/count)
        else:
            mkpn_data.append(np.nan)
            soc_data.append(np.nan)
        comp_time_data.append(time_sum/count)
        rate_data.append(rate_sum/num_cases)
        nodes_data.append(nodes_sum/num_cases)
        agents_data.append(agent)
        dataList = [agents_data, mkpn_data, soc_data,
                    comp_time_data, rate_data, nodes_data]
        if file_name is not None:
            write_multiple_csv(file_name, itemList, dataList)



def generate_json(map_name):
    map_file="./maps/"+map_name+".map"
    graph = Grids.Grid(map_file)
    num_agents = list(range(200,3601,200))
    num_cases = 50
    for k in range(num_cases):
        for agent in num_agents:
            starts, goals = generate_random_instance(graph, agent)
            data_dict = dict()
            name = "./instances/"+map_name+"/agents" + \
                str(agent)+"_"+str(k)+".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_short_instance(d:int, k:int):
    m=k*d
    nodes=[(x,y) for x in range(m) for y in range(m)]
    num_agents=int(m*m/3)
    starts=random.sample(nodes,num_agents)
    goals=[]
    occupied=set()
    for s in starts:
        x0,y0=s
        candidates=[(x,y) for x in range(max(x0-d,0),min(x0+d+1,m)) for y in range(max(y0-d,0),min(y0+d+1,m))]
        np.random.shuffle(candidates)
        while candidates[0] in occupied:
            candidates.pop(0)
        goals.append(candidates[0])
        occupied.add(candidates[0])
    return starts,goals


def generate_short_instance2(d:int, m:int,density=0.25):
    nodes=[(x,y) for x in range(m) for y in range(m)]
    num_agents=int(m*m*density)
    starts=random.sample(nodes,num_agents)
    goals=[]
    occupied=set()
    for s in starts:
        x0,y0=s
        candidates=[(x,y) for x in range(max(x0-d,0),min(x0+d+1,m)) for y in range(max(y0-d,0),min(y0+d+1,m))]
        np.random.shuffle(candidates)
        while candidates[0] in occupied:
            candidates.pop(0)
        goals.append(candidates[0])
        occupied.add(candidates[0])
    return starts,goals

def generate_short_instance_json(d:int):
    m=[i*d for i in range(1,20,2)]
   
    num_cases = 10
    for k in range(num_cases):
        for side in m:
            starts, goals = generate_short_instance(d,int(side/d))
            data_dict = dict()
            name = "./instances/d15/agents" + \
                str(int(side*side/3))+"_"+str(k)+".json"

            data_dict["starts"] = starts
            data_dict["goals"] =goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_14_density_instance_json():
    m=100
    dd=[5,7,9,11,13,15]
    num_cases=20
    density=1.0/3.0
    for k in range(num_cases):
        for d in dd:
            starts,goals=generate_short_instance2(d,m,density)
            assert(len(starts)==int(m*m*density))
            data_dict = dict()
            name = "./instances/m100/dist" + str(d)+"_"+str(k)+".json"

            data_dict["starts"] = starts
            data_dict["goals"] =goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)

def generate_density_instance_json():
    sizes=list(range(30,301,30))
    num_cases=20
    density=1.0
    for m in sizes:
        num_agents=int(m*m*density)
        nodes=[(x,y) for x in range(m) for y in range(m)]
        for k in range(num_cases):
            starts=random.sample(nodes,num_agents)
            goals=random.sample(nodes,num_agents)
            data_dict = dict()
            name = "./instances/density/agents" + str(num_agents)+"_"+str(k)+".json"
            print(name)
            data_dict["starts"] = starts
            data_dict["goals"] =goals
            with open(name, "w") as f:
                # print(name)
                json.dump(data_dict, f)


            
            







def generate_gauss_json(map_name):
    map_file="./maps/"+map_name+".map"
    graph = Grids.Grid(map_file)
    num_agents = list(range(20,181,10))
    num_cases = 50
    for k in range(num_cases):
        for agent in num_agents:
            starts=generate_gauss(agent,graph,10)
            goals=generate_gauss(agent,graph,10)
            data_dict = dict()
            name = "./instances/"+map_name+"_gauss/agents" + \
                str(agent)+"_"+str(k)+".json"

            data_dict["starts"] = starts
            data_dict["goals"] = goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)

def test32x32():
    graph = Grids.Grid("./maps/hrt002d.map")
    # m_sides=list(range(30,361,30))
    # dd=[5,7,9,11,13,15]
    num_agents = list(range(20,181,20))
    # benchmarkMRPP(graph, load_instance_json, num_agents, 50,
    #             "./data/hrt002d_pibt_gauss.csv", function_arg=1, pType="hrt002d_gauss")
    benchmarkMRPP2(graph, load_instance_json, num_agents, 50,
                "./data/hrt002dUNPPc_gauss.csv", function_arg=1, pType="hrt002d")

    # benchmarkMRPP3(m_sides, 10,  "./data/oneThirdLacam.csv", pType="density")

    # benchmarkMRPP4(dd, 20,  "./data/m100PIBTc.csv", pType="m100")



    