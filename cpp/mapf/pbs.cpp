#include "pbs.hpp"
#include <chrono>
PBS::PBS(Problem * p_):Solver(p_){}


PBS::PBS(Grid*graph,Config starts,Config goals):Solver(graph,starts,goals){}

void PBS::run()
{
    std::stack<PBSNode_p> OPEN;
    PBSNode_p n = std::make_shared<PBSNode>();
    setInitialHighLevelNode(n);
    OPEN.push(n);
    int num_agents = P->getNum();
    int num_expanded=0;
    while (!OPEN.empty())
    {
        num_expanded++;
        if (overCompTime())
        {
            std::cout << "time out" << std::endl;
            break;
        }
        auto n = OPEN.top();
        OPEN.pop();

        auto agent_pair = findFirstConflict(n->paths);
        
        if (agent_pair.empty())
        {
            solved = true;
            break;
        }
        
        std::vector<PBSNode_p> children;

        for (int i = 0; i < 2; i++)
        {
            int a1 = agent_pair[i];
            int a2 = agent_pair[(i + 1) % 2];
            PBSNode_p m = std::make_shared<PBSNode>();
            m->higher_agents = n->higher_agents;
            m->lower_agents = n->lower_agents;
            m->higher_agents[a1].insert(a2);  
            m->lower_agents[a2].insert(a1);
            m->paths=n->paths;
            m->valid=true;
            // auto start=std::chrono::high_resolution_clock::now();
            updatePlan(m, a1);
            // auto end=std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = end - start;
            // printf("update the plan time cost=%f\n", elapsed.count());
            if (!m->valid){
                printf("node is invalid!\n");
                continue;
            }
            children.push_back(m);
            // printf("conflict between %d and %d addressed\n",a1,a2);      
            
        }
        
        if(children.size()==2&&children[0]->soc>children[1]->soc)std::swap(children[0],children[1]);
        for(auto c: children) OPEN.push(c);
        if (solved)
            solution = pathsToPlan(n->paths);
    }
    // printf("num of nodes expanded=%d\n",num_expanded);
}

void PBS::setInitialHighLevelNode(PBSNode_p n)
{
    Paths paths(P->getNum());
    // std::cout << "num robots= " << P->getNum() << std::endl;
    std::vector<int> f_mins; // vector of costs for respective paths
    for (int i = 0; i < P->getNum(); ++i)
    {
        Path path = getInitialPath(i, paths);
        // std::cout << "path size for agent " << i << "  =  " << path.size() << std::endl;
        paths.insert(i, path);
        f_mins.push_back(path.size() - 1);
    }
    // std::cout<<"debug??? "<<paths.size()<<"  "<<paths.getMakespan()<<std::endl;
    n->paths = paths;
}

Path PBS::getInitialPath(int id, const Paths &paths)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    Nodes config_g = P->getConfigGoal();

    Path path = {s};
    Node *p = s;
    int t = 1;
    const int makespan = paths.getMakespan();
    const int num_agents = P->getNum();
    while (p != g)
    {
        p = *std::min_element(p->neighbor.begin(), p->neighbor.end(),
                              [&](Node *a, Node *b)
                            {
                                if (pathDist(id, a) != pathDist(id, b))
                                    return pathDist(id, a) < pathDist(id, b);
                                if (t <= makespan)
                                {
                                    Node *v;
                                    for (int i = 0; i < num_agents; ++i)
                                    {
                                        if (paths.empty(i))
                                            continue;
                                        v = paths.get(i, t);
                                        if (v == a)
                                            return false;
                                        if (v == b)
                                            return true;
                                    }
                                }
                                if (a != g && inArray(a, config_g))
                                    return false;
                                if (b != g && inArray(b, config_g))
                                    return true;
                                return false;
                            });
        path.push_back(p);
        ++t;
    }

    return path;
}

bool PBS::dfs(PBSNode_p node,int v, std::set<int>& visited, std::set<int>& exploring, std::vector<int>& stk){
    visited.insert(v);
    exploring.insert(v);
    for (auto w : node->lower_agents[v]) {
        if (visited.find(w)==visited.end()) {
            if (dfs(node, w, visited, exploring, stk)) {
                return true;
            }
        } else if (exploring.find(w)!=exploring.end()) {
            return true;
        }
    }
    exploring.erase(v);
    stk.push_back(v);
    return false;
}

std::vector<int> PBS::topological_sort(PBSNode_p node, int agent)
{
    // printf("doing topological sort start from agent %d\n", agent);
    // for(auto aj:node->lower_agents[agent]){
    //     printf("aj %d is lower than %d\n",aj,agent);
    // }
    std::vector<int> sorted_order;
    std::set<int> visited;
    std::set<int> exploring;
    bool cycle=dfs(node,agent, visited, exploring,sorted_order);
    if(cycle==false){
        std::reverse(sorted_order.begin(),sorted_order.end());
        return sorted_order;
    }
    else{
        return {};
    }

}



void PBS::updatePlan(PBSNode_p node, int id)
{
    // auto start = std::chrono::high_resolution_clock::now();
    auto agents = topological_sort(node, id);
    for(int i=0;i<agents.size();i++){
        for(int j=i+1;j<agents.size();j++){
            for(auto ai:node->higher_agents[i]) {
                node->lower_agents[ai].insert(j);
                node->higher_agents[j].insert(ai);
            }
            node->lower_agents[i].insert(j);
            node->higher_agents[j].insert(i);
        }
    }
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = end - start;
    // printf("topological sort comp time=%f\n", elapsed.count());
    // printf("higher agents of %d :",id);
    // for(auto a:node->higher_agents[id])printf("%d ",a);
    // printf("\n");
    // printf("lower agents of %d :",id);
    // for(auto a:node->lower_agents[id])printf("%d ",a);
    // printf("\n");
    // printf("topological sort:");
    // for(auto agent: agents) printf("%d  ",agent);
    // printf("\n");
    if(agents.empty()){

        printf("topological agents empty, cycle detected\n");
        node->valid=false;
        return;
    }
    // for(auto aj:node->higher_agents[id]){
    //     printf("higher agent aj %d  of agent id= %d\n",aj,id);
    // }
    // start=std::chrono::high_resolution_clock::now();
    preparePathTable(node, id);
    // end = std::chrono::high_resolution_clock::now();
    // elapsed = end - start;
    // printf("prepareTable time cost=%f\n", elapsed.count());
    for (auto a : agents)
    {
        bool conflicted = false;
        if (a != id)
        {
            for (auto k : node->higher_agents[a])
            {
                //find if there is conflict with higher agent
                for (int t = 1; t <= node->paths.getMakespan(); t++)
                {
                    if (node->paths.get(a, t) == node->paths.get(k, t))
                    {
                        conflicted = true;
                        break;
                    }
                    if (node->paths.get(a, t - 1) == node->paths.get(k, t) and
                        node->paths.get(a, t) == node->paths.get(k, t - 1))
                    {
                        conflicted = true;
                        break;
                    }
                }
            }
        }
        if (a == id or conflicted)
        {   
            // start=std::chrono::high_resolution_clock::now();
            replanPath(node, a);
            // end = std::chrono::high_resolution_clock::now();
            // elapsed = end - start;
            // printf("replanning time cost=%f\n", elapsed.count());
            if (node->valid == false){
                clearPathTable(node->paths);
                resetHardTable(node->paths);
                return;
            }
        }
    }
    // start=std::chrono::high_resolution_clock::now();
    clearPathTable(node->paths);
    resetHardTable(node->paths);
    // end = std::chrono::high_resolution_clock::now();
    // elapsed = end - start;
    // printf("clear path table=%f\n", elapsed.count());
}

void PBS::preparePathTable(PBSNode_p node, int id)
{
    int makespan = node->paths.getMakespan();
    std::set<int> agents;
    for(int k=0;k<P->getNum();k++) {
        if(k==id) continue;
        agents.insert(k);
    }
  


    for(auto k:node->higher_agents[id]) agents.erase(k);
    const int nodes_size = G->getNodesSize();
    // extend PATH_TABLE
    while ((int)PATH_TABLE.size() < makespan + 1)
        PATH_TABLE.push_back(std::vector<int>(nodes_size, NIL));
    // update locations
    for (auto k : agents)
    {
        if (node->paths.empty(k))
            continue;
        auto p = node->paths.get(k);
        for (int t = 0; t <= makespan; ++t)
            PATH_TABLE[t][p[t]->id] = k;
    }

    while ((int)HARD_TABLE.size() < makespan + 1)
        HARD_TABLE.push_back(std::vector<int>(nodes_size, NIL));
    // update locations
    for (auto k : node->higher_agents[id])
    {
        if (node->paths.empty(k))
            continue;
        auto p = node->paths.get(k);
        for (int t = 0; t <= makespan; ++t)
            HARD_TABLE[t][p[t]->id] = k;
    }
    

    // for(auto k:node->higher_agents[id]){
    //     if (node->paths.empty(k))
    //         continue;
    //     auto p = node->paths.get(k);
    //     for (int t = 0; t <= makespan; ++t)
    //         PATH_TABLE[t][p[t]->id] = k;
    // }
    // std::cout << "SOlver path updated" << std::endl;
}


void PBS::updateHardTable(PBSNode_p node,int id,Path &p){
    if (p.empty())
        return;

    const int makespan = HARD_TABLE.size() - 1;
    const int nodes_size = G->getNodesSize();
    const int p_makespan = p.size() - 1;

    // extend PATH_TABLE
    if (p_makespan > makespan)
    {
        while ((int)HARD_TABLE.size() < p_makespan + 1)
            HARD_TABLE.push_back(std::vector<int>(nodes_size, NIL));
        for (int i = 0; i < P->getNum(); ++i)
        {
            if (node->paths.empty(i))
                continue;
            auto v_id = node->paths.get(i, makespan)->id;
            for (int t = makespan + 1; t <= p_makespan; ++t)
                HARD_TABLE[t][v_id] = i;
        }
    }

    // register new path
    for (int t = 0; t <= p_makespan; ++t)
        HARD_TABLE[t][p[t]->id] = id;
    if (makespan > p_makespan)
    {
        auto v_id = p[p_makespan]->id;
        for (int t = p_makespan + 1; t <= makespan; ++t)
            HARD_TABLE[t][v_id] = id;
    }
}

void PBS::replanPath(PBSNode_p node, int id)
{
    const int ideal_dist = pathDist(id);
    Node *g = P->getGoal(id);
    Node *s = P->getStart(id);
    int max_constraint_time=0;
    // for (int t = node->paths.getMakespan(); t >= ideal_dist; --t)
    // {
    //     for (auto i:node->higher_agents[id])
    //     {
    //         if (i != id && !node->paths.empty(i) && node->paths.get(i, t) == g)
    //         {
    //             max_constraint_time = t;
    //             break;
    //         }
    //     }
    //     if (max_constraint_time > 0)
    //         break;
    // }
    // printf("max constraint time=%d, ideal_dist=%d\n", max_constraint_time,ideal_dist);

    auto h_agents=node->higher_agents[id];
    for (auto k : h_agents)
    {
        max_constraint_time = std::max(node->paths.costOfPath(k), max_constraint_time);
    }
    const auto paths = node->paths;
    FocalHeuristics f1Value;
    f1Value = [&](FocalNode *n)
    {
        return n->g + pathDist(id, n->v);
    };

    
    
    FocalHeuristics f2Value = [&](FocalNode *n)
    {
        if (n->g == 0)
            return 0;
        // last node
        if (n->g > max_constraint_time)
        {
            if (PATH_TABLE[max_constraint_time][n->v->id] != Solver::NIL)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            if (PATH_TABLE[n->g][n->v->id] != Solver::NIL)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLE[n->g][n->p->v->id] != Solver::NIL &&
                     PATH_TABLE[n->g - 1][n->v->id] ==
                         PATH_TABLE[n->g][n->p->v->id])
            {
                return n->p->f2 + 1;
            }
        }
        return n->p->f2;
    };

    CompareFocalNode compareOPEN = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->f2 != b->f2)
            return a->f2 > b->f2; 
        if (a->g != b->g)
            return a->g < b->g;

        return false;
    };

    CompareFocalNode compareFOCAL = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f2 != b->f2)
            return a->f2 > b->f2;
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    CheckFocalFin checkFocalFin = [&](FocalNode *n)
    {
        // printf("focal search node (%d,%d) with t=%d explored goal= (%d,%d)\n", n->v->pos.x,n->v->pos.y,n->g,g->pos.x,g->pos.y);
        return (n->v == g && n->g > max_constraint_time);
    };

    CheckInvalidFocalNode checkInvalidFocalNode = [&](FocalNode *m)
    {
        if (max_constraint_time > 0)
        {
            if (m->g > max_constraint_time)
            {
                if (HARD_TABLE[max_constraint_time][m->v->id] != NIL){
                    // printf(" higher agents=%d, makespan=%d,max_time=%d,g=%d\n",h_agents.size(),PATH_TABLE.size(),max_constraint_time,m->g);
                    // assert(PATH_TABLE.size()>m->g);
                    // printf(" PATH TABLE=%d\n",PATH_TABLE[max_constraint_time]][m->v->id]);
                    // if(h_agents.find(PATH_TABLE[max_constraint_time][m->v->id])!=h_agents.end())
                        return true;
                }
            }
            else
            {
                // vertex conflict
                if (HARD_TABLE[m->g][m->v->id] != NIL){
                    // if(h_agents.find(PATH_TABLE[m->g][m->v->id])!=h_agents.end())
                        return true;
                }
                    
                // swap conflict
                if (HARD_TABLE[m->g][m->p->v->id] != NIL &&
                    HARD_TABLE[m->g - 1][m->v->id] == HARD_TABLE[m->g][m->p->v->id]){
                    // if(h_agents.find(PATH_TABLE[m->g-1][m->v->id])!=h_agents.end())
                        return true;
                }
            }
        }
        return false;
    };
    auto start=std::chrono::high_resolution_clock::now();
    auto p = getTimedPathByFocalSearch(s, g, sub_optimality, f1Value, f2Value,
                                       compareOPEN, compareFOCAL, checkFocalFin,
                                       checkInvalidFocalNode, max_constraint_time);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double>elapsed = end - start;
    // printf("focal search cost=%f\n", elapsed.count());
    if (p.empty() == true)
    {
        s->println();
        g->println();
        node->valid = false;
        clearPathTable(node->paths);
        resetHardTable(node->paths);
    }
    else
    {
        node->paths.insert(id, p);
        updatePathTableWithoutClear(id,p,node->paths);
        updateHardTable(node,id,p);
    }
}

void PBS::resetHardTable(Paths &paths){
    const int makespan = paths.getMakespan();
    const int num_agents = paths.size();
    for (int i = 0; i < num_agents; ++i)
    {
        if (paths.empty(i))
            continue;
        auto p = paths.get(i);
        for (int t = 0; t <= makespan; ++t){
            if(t>=HARD_TABLE.size()) continue;
            // assert(t<PATH_TABLE.size());
            // if(t>=p.size()) printf("t=%d, p_size=%d\n",t,p.size());
            // assert(t<p.size());
            
            // assert(p[t]->id<PATH_TABLE[t].size());
            // if(p[t]->id>PATH_TABLE[t].size()){
            //     printf("debugging t=%d, vid=%d, size=%d\n",t,p[t]->id,PATH_TABLE[t].size());
            // }
            HARD_TABLE[t][p[t]->id] = NIL;
        }
    }

}

Path PBS::getTimedPathByFocalSearch(
    Node *const s, Node *const g,
    float w, // sub-optimality
    FocalHeuristics &f1Value, FocalHeuristics &f2Value,
    CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
    CheckFocalFin &checkFocalFin, CheckInvalidFocalNode &checkInvalidFocalNode,
    int max_constraint_time)
{
    auto getNodeName = [&max_constraint_time](FocalNode *n)
    {
        if (n->g <= max_constraint_time)
            return std::to_string(n->v->id) + "-" + std::to_string(n->g);
        else
            return std::to_string(n->v->id) + "-" + std::to_string(max_constraint_time+1);
    };

    auto getPathFromFocalNode = [](FocalNode *_n)
    {
        Path path;
        FocalNode *n = _n;
        while (n != nullptr)
        {
            path.push_back(n->v);
            n = n->p;
        }
        std::reverse(path.begin(), path.end());
        return path;
    };

    std::vector<FocalNode *> GC; // garbage collection
    auto createNewNode = [&](Node *v, int g, int f1, int f2, FocalNode *p)
    {
        FocalNode *new_node = new FocalNode{v, g, f1, f2, p};
        GC.push_back(new_node);
        return new_node;
    };

    // OPEN, FOCAL, CLOSE
    std::priority_queue<FocalNode *, std::vector<FocalNode *>, CompareFocalNode>
        OPEN(compareOPEN);
    std::unordered_map<std::string, bool> CLOSE;


    // initial node
    FocalNode *n;
    n = createNewNode(s, 0, 0, 0, nullptr);
    n->f1 = f1Value(n);
    n->f2 = f2Value(n);
    OPEN.push(n);

    int f1_min = n->f1;

    // main loop
    bool invalid = true;
    while (!OPEN.empty())
    {
        // focal minimum node
        n = OPEN.top();
        OPEN.pop();
        
        if (CLOSE.find(getNodeName(n)) != CLOSE.end())
            continue;
        CLOSE[getNodeName(n)] = true;

        // check goal condition
        if (checkFocalFin(n))
        {
            invalid = false;
            break;
        }

        // expand
        Nodes C = n->v->neighbor;
        C.push_back(n->v);
        for (auto u : C)
        {
            int g_cost = n->g + 1;
            FocalNode *m = createNewNode(u, g_cost, 0, 0, n);
            // set heuristics
            m->f1 = f1Value(m);
            m->f2 = f2Value(m);
            // already searched?
            if (CLOSE.find(getNodeName(m)) != CLOSE.end())
                continue;
            // check constraints
            if (checkInvalidFocalNode(m))
                continue;
            // update open list
            OPEN.push(m);
            // if (m->f1 <= f1_min * w)
            //     FOCAL.push(m);
        }
    }

    Path path;
    // success
    if (!invalid)
        path = getPathFromFocalNode(n);

    // free
    for (auto p : GC)
        delete p;
    // printf("*****************\n");
    return path;
}




std::vector<int> PBS::findFirstConflict(Paths &paths)
{
    int makespan = paths.getMakespan();
    int num_agents = P->getNum();
    for (int t = 1; t <= makespan; t++)
    {
        for (int i = 0; i < num_agents; i++)
        {
            for (int j = i + 1; j < num_agents; j++)
            {
                if (paths.get(i, t) == paths.get(j, t)){
                    // printf("find vertex conflict between agent %d and agent %d at (%d,%d), time %d\n", i,j,
                    //     paths.get(i, t)->pos.x,paths.get(i, t) ->pos.y,t);
                    return {i, j};
                }
                    
                if (paths.get(i, t - 1) == paths.get(j, t) and paths.get(i, t) == paths.get(j, t - 1)){
                    // printf("find edge conflict between agent %d and agent %d at (%d,%d) to (%d,%d), time %d\n", i,j,
                    //     paths.get(i, t-1)->pos.x,paths.get(i, t-1) ->pos.y,
                    //     paths.get(i, t)->pos.x,paths.get(i, t) ->pos.y,
                    //     t);
                    return {i, j};
                }
                    
            }
        }
    }
    return {};
}