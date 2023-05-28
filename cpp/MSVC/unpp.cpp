#include "unpp.hpp"
#include "../mapf/json.hpp"

#include <iostream>
#include <fstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>
#include <boost/graph/maximum_weighted_matching.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <numeric>
#include<thread>
#include <future>
// #include <scipy/scipy.h>

namespace py = pybind11;

UNPP::UNPP(Problem *p):Solver(p){

}


UNPP::UNPP(Grid *graph,Config starts,Config goals):Solver(graph,starts,goals){

}



void UNPP::load_data(std::string file_name){
    std::ifstream json_file(file_name);
    nlohmann::json json_data;
    json_file >> json_data;

    // std::vector<Node*> maximal_vSet;
    for (auto& node : json_data["maximal_vSet"]) {
        Node *new_node=P->getG()->getNode(node[0],node[1]);
        // Node* new_node = new Node{node[0], node[1]};
        // maximal_vSet.push_back(new_node);
        data_set.push_back(new_node);
    }
}


void UNPP::unlabeled_complete(){
    int num_agents=P->getNum();
    auto starts=P->getConfigStart();
    auto goals=P->getConfigGoal();
    Config tmp_starts, tmp_goals,tmp_starts_goals,starts_goals;
    for(auto s:starts) starts_goals.push_back(s);
    for(auto g:goals) starts_goals.push_back(g);
    std::vector<int> assignment(2*num_agents);
    std::vector<std::vector<int>> costMatrix(2*num_agents,std::vector<int>(data_set.size(),0));
    for(int i=0;i<starts_goals.size();i++){
        for(int j=0;j<data_set.size();j++){
            if(all_pair_distance_table.empty())
                costMatrix[i][j]=pathDist(starts_goals[i],data_set[j]);
            else
                costMatrix[i][j]=all_pair_distance_table[starts_goals[i]->id][data_set[j]->id];
            // std::cout<<costMatrix[i][j]<<" ";
        }
    }
    // std::cout<<std::endl;
    min_cost_matching(costMatrix,assignment);
    for(int i=0;i<num_agents;i++){
        tmp_starts.push_back(data_set[assignment[i]]);
        tmp_goals.push_back(data_set[assignment[i+num_agents]]);
    }
    Paths p_s,p_g;
    // auto umapf=[&](Config & starts,Config &goals,Paths &returned){
    //     distance_optimal_formation(starts,goals,returned);
    // };
    // std::thread th1(umapf,std::ref(starts),std::ref(tmp_starts),std::ref(p_s));
    // std::thread th2(umapf,std::ref(goals),std::ref(tmp_goals),std::ref(p_g));
    // th1.join();
    // th2.join();
    distance_optimal_formation(starts,  tmp_starts,p_s);
    distance_optimal_formation(goals,  tmp_goals,p_g);
    Paths p_m(num_agents);
    prioritized_planner(tmp_starts,tmp_goals,p_m);
    if(p_m.empty()) return;
    // p_s.format();
    // p_g.format();
    // for(int i=0;i<num_agents;i++){
    //     auto psi=p_s.get(i);
    //     auto pmi=p_m.get(i);
    //     auto pgi=p_g.get(i);

    //     printf("ls=%d,lm=%d,lg=%d\n",psi.size(),pmi.size(),pgi.size());
    // }

    // printf("debugging makespan=%d  soc=%d\n",p_s.getMakespan(),p_s.getSOC());
    // printf("debugging makespan=%d  soc=%d\n",p_m.getMakespan(),p_m.getSOC());
    // printf("debugging makespan=%d  soc=%d\n",p_g.getMakespan(),p_g.getSOC());
    // p_s+=p_m;
    // p_s+=p_g;
    // solution=pathsToPlan(p_s);
    // intermediate_data.push_back(0);
    // intermediate_data.push_back(p_m.getMakespan());
    // intermediate_data.push_back(0);
    // intermediate_data.push_back(0);
    // intermediate_data.push_back(p_m.getSOC());
    // intermediate_data.push_back(0);


    intermediate_data.push_back(p_s.getMakespan());
    intermediate_data.push_back(p_m.getMakespan());
    intermediate_data.push_back(p_g.getMakespan());
    intermediate_data.push_back(p_s.getSOC());
    intermediate_data.push_back(p_m.getSOC());
    intermediate_data.push_back(p_g.getSOC());
    solution=pathsToPlan(p_m);


}


void UNPP::unlabeled_incomplete(){
    Config tmp_starts,tmp_goals; 

    int num_agents=P->getNum();
    auto starts=P->getConfigStart();
    auto goals=P->getConfigGoal();

    std::vector<int> assignment1(num_agents);
    std::vector<int> assignment2(num_agents);
    std::vector<std::vector<int>> costMatrix1(num_agents,std::vector<int>(data_set.size(),0));
    std::vector<std::vector<int>> costMatrix2(num_agents,std::vector<int>(data_set.size(),0));
    for(int i=0;i<starts.size();i++){
        for(int j=0;j<data_set.size();j++){
            if(all_pair_distance_table.empty()){
                costMatrix1[i][j]=pathDist(starts[i],data_set[j]);
                costMatrix2[i][j]=pathDist(goals[i],data_set[j]);
            }
                
            else{
                costMatrix1[i][j]=all_pair_distance_table[starts[i]->id][data_set[j]->id];
                costMatrix2[i][j]=all_pair_distance_table[goals[i]->id][data_set[j]->id];
            }
                
            // std::cout<<costMatrix[i][j]<<" ";
        }
    }
    // std::cout<<std::endl;
    min_cost_matching(costMatrix1,assignment1);
    min_cost_matching(costMatrix2,assignment2);
    for(int i=0;i<num_agents;i++){
        tmp_starts.push_back(data_set[assignment1[i]]);
        tmp_goals.push_back(data_set[assignment2[i]]);
    }
    Paths p_s,p_g;

    distance_optimal_formation(starts,  tmp_starts,p_s);
    distance_optimal_formation(goals,  tmp_goals,p_g);
    Paths p_m(num_agents);
    prioritized_planner(tmp_starts,tmp_goals,p_m);
    if(p_m.empty()) {
        std::cout<<"failed"<<std::endl;
        return;
    }

    intermediate_data.push_back(p_s.getMakespan());
    intermediate_data.push_back(p_m.getMakespan());
    intermediate_data.push_back(p_g.getMakespan());
    intermediate_data.push_back(p_s.getSOC());
    intermediate_data.push_back(p_m.getSOC());
    intermediate_data.push_back(p_g.getSOC());
    solution=pathsToPlan(p_m);

}


void UNPP::read_distance_table(std::string file_name){
    int nodeSize=P->getG()->getNodesSize();
    all_pair_distance_table=std::vector<std::vector<int>>(nodeSize, std::vector<int>(nodeSize, 0));

    // Open the binary file for reading
    std::ifstream infile(file_name, std::ios::binary);

    // Read the matrix from the file
    for (int i = 0; i < all_pair_distance_table.size(); i++) {
        infile.read(reinterpret_cast<char*>(all_pair_distance_table[i].data()), all_pair_distance_table[i].size() * sizeof(int));
    }

    // Close the file
    infile.close();

    // createDistanceTable();

    //debug
    // std::cout<<"debug   "<<all_pair_distance_table[0][0]<<std::endl;


}


void UNPP::distance_optimal_formation(Config &starts, Config &goals, Paths  &result){
    std::vector<Path> paths;
    int numAgents=starts.size();
    std::vector<std::vector<int>> cost_matrix(numAgents, std::vector<int>(numAgents));
    for (int i = 0; i < numAgents; ++i) {
        for (int j = 0; j < numAgents; ++j) {
            cost_matrix[i][j] = pathDist(starts[i],goals[j]);
            if(cost_matrix[i][j]>20) cost_matrix[i][j]=1000;
        }
    }
    printf("cost matrix created!\n");
    std::vector<int> assignment;
    // boost::hungarian_algorithm(cost_matrix, assignment);
    min_cost_matching(cost_matrix,assignment);
    printf("min cost matching done!\n");
    Config new_goals(numAgents);
    for(int i=0;i<numAgents;i++) new_goals[i]=goals[assignment[i]];
    goals.swap(new_goals);
    new_goals.clear();
    find_initial_paths(starts,goals,paths);
    printf("find initial paths done\n");
    update_paths(starts,goals,paths);
    printf("update paths done\n");
    schedule(starts,goals,paths);
    printf("schedule done\n");
    result=Paths(paths);


}

void UNPP::find_initial_paths(const Config &starts,const Config &goals,std::vector<Path> & paths){
    for(int i=0;i<starts.size();i++){
        Path pi;
        astar_search(starts[i],goals[i],pi);
        paths.push_back(pi);
    }
}

void UNPP::schedule(const Config &starts,const Config &goals,std::vector<Path> & old_paths){
    auto num_agents=old_paths.size();
    using timeObstacle=std::tuple<int,int>;
    std::vector<Path> timed_paths(num_agents);
    std::set<timeObstacle> reserveTable;
    for(auto &p:old_paths){
        Path pi;
        int t=0,k=0;
        while(k<p.size()){
            timeObstacle obsk={p[k]->id,t};
            if(reserveTable.find(obsk)==reserveTable.end()){
                pi.push_back(p[k]);
                reserveTable.insert(obsk);
                t++;
                k++;
            }else{
                //wait
                pi.push_back(pi.back());
                auto v=pi.back();
                reserveTable.insert({v->id,t});
                t++;
            }          
        }  
        auto si=pi[0];
        
        auto itr=std::find(starts.begin(),starts.end(),si);
        int index=std::distance(starts.begin(), itr);
        timed_paths[index]=pi;
        // timed_paths.push_back(pi);
    }
    old_paths.swap(timed_paths);
}


void UNPP::update_paths(Config &starts, Config &goals,std::vector<Path> &old_paths){
    int num_agents=old_paths.size();
    auto toPathSet=[](Path &p){
        std::set<Node*> path_set;
        for(auto &vs:p) path_set.insert(vs);
        return path_set;
    };
    using LocationSet=std::set<Node*>;
    LocationSet goalSet,startSet;
    std::unordered_map<int,int> degrees;



    auto findStandAloneGoal=[&](){
        for(auto &goal:goalSet){
            if(degrees[goal->id]<=1) return goal;
        }
        throw std::runtime_error("no standlone goal!");
    };

    for(int i=0;i<num_agents;i++){
        goalSet.insert(old_paths[i].back());
        startSet.insert(old_paths[i][0]);
        degrees[old_paths[i].back()->id]=0;
    }

    std::vector<LocationSet> path_sets;
    for(int i=0;i<num_agents;i++){
        LocationSet pi=toPathSet(old_paths[i]);
        path_sets.push_back(pi);
    }

    for(int i=0;i<num_agents;i++){
        for(auto &v:path_sets[i]) if(degrees.find(v->id)!=degrees.end()) degrees[v->id]++;
    }

    path_sets.clear();
    DAG dag;
    formDAG(old_paths,dag);
    printf("DAG formed\n");
    std::vector<Path> new_paths;
    while(! startSet.empty()){
        // printf("standalone step, startSet size=%d\n",startSet.size());
        auto standAloneGoal=findStandAloneGoal();
        Path pi;
        auto isGoal=[&](Node*v){
            return startSet.find(v)!=startSet.end();
        };
        BFS(standAloneGoal,dag,pi,isGoal);
        // assert(pi.empty()==false);
        // printf("BFS completed\n");
        // BFS_solver searcher(smaketandAloneGoal);
        // searcher.getNeighbors=[&](Location3d *v){
        //     auto possible_n=dag[v->id];
        //     Configs tmp;
        //     for(auto &n:possible_n){
        //         if(n.second>0) tmp.push_back(graph->getVertex(n.first));
        //     }
        //     return tmp;
        // };
        // searcher.isGoal=[&](Location3d*v){
        //     return startSet.find(v)!=startSet.end();
        // };
        // Path3d pi=searcher.solve();
        // if(pi.size()==0){
        //     std::cout<<" bug happend"<<std::endl;
        // }
        // else{
        //     auto si=pi.back();
        //     // std::cout<<si->print()<<" "<<standAloneGoal->print()<<std::endl;
        // }
        
        auto si=pi[0];
        startSet.erase(si);
        goalSet.erase(standAloneGoal);
        // std::reverse(pi.begin(),pi.end());
        new_paths.push_back(pi);
        degrees.erase(standAloneGoal->id);
        auto new_pathSet=toPathSet(pi);

        //remove the weight of used edges
        for(int t=pi.size()-1;t>=1;t--){
            auto it1=dag[pi[t]].begin();
            auto it2=dag[pi[t]].end();
            auto cond=[&](point2d &v){
                return v.first==pi[t-1];
            };
            auto it=std::find_if(it1,it2,cond);
            if(it==it2){
                printf("removing (%d,%d), (%d,%d)\n",pi[t]->pos.x,pi[t]->pos.y,pi[t-1]->pos.x,pi[t-1]->pos.y);
            }
            it->second=it->second-1;
        }

        for(auto &v:new_pathSet){
            if(degrees.find(v->id)!=degrees.end()) degrees[v->id]--;
        }
    }
    
    // std::cout<<"updated : num_agents="<<new_paths.size()<<std::endl;
    old_paths.swap(new_paths);    
}

/**
 * @brief 
 * 
 * @param paths 
 * @param dag_graph 
 */
void UNPP::formDAG(std::vector<Path> &paths, DAG&dag_graph){
    for(auto p:paths){
        for(int t=1;t<p.size();t++){
            assert(p[t]->manhattanDist(p[t-1])<=1);
        }
    }
    for(auto &p:paths){
    
        for(int t=p.size()-1;t>=1;t--){
            auto it1=dag_graph[p[t]].begin();
            auto it2=dag_graph[p[t]].end();
            auto cond=[&](point2d & v){
                return v.first==p[t-1];
            };
            auto it=std::find_if(it1,it2,cond);
            if(it==it2) dag_graph[p[t]].push_back({p[t-1],1});
            else it->second=it->second+1;
            
        }
    }

    for(auto [u, nbrs]:dag_graph){
        for(auto v:nbrs){
            assert(u->manhattanDist(v.first)<=1);
        }
    }
}




void UNPP::astar_search(Node* start, Node* goal, Path &path){
    std::priority_queue<AStarNode_p,std::vector<AStarNode_p>,compareOpen> open;
    AStarNode_p root=std::make_shared<AStarNode>(start,start->manhattanDist(goal),0,nullptr);
    open.push(root);
    std::set<Node*> closed;
    while(open.empty()==false){
        auto node=open.top();
        open.pop();
        if(closed.find(node->v)!=closed.end()) continue;
        else closed.insert(node->v);
        if(node->v==goal){
            AStarNode_p curr=node;
            while(curr!=nullptr){
                path.push_back(curr->v);
                curr=curr->parent;
            }
            std::reverse(path.begin(),path.end());
        }
        for(auto nbr:node->v->neighbor){
            if(closed.find(nbr)!=closed.end()) continue;
            int t=node->t+1;
            int f=t+nbr->manhattanDist(goal);
            AStarNode_p child=std::make_shared<AStarNode>(nbr,f,t,node);
            open.push(child);
        }
    }
}



void UNPP::prioritized_planner(const Config &starts,const Config &goals, Paths&paths){
    int num_agents=starts.size();
    
    // std::set<int> starts_goals;
    // for(auto s:starts) starts_goals.insert(s->id);
    // for(auto g:goals) starts_goals.insert(g->id);
    std::vector<int> ids(num_agents);
    int nodeSize=P->getG()->getNodesSize();
    std::vector<bool> table_starts(nodeSize,false);
    std::vector<bool> table_goals(nodeSize,false);
    for(int i=0;i<num_agents;i++){
        table_starts[starts[i]->id]=true;
        table_goals[goals[i]->id]=true;
    }
    if(PATH_TABLE.empty())PATH_TABLE.push_back(std::vector<int>(nodeSize,-1));
    std::iota(ids.begin(), ids.end(), 0); // 0,1,2,....
    
    auto get_distace=[&](int a){
        return all_pair_distance_table[starts[a]->id][goals[a]->id];
    };

    {
        // original shorter with higher priority
        std::sort(ids.begin(), ids.end(),
                [&](int a, int b)
                { return get_distace(a) > get_distace(b); });
        // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

        // // std::shuffle (ids.begin(), ids.end(), std::default_random_engine(seed));
        // std::sort(ids.begin(), ids.end(),
        //           [&](int a, int b) { return pathDist(a) < pathDist(b); });
    }

    auto start=std::chrono::high_resolution_clock::now();
    for(int j=0;j<num_agents;j++){
        int i=ids[j];
        Node *s=starts[i];
        Node *g=goals[i];
        
        std::vector<std::tuple<Node *, int>> constraints;
        // for(int k=0;k<num_agents;k++){
        //     if(k==i) continue;
        //     constraints.push_back({starts[i],-1});
        //     constraints.push_back({goals[i],-1});
        // }
        // printf("PLanning for agent %d\n",j);
        // std::function<bool(AStarNode_p)> isGoal=[&](AStarNode_p n){
        //     return n->v==g and n->t>paths.getMakespan();
        // };
        // std::function<bool(AStarNode_p)> isValid=[&](AStarNode_p n){
        //     if(starts_goals.find(n->v->id)!=starts_goals.end()) return false;
            
        //     if(n->t>paths.getMakespan()){
                
        //         if(PATH_TABLE.back()[n->v->id]!=NIL) return false;
        //     }
        //     else{
        //         // std::cout<<"debugggg ="<<PATH_TABLE.size()<<"   "<<paths.getMakespan()<<"   "<<n->t<<"   "<<n->v->id<<std::endl;
        //         // if(PATH_TABLE[n->v->id].empty()) return true;
        //         if(PATH_TABLE[n->t][n->v->id]!=NIL) {
        //             // std::cout<<"vertex invalid  "<<PATH_TABLE[n->t][n->v->id]<<"  "<<i<<" vid="<<n->v->id<<std::endl;
        //             return false;
        //         }
        //         if(PATH_TABLE[n->t][n->parent->v->id]!=NIL&&PATH_TABLE[n->t-1][n->v->id]==PATH_TABLE[n->t][n->parent->v->id]) {
        //             // std::cout<<"edge invalid "<<std::endl;
        //             return false;
        //         }
        //     }
        //     return true;
        // };

        CompareAstarNode compare = [&](AstarNode *a, AstarNode *b)
        {
            if (a->f != b->f)
                return a->f > b->f;
            // tie-break, avoid goal locations of others
            if (a->v != g && table_goals[a->v->id])
                return true;
            if (b->v != g && table_goals[b->v->id])
                return false;
            // tie-break, avoid start locations
            if (a->v != s && table_starts[a->v->id])
                return true;
            if (b->v != s && table_starts[b->v->id])
                return false;
            if (a->g != b->g)
                return a->g < b->g;
            return false;
        };

        Path pi=find_path(s,g,i, paths, getRemainedTime(),
                                              max_timestep, constraints, compare, false);
        // Path pi=Solver::getPrioritizedPath(i, paths, getRemainedTime(),
        //                                       max_timestep, constraints, compare, false);
        // Path pi=space_time_astar(starts[i],goals[i],isGoal,isValid,paths.getMakespan());
     
        if(pi.empty()){
            std::cout<<"prioritized planner failed"<<std::endl;
            clearPathTable(paths);
            paths=Paths();
            // paths=new Paths();
            return;
        }
        
        updatePathTableWithoutClear(i,pi,paths);
        // starts_goals.insert(s->id);
        // starts_goals.insert(g->id);
        paths.insert(i, pi);
    }
    clearPathTable(paths);

       auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double>elapsed = end - start;
    // printf("focal search cost=%f\n", elapsed.count());

}

Path UNPP::find_path(Node *s,Node *g,int id, const Paths &paths,
                                const int time_limit,
                                const int upper_bound,
                                const std::vector<std::tuple<Node *, int>> &constraints,
                                Solver::CompareAstarNode &compare,
                                const bool manage_path_table)
{

    const int ideal_dist = all_pair_distance_table[s->id][g->id];
    const int makespan = paths.getMakespan();

    // max timestep that another agent uses the goal
    int max_constraint_time = 0;
    // for(int i=0;i<paths.size();i++){
    //   max_constraint_time=std::max((int)paths.get(i).size(),max_constraint_time);
    // }
    for (int t = makespan; t >= ideal_dist; --t)
    {
        for (int i = 0; i < P->getNum(); ++i)
        {
            if (i != id && !paths.empty(i) && paths.get(i, t) == g)
            {
                max_constraint_time = t;
                break;
            }
        }
        if (max_constraint_time > 0)
            break;
    }

    // setup functions

    /*
     * Note: greedy f-value is indeed a good choice but sacrifice completeness.
     * > return pathDist(id, n->v)
     * Since prioritized planning itself returns sub-optimal solutions,
     * the underlying pathfinding is not limited to optimal sub-solution.
     * c.f., classical f-value: n->g + pathDist(id, n->v)
     */
    AstarHeuristics fValue;
    if (ideal_dist > max_constraint_time)
    {
        fValue = [&](AstarNode *n)
        // { return n->g + pathDist(id, n->v); };
        { return n->g + all_pair_distance_table[g->id][n->v->id]; };
    }
    else
    {
        // when someone occupies its goal
        fValue = [&](AstarNode *n)
        {
            // return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
            return std::max(max_constraint_time + 1, n->g +all_pair_distance_table[g->id][n->v->id]);
        };
    }

    CheckAstarFin checkAstarFin = [&](AstarNode *n)
    {
        return n->v == g && n->g > max_constraint_time;
    };

    // update PATH_TABLE
    if (manage_path_table)
        updatePathTable(paths, id);

    // fast collision checking
    CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode *m)
    {
        if (upper_bound != -1 && m->g > upper_bound)
            return true;

        if (makespan > 0)
        {
            if (m->g > makespan)
            {
                if (PATH_TABLE[makespan][m->v->id] != NIL)
                    return true;
            }
            else
            {
                // vertex conflict
                if (PATH_TABLE[m->g][m->v->id] != NIL)
                    return true;
                // swap conflict
                if (PATH_TABLE[m->g][m->p->v->id] != NIL &&
                    PATH_TABLE[m->g - 1][m->v->id] == PATH_TABLE[m->g][m->p->v->id])
                    return true;
            }
        }

        // check additional constraints
        for (auto c : constraints)
        {
            const int t = std::get<1>(c);
            if (m->v == std::get<0>(c) && (t == -1 || t == m->g))
                return true;
        }
        return false;
    };

    auto p = getPathBySpaceTimeAstar(s, g, fValue, compare, checkAstarFin,
                                     checkInvalidAstarNode, time_limit);

    // clear used path table
    if (manage_path_table)
        clearPathTable(paths);

    return p;
}

void UNPP:: BFS(Node *s,DAG &dag,Path &pi,std::function<bool(Node*)>isGoal){
    std::queue<Node*> open;
    open.push(s);
    std::set<Node*> closed;
    std::map<Node*,Node*> parents;
    pi.clear();
    for(auto [u, nbrs]:dag){
        for(auto v:nbrs){
            assert(u->manhattanDist(v.first)<=1);
        }
    }
    while(!open.empty()){
        auto top=open.front();
        open.pop();
        if(closed.find(top)!=closed.end()) continue;
        closed.insert(top);
        
        if(isGoal(top)){
            Node *curr=top;
            while(curr!=s){
                pi.push_back(curr);
                curr=parents[curr];
            }
            pi.push_back(s);
            break;
            // std::reverse(pi.begin(),pi.end());
        }
        for(auto c:dag[top]){
            if(closed.find(c.first)!=closed.end()) continue;
            assert(top->manhattanDist(c.first)<=1);
            if(c.second==0) continue;
        
            open.push(c.first);
            parents[c.first]=top;
        }
    }




}


void UNPP::min_cost_matching(std::vector<std::vector<int>> &cost_matrix,std::vector<int> &assignment){
     // Convert cost matrix to a NumPy array
    auto rows = cost_matrix.size();
    auto cols = cost_matrix[0].size();
    auto cost_array = py::array_t<double>({rows, cols});

    auto cost_array_unchecked = cost_array.mutable_unchecked<2>();
    for (auto i = 0; i < rows; i++) {
        for (auto j = 0; j < cols; j++) {
            cost_array_unchecked(i, j) = cost_matrix[i][j];
        }
    }
    

    // Call linear_sum_assignment function from scipy.optimize
    auto module_scipy_optimize = py::module::import("scipy.optimize");

    py::list result = module_scipy_optimize.attr("linear_sum_assignment")(cost_array).cast<py::list>();
    auto row_ind = result[0].cast<std::vector<int>>();
    auto col_ind = result[1].cast<std::vector<int>>();
    // Copy assignments to output vector
    assignment.resize(rows);
    std::fill(assignment.begin(), assignment.end(), -1);
    for (auto i = 0; i < rows; i++) {
        if (col_ind[i] != -1) {
            assignment[i] = col_ind[i];
        }
    }
    // using EdgeProperty=boost::property< boost::edge_weight_t, float, boost::property< boost::edge_index_t, int > >;
    // using BiGraph= boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,boost::no_property, EdgeProperty>;


    // int m=costMatrix.size();
    // int n=costMatrix[0].size();
    // BiGraph bi_graph(m+n);
    // assignment.resize(m);
    // for(int i=0;i<m;i++){
    //     for(int j=0;j<n;j++){
    //         // printf("%d ",-costMatrix[i][j]);
    //         boost::add_edge(i, m + j, EdgeProperty(1000-costMatrix[i][j]), bi_graph);
    //     }
    //     // printf("\n");
    // }

    
    // std::vector< boost::graph_traits< BiGraph >::vertex_descriptor >  mate(boost::num_vertices(bi_graph));
    // std::cout<<"starting to solve"<<std::endl;
    // boost::maximum_weighted_matching(bi_graph, &mate[0]);
    // boost::graph_traits< BiGraph >::vertex_iterator vi, vi_end;

    // for (boost::tie(vi, vi_end) =boost::vertices(bi_graph); vi != vi_end; ++vi){
    //     if (mate[*vi] != boost::graph_traits< BiGraph >::null_vertex()&& *vi < mate[*vi]){
    //         assignment[*vi]=mate[*vi]-m;
    //         // std::cout << "{" << *vi << ", " << mate[*vi] << "}" << std::endl;
    //     }
    // }
     
    //  debug
    // printf("debug assignment,m=%d,n=%d\n",rows,cols);
    // for(auto id:assignment){
    //     printf("%d ",id);
    // }
}

Path UNPP::space_time_astar(Node *start,Node *goal,std::function<bool(AStarNode_p)> isGoal,std::function<bool(AStarNode_p)> isValid,int max_constraint_time){
    std::priority_queue<AStarNode_p,std::vector<AStarNode_p>,compareOpen> open;
    AStarNode_p root=std::make_shared<AStarNode>(start,start->manhattanDist(goal),0,nullptr);
    open.push(root);
    Path path;
    // std::cout<<"max_constraint_time="<<max_constraint_time<<std::endl;
    auto getName=[&](AStarNode_p n){
        // if(n->t>max_constraint_time) 
        //     return std::to_string(max_constraint_time+1)+"-"+std::to_string(n->v->id);
        return std::to_string(n->t)+"-"+std::to_string(n->v->id);
    };

    std::set<std::string> closed;
    while(open.empty()==false){
        auto node=open.top();
        open.pop();
        auto nodeName=getName(node);
        // std::cout<<nodeName<<std::endl;
        if(closed.find(nodeName)!=closed.end()) continue;
        else closed.insert(nodeName);
        if(isGoal(node)){
            AStarNode_p curr=node;
            while(curr!=nullptr){
                path.push_back(curr->v);
                curr=curr->parent;
            }
            std::reverse(path.begin(),path.end());
            return path;
        }
        for(auto nbr:node->v->neighbor){
            // if(closed.find(getName())!=closed.end()) continue;
            
            int t=node->t+1;
            int f=t+nbr->manhattanDist(goal);
            // int f=t+all_pair_distance_table[nbr->id][goal->id];
            AStarNode_p child=std::make_shared<AStarNode>(nbr,f,t,node);
            if(isValid(child)==false) continue;
            open.push(child);
        }
    }
 
    return path;
}

int UNPP::getLowerBoundSOC(){
    int LB=0;
    for(int i=0;i<P->getNum();i++){
        if(all_pair_distance_table.empty()){
            LB+=pathDist(P->getStart(i),P->getGoal(i));
        }
        else{
            LB+=all_pair_distance_table[P->getStart(i)->id][P->getGoal(i)->id];
        }

    }
    return LB;
}

int UNPP::getLowerBoundMakespan(){
    int LB=0;
    for(int i=0;i<P->getNum();i++){
        if(all_pair_distance_table.empty()){
            LB=std::max(LB,pathDist(P->getStart(i),P->getGoal(i)));
        }
        else{
            LB=std::max(LB,all_pair_distance_table[P->getStart(i)->id][P->getGoal(i)->id]);
        }
    }
    return LB;
}