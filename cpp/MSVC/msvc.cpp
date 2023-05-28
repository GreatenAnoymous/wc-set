/**
 * @file msvc.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include"msvc.hpp"
#include<algorithm>
#include <random>
#include<cassert>
#include"mathtools.hpp"
#include<map>
#include <limits>
#include <fstream>
#include "../mapf/json.hpp"


bool MSVC::AStarSearchCheck(Node * s, Node *g, Grid *graph, const std::set<Node*> &constraints){
    CmpAstarFunc compareOpen=[](AStarNode n1,AStarNode n2){
        return std::get<1>(n1)>std::get<1>(n2);
    };
    std::priority_queue<AStarNode,std::vector<AStarNode>,CmpAstarFunc> open(compareOpen);
    std::set<Node*> closed;
    open.push({s,s->manhattanDist(g),0});
    while(open.empty()==false){
        auto n=open.top();
        open.pop();
        if(std::get<0>(n)==g) return true;
        closed.insert(std::get<0>(n));
        auto nbrs=std::get<0>(n)->neighbor;
        for(auto nbr:nbrs){
            if(closed.find(nbr)!=closed.end()) continue;
            if(constraints.find(nbr)!=constraints.end()) continue;
            int t=std::get<2>(n)+1;
            int f=t+nbr->manhattanDist(g);
            AStarNode childNode={nbr,f,t};
            open.push(childNode);
        }
    }
    return false;
}


/**
 * @brief 
 * 
 * @param candidate 
 * @return Node* 
 */
Node * MSVC::chooseRandomNode(std::set<Node*>& candidate) {
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Get the size of the set
    int size = candidate.size();
    
    // Choose a random index in the set
    std::uniform_int_distribution<> dis(0, size - 1);
    int index = dis(gen);
    
    // Iterate over the set to find the node at the chosen index
    auto it = candidate.begin();
    std::advance(it, index);
    return *it;
}


double MSVC::evaluate_raito(Grid *graph,Nodes &candidates, Node *reference){
    if(reference==nullptr){
        for(auto c:graph->getV()){
            if(std::find(candidates.begin(),candidates.end(),c)==candidates.end()){
                reference=c;
                break;
            }
        }
    }
    double soc=0,soc_lb=0;
    std::set<Node*> constraints;
    std::set<Node*> empty_set;
    for(auto c:candidates) constraints.insert(c);
    for(auto c:candidates){
        if(c==reference) continue;
        constraints.erase(c);
        // auto pp=AStarSearchPath(reference,c,graph,empty_set);
        auto pp=graph->getPathWithoutCache(reference,c,empty_set);
        // for(auto u:pp){
        //     printf("path u=(%d,%d)\n",u->pos.x,u->pos.y);
        // }
        // assert(pp.size()-1==reference->DiagonalDist(c));
        // std::cout<<p1<<"   "<<std::endl;
        // printf("======================================");
        // auto p=AStarSearchPath(reference,c,graph,constraints);
        auto p=graph->getPathWithoutCache(reference,c,constraints);
        // reference->println();
        // c->println();
  
        constraints.insert(c);
        if(pp.size()>p.size()){
            std::cout<<"pp.size()="<<pp.size()<<"manhattan=" <<graph->pathDist(reference,c)<<"   p.size()="<<p.size()<<std::endl;
        }
        assert(pp.size()<=p.size());
        soc+=p.size();
        soc_lb+=pp.size();   

    }
    return soc_lb/soc;

}



Path MSVC::AStarSearchPath(Node*s,Node*g,Grid *graph,const std::set<Node*> &constraints){
    CmpAstarFunc compareOpen=[](AStarNode n1,AStarNode n2){
        if (std::get<1>(n1)!=std::get<1>(n2))
            return std::get<1>(n1)>std::get<1>(n2);
        return std::get<2>(n1)<std::get<2>(n2);
    };
    std::priority_queue<AStarNode,std::vector<AStarNode>,CmpAstarFunc> open(compareOpen);
    std::map<Node*,int> closed;
    Path result;
    std::map<Node*,AStarNode> parentMap;
    AStarNode root={s,s->manhattanDist(g),0};
    open.push(root);
    // printf("s=(%d,%d)   g=(%d,%d)\n",s->pos.x,s->pos.y,g->pos.x,g->pos.y);
    while(open.empty()==false){
        auto n=open.top();
        open.pop();
        auto u=std::get<0>(n);
    
        printf("(%d,%d) f=%d t=%d->\n",u->pos.x,u->pos.y,std::get<1>(n),std::get<2>(n));
        if(std::get<0>(n)==g){
            auto curr=n;
            while(std::get<0>(curr)!=s){
                auto v=std::get<0>(curr);
                result.push_back(v);
                curr=parentMap[v];
            }
            result.push_back(s);
            return result;
        }
       
        closed[std::get<0>(n)]=std::get<2>(n);
        auto nbrs=std::get<0>(n)->neighbor;
        for(auto nbr:nbrs){
        
            if(constraints.find(nbr)!=constraints.end()) continue;
            int t=std::get<2>(n)+1;
            if(closed.find(nbr)==closed.end()){
                if(closed[nbr]<=t) continue;
            }
            // int f=t+nbr->manhattanDist(g);
            int f=t+graph->dist(nbr,g);
            AStarNode childNode={nbr,f,t};
            parentMap[nbr]=n;
            open.push(childNode);
        }
    }
    return {};
}


bool MSVC::BFSCheckIfPathConnected(const Nodes &nodes,Grid *graph)
{
    std::set<Node*> nodeSet;
    for(auto n:nodes) nodeSet.insert(n);
    for(int i=0;i<nodes.size();i++){
        std::set<Node*> constraints;
        std::queue<Node*> open;
        open.push(nodes[i]);
        std::set<Node*> closed;
        while(open.empty()==false){
            auto n=open.front();
            open.pop();
            closed.insert(n);
            auto nbrs=n->neighbor;
            for(auto nbr:nbrs){
                if(closed.find(nbr)!=closed.end()) continue;
                if(nodeSet.find(nbr)!=nodeSet.end()) continue;
                open.push(nbr);
            }
        }
        if(closed.size()!=(graph->getV().size()-nodes.size()+1)) return false; 
    }
    return true;    
}



Nodes MSVC::findMaxPathConnectedVertexSetWithBinarySearch(Grid *graph){
    Nodes result;
    return result;

}


Nodes MSVC::findMaxPathDominatedVertexSetWithBinarySearch(Grid *graph){
    //find the vertex set with size=k
    auto helper=[&](int k, Nodes &solution){
        auto all_nodes=graph->getV();
        std::cout<<"k="<<k<<std::endl;
         // Shuffle the vector
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(all_nodes.begin(), all_nodes.end(), g);
        std::vector<int> indices(k);
        for (int i = 0; i < k; ++i) {
            indices[i] = i;
        }
        do{
            Nodes nodes(k);
            for(int i=0;i<k;i++){
                nodes[i]=all_nodes[indices[i]];
            }
            if(OnePointCheckIfPathConnected(nodes,graph)==true) {
                solution=nodes;
                return true;
            }
        }while(next_combination(indices, all_nodes.size(), k));
        return false;
    };
    Nodes result;
    int low=0;
    int high=graph->getV().size();
    while(low<=high){
        int mid=low+(high-low)/2;
        
        if(helper(mid,result)==true) {
            low=mid+1;
        }
        else {
            high=mid-1;
        }
    }
    return result;
}

void MSVC::preprocessDistanceMatrix(Grid * graph, std::string filename){

    const int INF = std::numeric_limits<int>::max();
    auto dijkstra=[&](int src,std::vector<int>& dist){
        
        std::priority_queue<std::pair<int,int>, std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>>> pq;
        pq.push(std::make_pair(0,src));
        dist[src]=0;

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto v:graph->getNode(u)->neighbor) {
                int weight = 1;
                if (dist[u] != INF && dist[v->id] > dist[u] + weight) {
                    dist[v->id] = dist[u] + weight;
                    pq.push(std::make_pair(dist[v->id], v->id));
                }
            }
        }
    };

    int m=graph->getNodesSize();
    std::vector<std::vector<int>> dist(m, std::vector<int>(m, INF));
    // nlohmann::json data;
    for(int i=0;i<m;i++){
        if(graph->existNode(i)==false) continue;
        dijkstra(i, dist[i]);
        for(int k=0;k<dist[i].size();k++){
            if(dist[i][k]==INF) dist[i][k]=-1;
        }
        // data.push_back(dist[i]);
    }


    std::ofstream outfile(filename,std::ios::binary);
    for (int i = 0; i < dist.size(); i++) {
        outfile.write(reinterpret_cast<const char*>(&dist[i][0]), dist[i].size() * sizeof(int));
    }
    // file << data.dump();

    outfile.close();
}

bool MSVC::OnePointCheckIfPathConnected(const Nodes &nodes,Grid *graph){

    std::set<Node*> nodeSet;
    for(auto v:nodes) nodeSet.insert(v);
    Node *u=nullptr;
    for(auto v:graph->getV()){
        if(nodeSet.find(v)==nodeSet.end()){
            u=v;
            break;
        }
    }
    assert(nodes.size()==nodeSet.size());
    for(auto v:nodes){
        nodeSet.erase(v);
        if(AStarSearchCheck(u,v,graph,nodeSet)==false) return false;
        nodeSet.insert(v);
    }
    return true;
}

bool MSVC::AStarSearchCheckConnected(const Nodes &nodes,Grid *graph){
    std::set<Node*> nodeSet;
    for(auto v:nodes) nodeSet.insert(v);
    for(int i=0;i<nodes.size();i++){
        nodeSet.erase(nodes[i]);
        for(int j=i+1;j<nodes.size();j++){
            nodeSet.erase(nodes[j]);
            if(AStarSearchCheck(nodes[i],nodes[j],graph,nodeSet)==false) return false;
            nodeSet.insert(nodes[j]);
        }
        nodeSet.insert(nodes[i]);
    }
    return true;
}


Nodes MSVC::greedyFindSet(Grid *graph){
    auto all_nodes=graph->getV();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(all_nodes.begin(), all_nodes.end(), gen);


    std::set<Node*> vertexSet;
    //select a random node for search
    Node* selected_root=all_nodes[0];

    //store the possible vertex for next selection
    all_nodes.erase(all_nodes.begin());

    while(all_nodes.size()!=0){
        Node *v=random_select_next_node(all_nodes);
        Path p=AStarSearchPath(selected_root,v,graph,vertexSet);
        for(auto u :p) {
            auto is_in_path = [&](Node* n) {
                return std::find(p.begin(), p.end(), n) != p.end();
            };
            all_nodes.erase(std::remove_if(all_nodes.begin(), all_nodes.end(), is_in_path), all_nodes.end());
        }
        vertexSet.insert(v);
    }
    Nodes vertices(vertexSet.size());
    std::copy(vertexSet.begin(),vertexSet.end(),vertices.begin());
    return vertices;
}

Node * MSVC::random_select_next_node(Nodes & nodes){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, nodes.size()-1);

    // generate a random index
    int index = dis(gen);
    return nodes[index];
}


Node* MSVC::nearest_select_node(Nodes &candidates, Node *last_node){
    if (candidates.empty()) {
        return nullptr;
    }

    Node* nearest_node = candidates[0];
    int min_distance = nearest_node->manhattanDist(last_node);

    for (size_t i = 1; i < candidates.size(); ++i) {
        Node* candidate = candidates[i];
        int distance = candidate->manhattanDist(last_node);

        if (distance < min_distance) {
            nearest_node = candidate;
            min_distance = distance;
        }
    }

    return nearest_node;
}

Node*MSVC::clutter_select_node(Grid* graph,std::set<Node*> &candidates,std::set<Node*> &selected){
    int min_dist=std::numeric_limits<int>::max();
    Node* nearest_node=nullptr;
    std::vector<Node*> P_vec(candidates.begin(),candidates.end());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(P_vec.begin(),P_vec.end(),gen);
    int min_soc=std::numeric_limits<int>::max();
    int index=0;
    // std::vector<int> distance(P_vec.size(),0);
    // std::vector<int> indexes;
    for(int i=0;i<P_vec.size();i++){
        // indexes.push_back(i);
        int soc=0;
        for (const auto& candidate : candidates) {
            // distance[i] += graph->dist(P_vec[i],candidate);
            soc+=graph->dist(P_vec[i],candidate);
        }
        if(soc<min_soc){
            min_soc=soc;
            index=i;
        }
    }


    // auto compareNode=[&](int a,int b){
    //     return distance[a] > distance[b];
    // };
    // std::sort(indexes.begin(), indexes.end(),compareNode);
    // return P_vec[indexes[0]];
    return P_vec[index];
}


//Articulation Points (or Cut Vertices) in a Graph using Tarjan’s Algorithm: 
std::set<Node*> MSVC::findAP(Grid *graph,std::set<Node*>&candidates, const std::set<Node*> &constraints)
{
    std::map<Node*,bool> visited;
    std::map<Node*,int> disc;
    std::map<Node*,int> low;
    std::set<Node*> isAP;
    for(auto c:candidates){
        disc[c]=0;
        low[c]=0;
        visited[c]=false;
    }
    int time = 0;
    Node *par=nullptr;
    // Adding this loop so that the
    // code works even if we are given
    // disconnected graph
    for (auto u:candidates)
        if (!visited[u])
            APUtil(graph, u, visited, disc, low,
                time, par, isAP,constraints);
    return isAP;
}

//Articulation Points (or Cut Vertices) in a Graph using Tarjan’s Algorithm: 
void MSVC::APUtil(Grid*graph,Node *u, std::map<Node*,bool> & visited,std::map<Node*,int>&disc,
                std::map<Node*,int>&low,int &time, Node* parent, std::set<Node*> &isAP,
                const std::set<Node*> &constraints){
    // Count of children in DFS Tree
    int children = 0;

    // Mark the current node as visited
    visited[u] = true;

    // Initialize discovery time and low value
    disc[u] = low[u] = ++time;
    // Go through all vertices adjacent to this
    for (auto v : u->neighbor) {
        if(constraints.find(v)!=constraints.end()) continue;
        // If v is not visited yet, then make it a child of u
        // in DFS tree and recur for it
        if (!visited[v]) {
            children++;
            APUtil(graph, v, visited, disc, low, time, u, isAP,constraints);

            // Check if the subtree rooted with v has
            // a connection to one of the ancestors of u
            low[u] = std::min(low[u], low[v]);

            // If u is not root and low value of one of
            // its child is more than discovery value of u.
            if (parent != nullptr && low[v] >= disc[u])
                isAP.insert(u);
        }

        // Update low value of u for parent function calls.
        else if (v != parent)
            low[u] = std::min(low[u], disc[v]);
    }

    // If u is root of DFS tree and has two or more children.
    if (parent == nullptr && children > 1)
        isAP.insert(u);

}

/**
 * @brief 
 * 
 * @param graph 
 * @param candidates the vertices that may be chosen to add to vertexSet 
 * @param vertexSet a path-connected vertex set
 * @return true 
 * @return false 
 */
bool MSVC::checkIfMaximalPathDominatedSet(Grid* graph, std::set<Node*> &candidates, std::set<Node*> &vertexSet){
    // auto candidates=graph->getVSet();
    // for(auto v:vertexSet) candidates.erase(v);
    std::cout<<"candidates size="<<candidates.size()<<std::endl;
    //find all aritulation points
    auto aps=findAP(graph,candidates,vertexSet);
    for(auto v:aps) candidates.erase(v);

    //find the nodes that are the only neighbor of another node in vertexSet 
    for(auto v:vertexSet){
        int k=0;
        Node* node_remain_unsed;
        for(auto nbr:v->neighbor){
            if(vertexSet.find(nbr)!=vertexSet.end()) k++;
            else node_remain_unsed=nbr;
        }
        if(k==v->neighbor.size()-1) candidates.erase(node_remain_unsed);
    }

    //if there is no candidate any more, vertexSet it is maximal
    return candidates.empty();
}


/**
 * @brief 
 * 
 * @param graph 
 * @return std::set<Node*> 
 */
Nodes MSVC::findMaximalPathDominatedSet(Grid *graph){
    auto candidates=graph->getVSet();
    std::cout<<"grapg vertex size="<<candidates.size()<<std::endl;
    std::set<Node*> vertexSet;
    Nodes  result;
    while(true){
        if(checkIfMaximalPathDominatedSet(graph,candidates,vertexSet)==true) break; // already maximal
        // choose randomly a candidate to add to the vertexSet
        auto n=chooseRandomNode(candidates);
        // auto n=clutter_select_node(graph,candidates,vertexSet);
        std::cout<<"add node ("<<n->pos.x<<","<<n->pos.y<<") to the set"<<std::endl;
        vertexSet.insert(n);
        candidates.erase(n);
        result.push_back(n);
    }
    
    return result;
}

/**
 * @brief 
 * 
 * @param graph 
 * @return Nodes 
 */
Nodes MSVC::findMaximumPathDominatedSet(Grid *graph,int k){
    Nodes maximalSet;
    if(k==-1){
        maximalSet=findMaximalPathDominatedSet(graph);
        k=maximalSet.size()+1;
    }
    

    std::set<Node*> kMaximalSet,currentBestSet;
    
    for(auto v:maximalSet) currentBestSet.insert(v);
    maximalSet.clear();
    clock=std::chrono::high_resolution_clock::now();
    std::cout<<"found maximal vertex set with size="<<k-1<<"  |V|="<<graph->getNodesSize()<<std::endl;
    while(k<graph->getNodesSize()){
        
        kMaximalSet=findSetOfSizeK(graph,k);
        std::chrono::duration<double> elapsed_seconds=(std::chrono::high_resolution_clock::now()-clock);
        if(elapsed_seconds.count()>time_limits) break;
        if(kMaximalSet.empty()==false){
            currentBestSet=kMaximalSet;
            std::cout<<"debug k="<<k<<"   kMaximalSet size="<<kMaximalSet.size()<<std::endl;
            k=kMaximalSet.size()+1;
        }
        else k=graph->getNodesSize();
    }
    for(auto v :currentBestSet) maximalSet.push_back(v);
    std::cout<<"optimal size="<<maximalSet.size()<<std::endl;
    return maximalSet;

}


/**
 * @brief 
 * 
 * @param graph 
 * @param k 
 * @return std::set<Node*> 
 */
std::set<Node*> MSVC::findSetOfSizeK(Grid* graph, int k){
    std::cout<<"searching for a maximum vertex set with size="<<k<<std::endl;
    std::set<Node*> result;
    std::unordered_set<Configuration,ConfigurationHash> visited;
    std::set<Node*> candidantes;
    DFS_search(graph,k,visited,candidantes,result);
    return result;
}

/**
 * @brief 
 * 
 * @param graph 
 * @param k 
 * @param candidates 
 * @param result 
 * @return true 
 * @return false 
 */
bool MSVC:: DFS_search(Grid *graph, int k,std::unordered_set<Configuration,ConfigurationHash> &visited,std::set<Node*> &candidates, std::set<Node*> &result){
    std::chrono::duration<double> elapsed_seconds=(std::chrono::high_resolution_clock::now()-clock);
    if(elapsed_seconds.count()>time_limits) return false;
    if(candidates.size()>=k){
        result=candidates;
        return true;
    } 
    Configuration config;
    for(auto v:candidates)config.insert(v->id);
    if(visited.find(config)!=visited.end()) return false;
    visited.insert(config);
    // for(auto id:config) std::cout<<id<<"   ";
    // std::cout<<std::endl;
    config.clear();
    auto P=computeCandidates(graph,candidates);
    
    if(candidates.size()+P.size()<k) {
        // std::cout<<"pruned"<<std::endl;
        return false;
    }
    std::vector<Node*> P_vec(P.begin(), P.end());
    std::vector<int> distance(P_vec.size(),0);
    std::vector<int> indexes;
    for(int i=0;i<P_vec.size();i++){
        indexes.push_back(i);
        for (const auto& candidate : candidates) {
            distance[i] += graph->dist(P_vec[i],candidate);
            // distance[i] += P_vec[i]->manhattanDist(candidate);
        }
    }
    // auto getDistanceSum=[](Node *a, const std::set<Node*>& candidates){
    //     int sum = 0;
    //     for (const auto& candidate : candidates) {
    //         sum += a->manhattanDist(candidate);
    //     }
    //     return sum;
    // };

    auto compareNode=[&](int a,int b){
        return distance[a] > distance[b];
    };
    std::sort(indexes.begin(), indexes.end(),compareNode);
    P.clear();
    for(auto id:indexes){
        auto v=P_vec[id];
        auto next_candidate=candidates;
        next_candidate.insert(v);
        if(DFS_search(graph,k,visited,next_candidate,result)){
            return true;
        }
    }
    // std::cout<<"Cannot found larger set"<<std::endl;
    return false;
}


/**
 * @brief 
 * 
 * @param graph 
 * @param selected 
 * @return std::set<Node*> 
 */
std::set<Node*> MSVC::computeCandidates(Grid *graph, std::set<Node*> &selected){
    auto candidates=graph->getVSet();
    for(auto u:selected) candidates.erase(u);
    auto aps=findAP(graph,candidates,selected);
    for(auto u:aps) candidates.erase(u);
    for(auto v:selected){
        int k=0;
        Node* node_remain_unsed;
        for(auto nbr:v->neighbor){
            if(selected.find(nbr)!=selected.end()) k++;
            else node_remain_unsed=nbr;
        }
        if(k==v->neighbor.size()-1) candidates.erase(node_remain_unsed);
    }
    return candidates;
}