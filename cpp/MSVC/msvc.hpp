/**
 * @file msvc.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "../grid_pathfinding/graph.hpp"
#include "../grid_pathfinding/node.hpp"
#include<iostream>
#include<set>
#include<queue>
#include<functional>
#include<map>
#include <unordered_set>
#include <chrono>
class MSVC{
public:

    using Configuration=std::set<int>;
    struct ConfigurationHash {
        std::size_t operator()(const Configuration& config) const {
            std::size_t seed = config.size();
            for (auto& elem : config) {
                seed ^= std::hash<int>{}(elem) + 0x9e3779b9 + (seed<<6) + (seed>>2);
            }
            return seed;
        }
    };
    using AStarNode=std::tuple<Node*,int,int>;
    using CmpAstarFunc=std::function<bool(AStarNode,AStarNode)> ;

    

    Path AStarSearchPath(Node *s,Node *g, Grid *graph,const std::set<Node*> &constraints);

    MSVC(){}
    
    Nodes findMaxPathConnectedVertexSetWithBinarySearch(Grid *graph);

    Nodes findMaxPathDominatedVertexSetWithBinarySearch(Grid *graph);


    bool BFSCheckIfPathConnected(const Nodes& nodes,Grid *graph);

    bool OnePointCheckIfPathConnected(const Nodes &, Grid *graph);

    bool AStarSearchCheckConnected(const Nodes &nodes,Grid *graph);

    bool AStarSearchCheck(Node * s, Node *g, Grid *graph,const std::set<Node*> &constraints);

    Nodes greedyFindSet(Grid *graph);


    std::set<Node*> findAP(Grid *graph,std::set<Node*>&candidates, const std::set<Node*> & constraints);
    
    void APUtil(Grid*graph,Node *u, std::map<Node*,bool> & visited,std::map<Node*,int>&disc,std::map<Node*,int>&low,int &time, Node* parent, std::set<Node*> &isAP,const std::set<Node*> &constraints);


    bool checkIfMaximalPathDominatedSet(Grid* graph, std::set<Node*> &candidates,std::set<Node*> &vertexSet);

    Nodes findMaximalPathDominatedSet(Grid *graph);

    Nodes findMaximumPathDominatedSet(Grid *graph,int lowerbound=-1);

    void preprocessDistanceMatrix(Grid *graph,std::string fileName);

    double evaluate_raito(Grid *graph,Nodes &candidates, Node *reference=nullptr);



private:
    Node *random_select_next_node(Nodes &);
    Node *nearest_select_node(Grid*graph,std::set<Node*> &,std::set<Node*> &selected);
    Node *clutter_select_node(Grid*graph,std::set<Node*> &,std::set<Node*> &selected);
    


    Node *chooseRandomNode(std::set<Node*>& nodeSet);

    std::set<Node*> findSetOfSizeK(Grid *graph,int k);

    std::set<Node*> computeCandidates(Grid *graph, std::set<Node*> &selected);

    bool DFS_search(Grid *graph, int k,std::unordered_set<Configuration,ConfigurationHash> &visited ,std::set<Node*> &candidates, std::set<Node*> &result);


    const double time_limits=600;
    std::chrono::_V2::system_clock::time_point clock;

};


