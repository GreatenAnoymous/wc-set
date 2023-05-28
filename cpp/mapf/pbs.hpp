#pragma once
#include "solver.hpp"

#include<set>
#include<unordered_set>
#include<map>
#include<unordered_map>
#include<stack>

#include"lib_cbs.hpp"

class PBSNode{
public:
    std::map<int,std::unordered_set<int>> higher_agents; //agent id, agents that have higher priority orderings
    std::map<int,std::unordered_set<int>> lower_agents; //agent id ,agents that have lower priority orderings
    Paths paths;
    int soc;
    int makespan;
    bool valid=true;
    LibCBS::Constraints constraints;
    
};

class PBS:public Solver{
public:
    using PBSNode_p=std::shared_ptr<PBSNode>;
    using OpenList=std::stack<PBSNode_p>;
  // used in the low-level search
    struct FocalNode
    {
        Node *v;      // location
        int g;        // in getTimedPath, g represents t
        int f1;       // used in open list
        int f2;       // used in focal list
        FocalNode *p; // parent
    };

    float sub_optimality=1.5;
    using CompareFocalNode = std::function<bool(FocalNode *, FocalNode *)>;
    using CheckFocalFin = std::function<bool(FocalNode *)>;
    using CheckInvalidFocalNode = std::function<bool(FocalNode *)>;
    using FocalHeuristics = std::function<int(FocalNode *)>;

    PBS(Problem *);
    PBS(Grid*,Config,Config);


protected:
    void run();
    void setInitialHighLevelNode(PBSNode_p );
    Path getInitialPath(int id, const Paths &paths);
    void updatePlan(PBSNode_p h_node,int id);
    void replanPath(PBSNode_p h_node,int id);
    Path getFocalPath(PBSNode_p,int id);
    Path getTimedPathByFocalSearch(
        Node *const s, Node *const g, float w, // sub-optimality
        FocalHeuristics &f1Value, FocalHeuristics &f2Value,
        CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
        CheckFocalFin &checkFocalFin,
        CheckInvalidFocalNode &checkInvalidFocalNode,
        int max_constraint_time=-1);


    


    bool dfs(PBSNode_p node,int v, std::set<int>& visited, std::set<int>& exploring, std::vector<int>& stk);     

    std::vector<int> topological_sort(PBSNode_p, int agent);

    void preparePathTable(PBSNode_p node, int agent);

    void resetHardTable(Paths &paths);

    void updateHardTable(PBSNode_p node,int k, Path &pk);

    std::vector<int> findFirstConflict(Paths &paths);

    std::vector<std::vector<int>> HARD_TABLE;


};