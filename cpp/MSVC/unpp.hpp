

#pragma once

#include "../mapf/solver.hpp"
#include "../mapf/hca.hpp"
#include <map>

class UNPP: public Solver{
public:

    using point2d=std::pair<Node*,int>;   //(vertex, weight)
    using DAG=std::map<Node*,std::vector<point2d>>;
    UNPP(Problem *p);
    UNPP(Grid *graph, Config starts,Config goals);
    void load_data(std::string file_name);

    void read_distance_table(std::string data_file);

    void unlabeled_complete();
    void unlabeled_incomplete();
    
    std::vector<int> intermediate_data;
    
    void prioritized_planner(const Config &starts,const Config &goals, Paths&paths);


    Path find_path(Node *s,Node *g,int i,
                                const Paths &paths,
                                const int time_limit,
                                const int upper_bound,
                                const std::vector<std::tuple<Node *, int>> &constraints,
                                Solver::CompareAstarNode &compare,
                                const bool manage_path_table);
    void distance_optimal_formation(Config &starts, Config &goals,Paths &paths);

    int getLowerBoundMakespan();
    int getLowerBoundSOC();

private:
    // std::set<Node*> wellConnectedSet;
  

    void update_paths(Config &starts, Config &goals,std::vector<Path> &paths);

    std::vector<Node*> data_set;

    
    

    void find_initial_paths(const Config &starts,const Config &goals,std::vector<Path> & paths);

    void schedule(const Config &starts,const Config &goals,std::vector<Path> & paths);

    void formDAG(std::vector<Path> &paths, DAG&dag_graph);

    void BFS(Node *s, DAG &dag,Path &,std::function<bool(Node*)>isGoal );

    void astar_search(Node* start, Node* goal, Path &path);

        

    std::vector<std::vector<int>>all_pair_distance_table;


    void min_cost_matching(std::vector<std::vector<int>> &costMatrix,std::vector<int> &assignment);

    struct AStarNode{
        int f;
        std::shared_ptr<AStarNode> parent;
        int t;
        Node *v;
        AStarNode(Node*v_, int f_,int t_, std::shared_ptr<AStarNode>  parent_=nullptr):v(v_),f(f_),t(t_),parent(parent_){}
    };

    using AStarNode_p=std::shared_ptr<AStarNode>;


    struct compareOpen{
        bool operator()(AStarNode_p a, AStarNode_p b){
            if(a->f!=b->f)
                return a->f>b->f;
            return a->t>b->t;
        }
    };



    

    // Path getPrioritizedPath(int id, const Paths &paths);
    Path space_time_astar(Node *start,Node *goal,std::function<bool(AStarNode_p)> isGoal,std::function<bool(AStarNode_p)> isValid,int max_constraint_time=0);
};


