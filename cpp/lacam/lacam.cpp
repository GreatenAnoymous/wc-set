#include"lacam.hpp"
#include <random>

LacamSolver::LacamSolver(Problem *p):Solver(p){


}

LacamSolver::~LacamSolver(){
    delete ins;
    delete planner;
}

LacamSolver::LacamSolver(Grid *graph, Config starts,Config goals):Solver(graph,starts,goals){
    
    std::random_device rd;
    deadline=new LACAM::Deadline(P->getMaxCompTime());
    std::mt19937 MT(rd());
    
    std::vector<int> start_indexes,goal_indexes;
    ins=new LACAM::Instance();
    int width=graph->getWidth();
    int height=graph->getHeight();
    ins->G.width=width;
    ins->N=starts.size();
    ins->G.height=height;
    ins->G.U=LACAM::Vertices(width * height, nullptr);
    for(int y=0;y<width;y++){
        for(int x=0;x<height;x++){
            if(graph->existNode(x,y)==false) continue;
            auto index = width * y + x;
            auto v=new LACAM::Vertex(ins->G.V.size(),index);
            ins->G.V.push_back(v);
            ins->G.U[index] = v;
        }
    }

    for(int y=0;y<width;y++){
        for(int x=0;x<height;x++){
           auto v = ins->G.U[width * y + x];
            if (v == nullptr)
                continue;
            // left
            if (x > 0)
            {
                auto u = ins->G.U[width * y + (x - 1)];
                if (u != nullptr)
                    v->neighbor.push_back(u);
            }
            // right
            if (x < width - 1)
            {
                auto u = ins->G.U[width * y + (x + 1)];
                if (u != nullptr)
                    v->neighbor.push_back(u);
            }
            // up
            if (y < height - 1)
            {
                auto u =ins->G.U[width * (y + 1) + x];
                if (u != nullptr)
                    v->neighbor.push_back(u);
            }
            // down
            if (y > 0)
            {
                auto u = ins->G.U[width * (y - 1) + x];
                if (u != nullptr)
                    v->neighbor.push_back(u);
            }
        }
    }

    for(auto start:starts){
        int id=width * start->pos.y + start->pos.x;
        ins->starts.push_back(ins->G.U[id]);
    }

    for(auto goal:goals){
        int id=width * goal->pos.y + goal->pos.x;
        ins->goals.push_back(ins->G.U[id]);
    }

    planner=new LACAM::Planner(ins,deadline,&MT,0);

}



void LacamSolver::solveInstance(){
    auto solutionVec=planner->solve();

    for(auto config:solutionVec){
        Config configx;
        for(auto v:config){
            configx.push_back(P->getG()->getNode(v->id));
        }
        solution.add(configx);
    }
}