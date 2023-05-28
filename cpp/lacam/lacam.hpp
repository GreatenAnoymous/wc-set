#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "utils.hpp"

#include "../mapf/solver.hpp"


// a wrapper class
class LacamSolver:public Solver{
public:
    LacamSolver(Problem *p);
    LacamSolver(Grid *graph, Config starts,Config goals);

    ~LacamSolver();
    void solveInstance();

private:



    LACAM::Planner *planner=nullptr;
    LACAM::Instance* ins;
    LACAM::Deadline* deadline;

};

