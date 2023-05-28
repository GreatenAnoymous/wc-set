/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

namespace LACAM
{
    struct Instance
    {
        Graph G; // graph
        Config starts; // initial configuration
        Config goals;  // goal configuration
        uint N;        // number of agents
        Instance(){}
        // for testing
        Instance(const std::string &map_filename,
                 const std::vector<int> &start_indexes,
                 const std::vector<int> &goal_indexes);
        // for MAPF benchmark
        Instance(const std::string &scen_filename, const std::string &map_filename,
                 const int _N = 1);
        // random instance generation
        Instance(const std::string &map_filename, std::mt19937 *MT, const int _N = 1);

        Instance(const std::string &map_name, const std::string &json_name);
        ~Instance() {}

        // simple feasibility check of instance
        bool is_valid(const int verbose = 0) const;
    };

    // solution: a sequence of configurations
    using Solution = std::vector<Config>;
}