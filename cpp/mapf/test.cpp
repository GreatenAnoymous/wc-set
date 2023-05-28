#include "../lacam/lacam.hpp"
#include "pibt_complete.hpp"
#include <iostream>
#include <fstream>
#include"json.hpp"
#include <chrono>
void test()
{
    auto graph = new Grid(30, 30, 0);
    auto starts = graph->generateRandomConfig(20);
    auto goals = graph->generateRandomConfig(20);
    LacamSolver *solver = new LacamSolver(graph, starts, goals);
    solver->solveInstance();
    auto solution = solver->getSolution();
    std::cout << solution.getMakespan() << "  " << solution.getSOC() << std::endl;
    std::cout << solver->getLowerBoundMakespan() << "  " << solver->getLowerBoundSOC() << std::endl;
    delete solver;
}

void lacam(int argc, char *argv[]){
     // Check the number of arguments
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    int grid_size = std::atoi(argv[1]);
    int id=std::atoi(argv[2]);
    float density=std::atof(argv[3]);
    int agents=int(grid_size*grid_size*density);
    std::cout<<grid_size<<"   "<<id<<"    "<<density<<std::endl;
    std::string instance_name = "./instances/density/agents"+std::to_string(agents)+"_"+std::to_string(id)+".json";
    std::string output_name = "./tmp/agent"+std::to_string(agents)+"_"+std::to_string(id)+".json";

    Grid *graph=new Grid(grid_size,grid_size,0);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout<<instance_name<<std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout<<"number of agents="<<starts_vec.size()<<std::endl;
    Config starts,goals;


    for(int k=0;k<starts_vec.size();k++){
        starts.push_back(graph->getNode(starts_vec[k][0],starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0],goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);
    LacamSolver *solver = new LacamSolver(graph, starts, goals);
    auto start_time = std::chrono::high_resolution_clock::now();
    solver->solveInstance();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds=duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

// Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan()/(double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC()/(double)solver->getLowerBoundSOC();
    data["runtime"] = seconds;
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();


}


void pibt_complete(int argc, char *argv[]){
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }


    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name=argv[2];
    std::string output_name=argv[3];



    Grid *graph=new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout<<instance_name<<std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout<<"number of agents="<<starts_vec.size()<<std::endl;
    Config starts,goals;


    for(int k=0;k<starts_vec.size();k++){
        starts.push_back(graph->getNode(starts_vec[k][0],starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0],goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);
    // LacamSolver *solver = new LacamSolver(graph, starts, goals);
    PIBT_COMPLETE *solver=new PIBT_COMPLETE(graph,starts,goals);
    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    solver->solve();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds=duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

// Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan()/(double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC()/(double)solver->getLowerBoundSOC();
    data["runtime"] = seconds;
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}





// Function to solve the problem


int main(int argc, char *argv[])
{
    pibt_complete(argc,argv);
    return 0;
}