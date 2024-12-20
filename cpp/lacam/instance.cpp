#include "instance.hpp"

#include"../mapf/json.hpp"
using namespace LACAM;

Instance::Instance(const std::string &map_filename,
                   const std::vector<int> &start_indexes,
                   const std::vector<int> &goal_indexes)
    : G(map_filename),
    starts(Config()),
    goals(Config()),
    N(start_indexes.size())
{
    for (auto k : start_indexes)
        starts.push_back(G.U[k]);
    for (auto k : goal_indexes)
        goals.push_back(G.U[k]);
}

Instance::Instance(const std::string &map_name,const std::string &json_name):G(map_name){
    nlohmann::json json_data;
    std::ifstream file(json_name);
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    starts=Config();
    goals=Config();
    N=starts_vec.size();
    for(int k=0;k<N;k++){
        int sid=starts_vec[k][1]*G.width+starts_vec[k][0];
        int gid=goals_vec[k][1]*G.width+starts_vec[k][0];
        starts.push_back(G.U[k]);
        goals.push_back(G.U[k]);
    }
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string &scen_filename,
                   const std::string &map_filename, const int _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
    // load start-goal pairs
    std::ifstream file(scen_filename);
    if (!file)
    {
        info(0, 0, scen_filename, " is not found");
        return;
    }
    std::string line;
    std::smatch results;

    while (getline(file, line))
    {
        // for CRLF coding
        if (*(line.end() - 1) == 0x0d)
            line.pop_back();

        if (std::regex_match(line, results, r_instance))
        {
            auto x_s = std::stoi(results[1].str());
            auto y_s = std::stoi(results[2].str());
            auto x_g = std::stoi(results[3].str());
            auto y_g = std::stoi(results[4].str());
            if (x_s < 0 || G.width <= x_s || x_g < 0 || G.width <= x_g)
                continue;
            if (y_s < 0 || G.height <= y_s || y_g < 0 || G.height <= y_g)
                continue;
            auto s = G.U[G.width * y_s + x_s];
            auto g = G.U[G.width * y_g + x_g];
            if (s == nullptr || g == nullptr)
                continue;
            starts.push_back(s);
            goals.push_back(g);
        }

        if (starts.size() == N)
            break;
    }
}

Instance::Instance(const std::string &map_filename, std::mt19937 *MT,
                const int _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
    // random assignment
    const auto K = G.size();

    // set starts
    auto s_indexes = std::vector<int>(K);
    std::iota(s_indexes.begin(), s_indexes.end(), 0);
    std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
    int i = 0;
    while (true)
    {
        if (i >= K)
            return;
        starts.push_back(G.V[s_indexes[i]]);
        if (starts.size() == N)
            break;
        ++i;
    }

    // set goals
    auto g_indexes = std::vector<int>(K);
    std::iota(g_indexes.begin(), g_indexes.end(), 0);
    std::shuffle(g_indexes.begin(), g_indexes.end(), *MT);
    int j = 0;
    while (true)
    {
        if (j >= K)
            return;
        goals.push_back(G.V[g_indexes[j]]);
        if (goals.size() == N)
            break;
        ++j;
    }
}

bool Instance::is_valid(const int verbose) const
{
    if (N != starts.size() || N != goals.size())
    {
        info(1, verbose, "invalid N, check instance");
        return false;
    }
    return true;
}
