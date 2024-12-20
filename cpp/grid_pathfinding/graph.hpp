#pragma once
#include <random>
#include <unordered_map>
#include "node.hpp"
#include <set>
using Path = std::vector<Node *>; // < loc_i[0], loc_i[1], ... >

// Pure graph. Base class of Grid class.
class Graph
{
private:
    /*
     * two approaches to find the shortest path
     * 1. without cache -> getPathWithoutCache
     * 2. with cache -> getPathWithCache
     */

    // get path avoiding several nodes
    Path getPathWithoutCache(Node *const s, Node *const g,
                             std::mt19937 *MT = nullptr,
                             const Nodes &prohibited_nodes = {}) const;

  
    // find a path using cache, if failed then return empty
    Path getPathWithCache(Node *const s, Node *const g,
                          std::mt19937 *MT = nullptr);

    // helpers for cache
    std::unordered_map<std::string, Path> PATH_TABLE;
    // get key name for cache
    static std::string getPathTableKey(const Node *const s, const Node *const g);
    // register already searched path to cache
    void registerPath(const Path &path);

    // body
protected:
    // V[y * width + x] = Node with position (x, y)
    // if (x, y) is occupied then V[y * width + x] = nullptr
    Nodes V;
   
    // something strange
    void halt(const std::string &msg);

public:
    Graph();
    virtual ~Graph();

     Path getPathWithoutCache(Node * s, Node * g,
                             std::set<Node *> &prohibited_nodes,std::mt19937 *MT = nullptr);
    // in grid, id = y * width + x
    virtual bool existNode(int id) const { return false; };
    virtual bool existNode(int x, int y) const { return false; };

    // in grid, id = y * width + x
    virtual Node *getNode(int x, int y) const { return nullptr; };
    virtual Node *getNode(int id) const { return nullptr; };

    // in grid, Manhattan distance
    virtual int dist(const Node *const v, const Node *const u) const { return 0; }
    virtual int getWidth() {return 0;};
    virtual int getHeight() {return 0;};
    // get path between two nodes
    Path getPath(Node *const s, Node *const g, const bool cache = true,
                 std::mt19937 *MT = nullptr, const Nodes &prohibited_nodes = {});
    Path getPath(Node *const s, Node *const g, const Nodes &prohibited_nodes, std::mt19937 *MT = nullptr);

    // get path length between two nodes
    int pathDist(Node *const s, Node *const g, const bool cache = true,
                 std::mt19937 *MT = nullptr, const Nodes &prohibited_nodes = {});

    // get all nodes without nullptr
    Nodes getV() const;

    int countEdges();
    

    std::set<Node*> getVSet();

    // get width*height
    int getNodesSize() const { return V.size(); }
};

class Grid : public Graph
{
protected:
    std::string map_file;
    int width;
    int height;

public:
    Grid(){};
    Grid(int xmax,int ymax,std::vector<std::vector<int>>obstacles);
    Grid(const std::string &_map_file);
    Grid(std::vector<std::vector<int>> &map);
    Grid(int xmax,int ymax,double obstacle_density);
    ~Grid(){};

    bool existNode(int id) const;
    bool existNode(int x, int y) const;

    bool is_blocked(int x,int y) const;
    bool is_blocked(Pos v) const;
    Node *getNode(int id) const;
    Node *getNode(int x, int y) const;
    std::vector<Pos> getObstacles();

    int dist(const Node *const v, const Node *const u) const
    {
        // printf("manhattan_dist is called=%d\n",v->manhattanDist(u));
        return v->manhattanDist(u);
    }
    Nodes generateRandomConfig(int num_agents);
    std::string getMapFileName() const { return map_file; };
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    std::vector<std::vector<int>> getBinaryMap();

    bool inBoundary(int x,int y) const;

  
};


class Grid8Connected:public Grid{
public:
    Grid8Connected(const std::string &_map_file);
    Grid8Connected(int xmax,int ymax,std::vector<std::vector<int>>obstacles);


    int dist(const Node *const v, const Node *const u) const
    {
        // printf("diag dist is called=%d\n",v->DiagonalDist(u));
        return v->DiagonalDist(u);
    }


};
