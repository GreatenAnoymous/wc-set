#include<iostream>
#include"msvc.hpp"
#include<algorithm>
#include"mathtools.hpp"
#include "unpp.hpp"


void test_exact(){
    auto graph=new Grid(30,30,0);
    // auto graph=new Grid("./maps/den312d.map");

    auto msvc=new MSVC();

    // std::cout<<msvc->OnePointCheckIfPathConnected(nodes,graph)<<std::endl;
    // auto nodes=msvc->findMaxPathDominatedVertexSetWithBinarySearch(graph);
    auto nodes=msvc->findMaximumPathDominatedSet(graph);

    printf("found maximum vertex set size=%d\n",nodes.size());
    for(auto n:nodes){
        printf("node (%d,%d)\n",n->pos.x,n->pos.y);
    }
    // test();
}

void test_maximal(){
    auto graph=new Grid8Connected("./maps/30x30.map");
    auto msvc=new MSVC();
    auto nodes=msvc->findMaximalPathDominatedSet(graph);
    // auto nodes=msvc->findMaximumPathDominatedSet(graph);
    printf("found maximal vertex set size=%d\n",nodes.size());
    for(auto n:nodes){
        printf("node (%d,%d)\n",n->pos.x,n->pos.y);
    }
}

void test_unpp(){
    // auto graph=new Grid("./maps/den312d.map");

    // auto unpp=new UNPP();


}

int main(){
    // test_maximal();
    test_exact();
    return 0;
}