#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>

#include <set>
#include "msvc.hpp"
#include "unpp.hpp"

namespace py = pybind11;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PYBIND11_MODULE(MSVC, m ){
    m.doc()="MSVC wrapper";
    
    pybind11::class_<MSVC>(m,"MSVC")
        .def(pybind11::init())
        .def("findMaxPathConnectedVertexSetWithBinarySearch",&MSVC::findMaxPathConnectedVertexSetWithBinarySearch,py::return_value_policy::reference)
        .def("findMaxPathDominatedVertexSetWithBinarySearch",&MSVC::findMaxPathDominatedVertexSetWithBinarySearch,py::return_value_policy::reference)
        .def("OnePointCheckIfPathConnected",&MSVC::OnePointCheckIfPathConnected)
        .def("AStarSearchCheckConnected",&MSVC::AStarSearchCheckConnected)
        .def("findMaximalPathDominatedSet",&MSVC::findMaximalPathDominatedSet,py::return_value_policy::reference)
        .def("findMaximumPathDominatedSet",&MSVC::findMaximumPathDominatedSet,py::return_value_policy::reference)
        .def("preprocessDistanceMatrix",&MSVC::preprocessDistanceMatrix)
        .def("evaluate_ratio",&MSVC::evaluate_raito)
        .def("AStarSearchCheck",&MSVC::AStarSearchCheck);
        


    pybind11::class_<UNPP,Solver>(m,"UNPP")
        .def(pybind11::init<Problem*>())
        .def(pybind11::init<Grid*,Config,Config>())
        .def_readonly("intermediate_data", &UNPP::intermediate_data)
        .def("load_data",&UNPP::load_data)
        .def("read_distance_table", &UNPP::read_distance_table)
        .def("distance_optimal_formation",&UNPP::distance_optimal_formation)
        .def("unlabeled_complete",&UNPP::unlabeled_complete)
        .def("unlabeled_incomplete",&UNPP::unlabeled_incomplete)
        .def("getLowerBoundMakespan",&UNPP::getLowerBoundMakespan)
        .def("getLowerBoundSOC",&UNPP::getLowerBoundSOC)
        .def("prioritized_planner", &UNPP::prioritized_planner);

}