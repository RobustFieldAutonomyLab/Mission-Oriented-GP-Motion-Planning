//
// Created by rfal on 11/22/22.
//
#include "Planning2D.h"
//#include "Planning3D.h"
#include "extern/pybind11/include/pybind11/pybind11.h"
#include "extern/pybind11/include/pybind11/stl.h"
namespace py = pybind11;

PYBIND11_MODULE(GPPlanning, m) {
    m.doc() = "GPPlanning"; // optional module docstring
    py::class_<Planning2D>(
            m, "pyPlanning2D"
                            )
            .def(py::init<bool,double,double,double,double,int>())
            .def("buildMap", &Planning2D::pybuildMap)
            .def("optimize", &Planning2D::pyoptimize)
            ;
}