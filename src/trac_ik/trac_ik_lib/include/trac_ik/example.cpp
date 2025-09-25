// example.cpp
#include <pybind11/pybind11.h>
#include "trac_ik.hpp"

namespace py = pybind11;

PYBIND11_MODULE(example, m) {
    py::class_<trac_ik>(m, "trac_ik")
        .def(py::init<>())  // 导出默认构造函数
        .def("getKDLChain", &MyClass::getKDLChain);  // 导出成员函数
}
