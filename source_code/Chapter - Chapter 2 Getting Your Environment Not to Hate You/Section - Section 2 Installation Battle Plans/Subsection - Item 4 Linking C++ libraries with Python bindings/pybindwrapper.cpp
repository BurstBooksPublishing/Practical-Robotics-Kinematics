#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "kinematics.hpp" // declaration for compute_fk (from libkin)

namespace py = pybind11;

// thin wrapper; returns Eigen::Affine3d which pybind11 converts to numpy-friendly types.
Eigen::Affine3d compute_fk_wrapper(const Eigen::VectorXd &q) {
    return compute_fk(q); // call into prebuilt C++ library (libkin)
}

PYBIND11_MODULE(kinpy, m) {
    m.doc() = "Python bindings for C++ kinematics (libkin)"; 
    m.def("compute_fk", &compute_fk_wrapper, "Compute forward kinematics"); 
}

/* CMake hints (put in CMakeLists.txt):
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
add_library(kinpy MODULE wrapper.cpp)
target_link_libraries(kinpy PRIVATE libkin pybind11::module Eigen3::Eigen)
# Ensure module name and suffix match Python expectations (pybind11 handles this).
# Set rpath or install rules so libkin is discoverable at runtime.
*/