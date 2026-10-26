#pragma once

#include <pybind11/pybind11.h>

void bind_enums(pybind11::module_& module);
void bind_types(pybind11::module_& module);
void bind_robot(pybind11::module_& module);
void bind_logging(pybind11::module_& module);
