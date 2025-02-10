find_package(pybind11 REQUIRED)
pybind11_add_module(_accurate_ri accurate_ri/bindings.cpp)
target_link_libraries(_accurate_ri PRIVATE accurate_ri)
