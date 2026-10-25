find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_core NB_STATIC STABLE_ABI src/bindings.cpp)
target_link_libraries(_core PRIVATE fiss_planner)

if(SKBUILD)
  install(TARGETS _core LIBRARY DESTINATION fiss_planner)
else()
  # In-tree development build: place the module next to the Python sources.
  set_target_properties(_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/fiss_planner)
endif()
