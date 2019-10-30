project(slam_ws_apps)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/)

include_directories(include)

## APPS
add_executable(example_slam
       src/example_slam.cpp
)

target_link_libraries(example_slam
   gicp_registration
   utils_g2o
   submaps_tools
   graph_construction
   ceres_optimizer
   read_g2o
   data_transforms
   benchmark
   std_data
   slam_solver
)
