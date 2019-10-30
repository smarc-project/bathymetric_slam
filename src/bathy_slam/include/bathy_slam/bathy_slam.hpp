#ifndef BATHY_SLAM_HPP
#define BATHY_SLAM_HPP

#include "submaps_tools/submaps.hpp"

#include "graph_optimization/graph_construction.hpp"
#include "graph_optimization/utils_g2o.hpp"
#include "graph_optimization/ceres_optimizer.hpp"

#include "registration/gicp_reg.hpp"
#include "registration/utils_visualization.hpp"

class BathySlam{

public:

    GraphConstructor* graph_obj_;
    SubmapRegistration* gicp_reg_;
    PCLVisualizer* viewer_;
    SubmapsVisualizer* visualizer_;

    BathySlam(GraphConstructor& graph_obj, SubmapRegistration& gicp_reg,
              PCLVisualizer& viewer, SubmapsVisualizer& visualizer);
    ~BathySlam();

    SubmapsVec runOffline(SubmapsVec &submaps_gt);
};


#endif //BATHY_SLAM_HPP
