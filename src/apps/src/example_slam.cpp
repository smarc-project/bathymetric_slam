/* Copyright 2019 Ignacio Torroba (torroba@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cereal/archives/binary.hpp>

#include "data_tools/std_data.h"
#include "data_tools/benchmark.h"

#include "submaps_tools/cxxopts.hpp"
#include "submaps_tools/submaps.hpp"

#include "registration/utils_visualization.hpp"
#include "registration/gicp_reg.hpp"

#include "graph_optimization/graph_construction.hpp"
#include "graph_optimization/ceres_optimizer.hpp"

#include "bathy_slam/bathy_slam.hpp"

using namespace Eigen;
using namespace std;
using namespace g2o;

int main(int argc, char** argv){

    // Inputs
    std::string path_str, output_str, auvlib_cereal, simulation;
    cxxopts::Options options("MyProgram", "Graph SLAM solver for bathymetric maps");
    options.add_options()
        ("help", "Print help")
        ("input_dataset", "Input batymetric survey as a cereal file or folder with .pdc submaps", cxxopts::value(path_str))
        ("simulation", "(yes/no). Simulation data from Gazebo (.pdc submaps)", cxxopts::value(simulation))
        ("auvlib_cereal", "(yes/no). Cereal output from AUVLib or from this solver", cxxopts::value(auvlib_cereal))
        ("output_cereal", "Output graph in cereal format", cxxopts::value(output_str));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }
    if(output_str.empty()){
        output_str = "output_cereal.cereal";
    }
    boost::filesystem::path output_path(output_str);
    string outFilename = "graph_corrupted.g2o";   // G2O output file

    // Parse submaps from cereal file
    SubmapsVec submaps_gt;
    boost::filesystem::path submaps_path(path_str);
    std::cout << "Input data " << boost::filesystem::basename(submaps_path) << std::endl;
    if(simulation == "yes"){
        submaps_gt = readSubmapsInDir(submaps_path.string());
    }
    else{
        if(auvlib_cereal == "yes"){
            std_data::pt_submaps ss = std_data::read_data<std_data::pt_submaps>(submaps_path);
            submaps_gt = parseSubmapsAUVlib(ss);
        }
        else{
            std::ifstream is(boost::filesystem::basename(submaps_path) + ".cereal", std::ifstream::binary);
            {
              cereal::BinaryInputArchive iarchive(is);
              iarchive(submaps_gt);
            }
        }
        // Voxelization of submaps
        PointCloudT::Ptr cloud_ptr (new PointCloudT);
        pcl::UniformSampling<PointT> us_filter;
        us_filter.setInputCloud (cloud_ptr);
        us_filter.setRadiusSearch(0.1);
        for(SubmapObj& submap_i: submaps_gt){
            *cloud_ptr = submap_i.submap_pcl_;
            us_filter.setInputCloud(cloud_ptr);
            us_filter.filter(*cloud_ptr);
            submap_i.submap_pcl_ = *cloud_ptr;
        }
    }

    // Benchmark GT
    benchmark::track_error_benchmark benchmark("auv_data");
    PointsT gt_map = pclToMatrixSubmap(submaps_gt);
    PointsT gt_track = trackToMatrixSubmap(submaps_gt);
    benchmark.add_ground_truth(gt_map, gt_track);
    ceres::optimizer::saveOriginalTrajectory(submaps_gt); // Save original graph and AUV trajectory

    // Visualization
    PCLVisualizer viewer ("SLAM viewer");
    SubmapsVisualizer visualizer(viewer);

    // GICP reg for submaps
    SubmapRegistration gicp_reg;

    // Graph constructor
    GraphConstructor graph_obj;
    google::InitGoogleLogging(argv[0]);

    // Create SLAM solver and run offline
    BathySlam* slam_solver = new BathySlam(graph_obj, gicp_reg, viewer, visualizer);
    SubmapsVec submaps_reg = slam_solver->runOffline(submaps_gt);

    // Save graph to cereal file
    std::cout << "Output cereal: " << boost::filesystem::basename(output_path) << std::endl;
    std::ofstream os(boost::filesystem::basename(output_path) + ".cereal", std::ofstream::binary);
    {
        cereal::BinaryOutputArchive oarchive(os);
        oarchive(submaps_reg);
        os.close();
    }

    // Visualize optimized solution
    visualizer.plotPoseGraphCeres(submaps_reg);
    while(!viewer.wasStopped ()){
        viewer.spinOnce ();
    }
    viewer.resetStoppedFlag();

    // Benchmark Optimized
    PointsT opt_map = pclToMatrixSubmap(submaps_reg);
    PointsT opt_track = trackToMatrixSubmap(submaps_reg);
    benchmark.add_benchmark(opt_map, opt_track, "optimized");
    benchmark.print_summary();

    // Plot initial and optimized trajectories
    std::string command_str = "./plot_results.py --initial_poses poses_original.txt "
                              "--corrupted_poses poses_corrupted.txt --optimized_poses poses_optimized.txt";
    const char *command = command_str.c_str();
    system(command);

    delete(slam_solver);

    return 0;
}
