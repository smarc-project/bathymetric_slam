#include "bathy_slam/bathy_slam.hpp"


BathySlam::BathySlam(GraphConstructor &graph_obj, SubmapRegistration &gicp_reg,
                     pcl::visualization::PCLVisualizer &viewer, SubmapsVisualizer &visualizer):
                     graph_obj_(&graph_obj), gicp_reg_(&gicp_reg), viewer_(&viewer), visualizer_(&visualizer){

}

// TODO: handler pointers here
BathySlam::~BathySlam(){

}

SubmapsVec BathySlam::runOffline(SubmapsVec& submaps_gt){

    SubmapObj submap_trg;
    SubmapsVec submaps_prev, submaps_reg;
    ofstream fileOutputStream;
    fileOutputStream.open("loop_closures.txt", std::ofstream::out);
    string outFilename = "graph_init.g2o";   // G2O output file

    visualizer_->setVisualizer(submaps_gt, 1);
    while(!viewer_->wasStopped ()){
        viewer_->spinOnce ();
    }
    viewer_->resetStoppedFlag();

    for(SubmapObj& submap_i: submaps_gt){
        std::cout << " ----------- Submap " << submap_i.submap_id_ << ", swath "
                  << submap_i.swath_id_ << " ------------"<< std::endl;

        // Look for loop closures
        for(SubmapObj& submap_k: submaps_reg){
            // Don't look for overlaps between consecutive submaps
            if(submap_k.submap_id_ != submap_i.submap_id_ - 1){
                submaps_prev.push_back(submap_k);
            }
        }
        submap_i.findOverlaps(submaps_prev);
        submaps_prev.clear();

        // Create graph vertex i
        graph_obj_->createNewVertex(submap_i);

        // Create DR edge i and store (skip submap 0)
        if(submap_i.submap_id_ != 0 ){
            std::cout << "DR edge from " << submap_i.submap_id_ -1 << " to " << submap_i.submap_id_<< std::endl;
            graph_obj_->createDREdge(submap_i);
        }

        // If potential loop closure detected
        SubmapObj submap_final = submap_i;
        if(!submap_i.overlaps_idx_.empty()){
            // Save loop closure to txt
            if(fileOutputStream.is_open()){
                fileOutputStream << submap_i.submap_id_;
                for(unsigned int j=0; j<submap_i.overlaps_idx_.size(); j++){
                    fileOutputStream << " " << submap_i.overlaps_idx_.at(j);
                }
                fileOutputStream << "\n";
            }

            // Register overlapping submaps
            submap_trg = gicp_reg_->constructTrgSubmap(submaps_reg, submap_i.overlaps_idx_);
            if(gicp_reg_->gicpSubmapRegistration(submap_trg, submap_i)){
                submap_final = submap_i;
            }
            submap_trg.submap_pcl_.clear();

            // Create loop closures
            graph_obj_->findLoopClosures(submap_final, submaps_reg);
        }
        submaps_reg.push_back(submap_final);    // Add registered submap_i
    }
    fileOutputStream.close();

    // Update visualizer
    visualizer_->updateVisualizer(submaps_reg);
    while(!viewer_->wasStopped ()){
        viewer_->spinOnce ();
    }
    viewer_->resetStoppedFlag();

//    // Add noise to edges on the graph
//    GaussianGen transSampler, rotSampler;
//    Matrix<double, 6,6> information = generateGaussianNoise(transSampler, rotSampler);
//    graph_obj_->addNoiseToGraph(transSampler, rotSampler);

    // Create initial DR chain
    graph_obj_->createInitialEstimate(submaps_reg);

    // Visualize initial estimate
    visualizer_->plotPoseGraphG2O(*graph_obj_, submaps_reg);
    while(!viewer_->wasStopped ()){
        viewer_->spinOnce ();
    }
    viewer_->resetStoppedFlag();

    // Save graph to output g2o file (optimization can be run with G2O)
    graph_obj_->saveG2OFile(outFilename);

    // Optimize graph and save to cereal
    ceres::optimizer::MapOfPoses poses = ceres::optimizer::ceresSolver(outFilename, graph_obj_->drEdges_.size());
    ceres::optimizer::updateSubmapsCeres(poses, submaps_reg);

    return submaps_reg;
}
