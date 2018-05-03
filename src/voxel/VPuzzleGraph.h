//
// Created by ziqwang on 19.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VPUZZLEGRAPH_H
#define UNDERSTAND_INTERLOCK_VPUZZLEGRAPH_H

#include "VoxelizedPart.h"
#include "PuzzleCandidateFilter.h"

#include <iostream>
#include "stdlib.h"
#include <queue>
#include <stack>
#include <map>
#include <unordered_map>
#include "../graph/DirectedGraph.h"
#include "../FrameInterface.h"

//#define VoxelPartitionSearchExhaustion
#define VoxelPartitiomSearchRandom

enum VoxelsListType
{
    NEW_PART = 0,
    REMAIN_PART = 1,
    BOTH_PART = 2,
};


struct VPuzzleGraphInput
{
public:

    DirectedGraph graph[3];

    FramePillar* cpillar;

    vector<pEmt> voxels_must_be_new_part;

    vector<pEmt> voxels_must_be_remain_part;

    int num_pillar;

public:

    vector<int> in_ID_[3];
    vector<VoxelsList *> in_voxels[3];
    vector<VoxelsListType> in_voxels_type_[3];

    vector<bool> out_ID_[3];
    vector<VoxelsList *> out_voxels[3];
    vector<VoxelsListType> out_voxels_type[3];
};

using Eigen::MatrixXi;

class VPuzzleGraph {
public:

    VPuzzleGraph(){
        max_double_loops_partition_plan = 10000;
        max_single_loop_partition_plan = 10000;
        max_joint_combination = 10000;
        max_voxel_combination = 10;
        max_failure_time = 10;
    }

public:

    void set_input(const VPuzzleGraphInput &input) {input_ = input;}

    void set_size(Vector3i size){size_ = size;}

public:

    void compute_distance_graph(int XYZ);

    void compute_input_in_out_data(int XYZ);

    void compute_input_in_data(int XYZ,
                               const DirectedGraphEdge& edge,
                               const vector<bool>& partition_joints);

    void compute_input_out_data(int XYZ,
                                const DirectedGraphEdge& edge,
                                const vector<bool>& partition_joints);

    VoxelsListType check_voxel_type(pEmt em);

public:

    void compute_one_loop_candidate_voxels(int XYZ);

    void compute_two_loops_candidate_voxels(int XYZ);

    void add_into_in_out_filter(int XYZ, int i0, int i1, int P0, int P1, vector<VPuzFECycle> &list_in_out);

    void add_into_out_in_filter(int XYZ, int i0, int i1, int P0, int P1, vector<VPuzFECycle> &list_in_out);

    void add_into_double_filter(int XYZ, int i0, int i1, int i2, int i3, int P0, int P1, int P2, int P3, vector<VPuzFECycle> &list_double);

public:

    void compute_combination_of_cycleXYZ(vector<VPuzCycleXYZDat> &plans_cycleXYZ);

    void compute_remain_volume_partition_plans(vector<VPuzRemainVolumePartitionDat> &voxel_partition_plan,
                                               const vector<VPuzCycleXYZDat> &combination_graph_partition_plan);

    void do_separation(vector<VPuzRemainVolumePartitionDat> &output);

public:

    bool random_compute_voxel_partition_plan(VPuzRemainVolumePartitionDat &plan,
                                             vector<vector<VPuzFE<VoxelElement *>>> *gr,
                                             vector<int> &require_blocking);

    void random_pick_voxels_in_a_group(vector<vector<VPuzFE<VoxelElement *>>> &gr,
                                       vector<VoxelElement *> &gvoxel);

public:

    void exhaust_compute_voxel_partition_plan(vector<VPuzRemainVolumePartitionDat> &plans,
                                              vector<vector<VPuzFE<VoxelElement *>>> *gr,
                                                  int relation);

    void dfs_compute_voxel_partition_plan(vector<VPuzRemainVolumePartitionDat> &plans,
                                          vector<vector<VPuzFE<VoxelElement *>>> *gr,
                                          int relation);

public:

    void remove_duplicate(vector<pEmt> &group_voxels);

public:

    VPuzzleGraphInput input_;

    MatrixXi dist_graph[3];

    Vector3i size_;

    vector<std::shared_ptr<VoxelsList>> voxel_lists;

public:

    VPuzFilterCycle graph_partition_plan_in_out[3];

    VPuzFilterCycle graph_partition_plan_out_in[3];

    VPuzFilterCycle graph_partition_plan_double[3];

private:

    int max_double_loops_partition_plan;

    int max_single_loop_partition_plan;

    int max_joint_combination;

    int max_voxel_combination;

    int max_failure_time;
};


#endif //UNDERSTAND_INTERLOCK_VPUZZLEGRAPH_H
