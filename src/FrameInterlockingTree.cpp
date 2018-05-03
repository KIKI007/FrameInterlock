//
// Created by ziqwang on 28.04.18.
//

#include "FrameInterlockingTree.h"

bool FrameInterlockingTree::generate_key(TreeNode *node)
{
    if(node && !node->candidate_pillar.empty())
    {
        //key only have one candidate_pillar usually
        FramePillar* key_pillar = node->candidate_pillar.front();

        std::map<int, int> visited;
        visited.insert(std::make_pair(key_pillar->index, true));

        //get pillar's fixed voxel coordinates
        vecVector3i remain_fixed_voxels[2];
        vecVector3i new_fixed_voxels[2];
        for(int kd = 0; kd < 2; kd++)
        {
            get_pillars_fixed_voxels(new_fixed_voxels[kd],
                                     remain_fixed_voxels[kd],
                                     key_pillar->cube_id[kd],
                                     key_pillar->index,
                                     visited);
        }

        //create key
        int relation = 0b110111;
        int voxel_num = interface->voxel_num_;
        vector<FinalPartiResult> final_parti_result[2];
        for(int kd = 0; kd < 2; kd++)
        {
            VoxelizedPuzzle* puzzle = node->puzzles[key_pillar->cube_id[kd]].get();

            //create plan
            VPuzRemainVolumePartitionDat plan;
            for(int id = 0; id < new_fixed_voxels[kd].size(); id++)
            {
                plan.groupA.push_back(puzzle->map_ip_[puzzle->V2I(new_fixed_voxels[kd][id])]);
            }
            for(int id = 0; id < remain_fixed_voxels[kd].size(); id++)
            {
                plan.groupB.push_back(puzzle->map_ip_[puzzle->V2I(remain_fixed_voxels[kd][id])]);
            }
            plan.relation = relation;

            int part_num_voxels = voxel_num * voxel_num * voxel_num/ map_joint_pillars_[key_pillar->cube_id[kd]].size();
            VoxelizedPartition partioner(part_num_voxels, part_num_voxels);
            partioner.maximum_inner_partition_plan_ = 1000;

            partioner.input(puzzle, plan);
            partioner.output(final_parti_result[kd]);
        }

        for(int id = 0; id < final_parti_result[0].size(); id++)
        {
            for(int jd = 0; jd < final_parti_result[1].size(); jd++)
            {
                if(final_parti_result[0][id].direction == final_parti_result[1][jd].direction)
                {

                    //Puzzle
                    std::shared_ptr<TreeNode> child_node = std::make_shared<TreeNode>();

                    child_node->puzzles = node->puzzles;
                    for(int kd = 0; kd < 2; kd++)
                    {
                        int joint_id = key_pillar->cube_id[kd];
                        std::shared_ptr<VoxelizedPuzzle> child_puzzle =
                                std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(*node->puzzles[joint_id]));
                        if(kd == 0)
                            child_puzzle->partition_part(final_parti_result[0][id].new_part_voxels, key_pillar->index);
                        else
                            child_puzzle->partition_part(final_parti_result[1][jd].new_part_voxels, key_pillar->index);
                        child_node->puzzles[joint_id] = child_puzzle;
                    }

                    //Relation
                    child_node->parent = node;
                    child_node->num_pillar_finished = node->num_pillar_finished + 1;
                    child_node->brother = nullptr;

                    //Graph
                    for(int kd = 0; kd < 3; kd++)
                    {
                        child_node->graph[kd] = std::make_shared<DirectedGraph>();
                        split_graph(*node->graph[kd], *child_node->graph[kd], key_pillar, child_node.get(), kd);
                    }

                    //order
                    child_node->pillar_visited_order[key_pillar->index] = child_node->num_pillar_finished - 1;
                    child_node->disassembly_order.push_back(key_pillar);


                    //push into node
                    node->children.push_back(child_node);
                }
            }
        }

        if(node->children.size() > 0)
        {
            TreeNode* last_child_node = node->children.back().get();
            select_candidates(last_child_node);
            for(int id = 0; id < node->children.size() - 1; id++)
            {
                node->children[id]->brother = node->children[id + 1].get();

                //select candidate
                node->children[id]->candidate_pillar = last_child_node->candidate_pillar;
            }
            node->children.back()->brother = nullptr;
        }
    }

    if(node->children.size() > 0)
        return true;
    else
        return false;
}


bool FrameInterlockingTree::generate_children(TreeNode *node)
{
    if(node->num_pillar_finished == 0)
    {
        return generate_key(node);
    }
    else
    {
        for(int id = 0 ;id < node->candidate_pillar.size(); id++)
        {
            if(generate_children(node, node->candidate_pillar[id]))
            {
                return true;
            }
        }
        return false;
    }
}

void FrameInterlockingTree::seperate_concept_design(int kd,
                                                    VPuzRemainVolumePartitionDat &concept,
                                                    VPuzRemainVolumePartitionDat &plan,
                                                    FramePillar *cpillar)
{
    plan.relation = concept.relation;
    //group A
    for(int jd = 0; jd < concept.groupA.size(); jd++)
    {
        pEmt em = concept.groupA[jd];
        if(em->joint_id_ == cpillar->cube_id[kd])
            plan.groupA.push_back(em);
    }

    //group B
    for(int jd = 0; jd < concept.groupB.size(); jd++)
    {
        pEmt em = concept.groupB[jd];
        if(em->joint_id_ == cpillar->cube_id[kd])
            plan.groupB.push_back(em);
    }
}


bool FrameInterlockingTree::generate_children(TreeNode *node, FramePillar *cpillar) {

    if(node) {
        VPuzzleGraph graph;

        //generate graph
        generate_vpuzzlegraph(graph, node, cpillar);

        vector<VPuzRemainVolumePartitionDat> concept_partition_plans;
        graph.do_separation(concept_partition_plans);

        int voxel_num = interface->voxel_num_;

        for(int id = 0; id < concept_partition_plans.size(); id++)
        {

            VPuzRemainVolumePartitionDat concept = concept_partition_plans[id];
            VPuzRemainVolumePartitionDat plan[2];
            vector<FinalPartiResult> final_parti_result[2];
            for(int kd = 0; kd < 2; kd++)
            {
                seperate_concept_design(kd, concept, plan[kd], cpillar);

                VoxelizedPuzzle* puzzle = node->puzzles[cpillar->cube_id[kd]].get();
                int part_num_voxels = voxel_num * voxel_num * voxel_num/ map_joint_pillars_[cpillar->cube_id[kd]].size();

                VoxelizedPartition partioner(part_num_voxels, part_num_voxels);
                partioner.input(puzzle, plan[kd]);
                partioner.output(final_parti_result[kd]);
            }

            for(int id = 0; id < final_parti_result[0].size(); id++)
            {
                for(int jd = 0; jd < final_parti_result[1].size(); jd++)
                {
                    if(final_parti_result[0][id].direction == final_parti_result[1][jd].direction)
                    {

                        //Puzzle
                        std::shared_ptr<TreeNode> child_node = std::make_shared<TreeNode>();

                        child_node->puzzles = node->puzzles;
                        for(int kd = 0; kd < 2; kd++)
                        {
                            int joint_id = cpillar->cube_id[kd];
                            std::shared_ptr<VoxelizedPuzzle> child_puzzle =
                                    std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(*node->puzzles[joint_id]));
                            if(kd == 0)
                                child_puzzle->partition_part(final_parti_result[0][id].new_part_voxels, cpillar->index);
                            else
                                child_puzzle->partition_part(final_parti_result[1][jd].new_part_voxels, cpillar->index);
                            child_node->puzzles[joint_id] = child_puzzle;
                        }

                        //Relation
                        child_node->parent = node;
                        child_node->num_pillar_finished = node->num_pillar_finished + 1;
                        child_node->brother = nullptr;

                        //Graph
                        for(int kd = 0; kd < 3; kd++)
                        {
                            child_node->graph[kd] = std::make_shared<DirectedGraph>();
                            split_graph(*node->graph[kd], *child_node->graph[kd], cpillar, child_node.get(), kd);
                        }

                        //order
                        child_node->pillar_visited_order[cpillar->index] = child_node->num_pillar_finished - 1;
                        child_node->disassembly_order.push_back(cpillar);


                        //push into node
                        node->children.push_back(child_node);
                    }
                }
            }
        }
    }

    if(node->children.size() > 0)
    {
        TreeNode* last_child_node = node->children.back().get();
        select_candidates(last_child_node);
        for(int id = 0; id < node->children.size() - 1; id++)
        {
            node->children[id]->brother = node->children[id + 1].get();

            //select candidate
            node->children[id]->candidate_pillar = last_child_node->candidate_pillar;
        }
        node->children.back()->brother = nullptr;
    }

    if(node->children.size() > 0)
        return true;
    else
        return false;
}

void FrameInterlockingTree::generate_vpuzzlegraph(VPuzzleGraph &graph, TreeNode *node, FramePillar *cpillar)
{
    VPuzzleGraphInput input;
    for(int kd = 0; kd < 3; kd++)
        input.graph[kd] = *node->graph[kd];

    input.num_pillar = interface->pillars_.size();
    input.cpillar = cpillar;

    std::map<int, int> visited;
    for(int id = 0; id < node->disassembly_order.size(); id++)
    {
        visited.insert(std::make_pair(node->disassembly_order[id]->index, true));
    }
    visited.insert(std::make_pair(cpillar->index, true));

    //get pillar's fixed voxel coordinates
    vecVector3i remain_fixed_voxels[2];
    vecVector3i new_fixed_voxels[2];
    for(int kd = 0; kd < 2; kd++)
    {
        get_pillars_fixed_voxels(new_fixed_voxels[kd],
                                 remain_fixed_voxels[kd],
                                 cpillar->cube_id[kd],
                                 cpillar->index,
                                 visited);
    }

    vector<pEmt> voxel_must_be_new_part;
    vector<pEmt> voxel_must_be_remain_part;
    for(int kd = 0; kd < 2; kd++) {
        VoxelizedPuzzle *puzzle = node->puzzles[cpillar->cube_id[kd]].get();
        //create plan
        for (int id = 0; id < new_fixed_voxels[kd].size(); id++) {
            voxel_must_be_new_part.push_back(puzzle->map_ip_[puzzle->V2I(new_fixed_voxels[kd][id])]);
        }
        for (int id = 0; id < remain_fixed_voxels[kd].size(); id++) {
            voxel_must_be_remain_part.push_back(puzzle->map_ip_[puzzle->V2I(remain_fixed_voxels[kd][id])]);
        }
    }

    input.voxels_must_be_remain_part = voxel_must_be_new_part;
    input.voxels_must_be_remain_part = voxel_must_be_remain_part;

    graph.set_input(input);
}

std::shared_ptr<FrameInterface> FrameInterlockingTree::output_frame(TreeNode *node) {

    std::shared_ptr<FrameInterface> frame_interface = std::make_shared<FrameInterface>(FrameInterface(*interface));

    int num_joints = node->puzzles.size();

    for(int id = 0;id < num_joints; id++)
    {
        if(node->puzzles[id])
        {
            frame_interface->joint_voxels_[id] = node->puzzles[id]->output_assembly_interface(false, false);
        }
    }

    return frame_interface;
}

void FrameInterlockingTree::get_pillars_fixed_voxels(vecVector3i &new_fixed_voxels,
                                                     vecVector3i &remain_fixed_voxels,
                                                     int joint_id,
                                                     int new_part_id,
                                                     const std::map<int, int> &visited)
{
    VoxelizedInterface *joint = interface->joint_voxels_[joint_id].get();
    int pillar_num = interface->pillars_.size();
    remain_fixed_voxels.clear();
    new_fixed_voxels.clear();
    for(int ix = 0; ix < interface->voxel_num_; ix++)
    {
        for(int iy = 0; iy < interface->voxel_num_; iy++)
        {
            for(int iz = 0; iz < interface->voxel_num_; iz++)
            {
                int vpart_id = joint->get_grid_part_index(Vector3i(ix, iy, iz));
                if(vpart_id >= pillar_num || vpart_id < 0) continue;
                auto find_it = visited.find(vpart_id);
                if(find_it == visited.end())
                {
                    remain_fixed_voxels.push_back(Vector3i(ix, iy, iz));
                }
                else if(vpart_id == new_part_id)
                {
                    new_fixed_voxels.push_back(Vector3i(ix, iy, iz));
                }
            }
        }
    }
    return;
}

void FrameInterlockingTree::split_graph(const DirectedGraph &in_graph,
                                        DirectedGraph &out_graph,
                                        FramePillar* pillar,
                                        TreeNode *node,
                                        int nrm)
{
    //initial out_graph
    out_graph.nodeLists_.clear();
    out_graph.init(in_graph.nodeLists_.size() + 1);

    //copy in_graph excpet last node info
    int graph_node_num = in_graph.nodeLists_.size() - 1; //exclude the last node
    for(int id = 0; id < graph_node_num; id++)
    {
        auto u = in_graph.nodeLists_[id];
        for(int jd = 0; jd < u->neighborList_.size(); jd++)
        {
            auto v = u->neighborList_[jd].node.lock();
            if(v->index_ < graph_node_num)
            {
                out_graph.add_edge(u->index_, v->index_);
            }
        }
    }

    //copy last node info
    auto u = in_graph.nodeLists_.back();
    int new_index = in_graph.nodeLists_.size() - 1;
    int remain_index = in_graph.nodeLists_.size();
    int num_pillar = interface->pillars_.size();
    for(int id = 0; id < u->neighborList_.size(); id++)
    {
        auto v = u->neighborList_[id].node.lock();
        std::shared_ptr<vector<pEmt>> plist = u->neighborList_[id].list;
        for(int jd = 0; jd < plist->size(); jd++)
        {
            pEmt pre_voxel = (*plist)[jd];
            pEmt voxel = node->puzzles[pre_voxel->joint_id_]->map_ip_[pre_voxel->order_];
            if(voxel->gid_ == num_pillar)
            {
                //belong to remaining volume
                out_graph.add_edge(remain_index, v->index_, voxel);
            }
            else
            {
                //belong to new part
                out_graph.add_edge(new_index, v->index_, voxel);
            }
        }
    }

    //create info between new part and remaining volume
    int dX[3] = {1, 0, 0};
    int dY[3] = {0, 1, 0};
    int dZ[3] = {0, 0, 1};
    for(int kd = 0; kd < 2; kd++)
    {
        VoxelizedPuzzle *puzzle = node->puzzles[pillar->cube_id[kd]].get();

        //new -> remain
        for(shared_pEmt new_voxel: puzzle->voxel_)
        {
            if(new_voxel->gid_ == pillar->index)
            {
                Vector3i rpos = new_voxel->pos_ - Vector3i(dX[nrm], dY[nrm], dZ[nrm]);
                auto find_it = puzzle->map_ip_.find(puzzle->V2I(rpos));
                if(find_it == puzzle->map_ip_.end()) continue;
                //std::cout << find_it->second->gid_ << std::endl;
                if(find_it->second->gid_ == num_pillar)
                {
                    out_graph.add_edge(new_index, remain_index, new_voxel.get());
                }
            }
        }


        //remain -> new
        for(shared_pEmt remain_voxel: puzzle->voxel_)
        {
            if(remain_voxel->gid_ == num_pillar)
            {
                Vector3i npos = remain_voxel->pos_ - Vector3i(dX[nrm], dY[nrm], dZ[nrm]);
                auto find_it = puzzle->map_ip_.find(puzzle->V2I(npos));
                if(find_it == puzzle->map_ip_.end()) continue;
                if(find_it->second->gid_ == pillar->index)
                {
                     out_graph.add_edge(remain_index, new_index, remain_voxel.get());
                }
            }
        }

    }

    return;
}

void FrameInterlockingTree::select_candidates(TreeNode *node)
{
    UndirectedGraph graph(interface->pillars_.size());
    std::map<int, bool> candidates_;

    //add all possible candidate
    for(int id = 0; id < node->disassembly_order.size(); id++)
    {
        FramePillar *pillar = node->disassembly_order[id];
        for(int kd = 0; kd < 2; kd++)
        {
            int cube_id = pillar->cube_id[kd];
            for(FramePillar *v : map_joint_pillars_[cube_id])
            {
                if(node->pillar_visited_order.find(v->index) == node->pillar_visited_order.end())
                {
                    candidates_.insert(std::make_pair(v->index, true));
                }
            }
        }
    }

    //build graph
    for(int id = 0; id < map_joint_pillars_.size(); id++)
    {
        for(int jd = 0; jd < map_joint_pillars_[id].size(); jd++)
        {
            int u = map_joint_pillars_[id][jd]->index;
            if(node->pillar_visited_order.find(u) != node->pillar_visited_order.end()) continue;
            for(int kd = 0; kd < map_joint_pillars_[id].size(); kd++)
            {
                int v = map_joint_pillars_[id][kd]->index;
                if(node->pillar_visited_order.find(v) != node->pillar_visited_order.end()) continue;
                if(u < v)
                {
                    graph.add_edge(u, v);
                }
            }
        }
    }

    std::map<int, bool> cut_points;
    for(int id = 0; id < node->disassembly_order.size(); id++)
    {
        cut_points.insert(std::make_pair(node->disassembly_order[id]->index, true));
    }
    graph.tarjan_cut_points(cut_points);

    for(auto em: candidates_)
    {
        if(cut_points.find(em.first) == cut_points.end())
        {
            node->candidate_pillar.push_back(interface->pillars_[em.first].get());
        }
    }

    return;
}



