//
// Created by ziqwang on 28.04.18.
//

#include "FrameInterlockingTree.h"
#include <cmath>

void FrameInterlockingTree::accept_partition_plan(TreeNode *node,
                                                  FramePillar *cpillar,
                                                  const vector<FinalPartiResult>& final_parti,
                                                  int *part_num_voxels,
                                                  Vector3d direction)
{
    //Puzzle
    std::shared_ptr<TreeNode> child_node = std::make_shared<TreeNode>();

    child_node->puzzles = node->puzzles;
    child_node->num_voxel_left_in_joints = node->num_voxel_left_in_joints;
    for(int kd = 0; kd < 2; kd++)
    {
        int joint_id = cpillar->cube_id[kd];
        std::shared_ptr<VoxelizedPuzzle> child_puzzle =
                std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(*node->puzzles[joint_id]));
        if(kd == 0)
            child_puzzle->partition_part(final_parti[0].new_part_voxels, cpillar->index);
        else
            child_puzzle->partition_part(final_parti[1].new_part_voxels, cpillar->index);
        child_node->puzzles[joint_id] = child_puzzle;

        child_node->num_voxel_left_in_joints[cpillar->cube_id[kd]] -= part_num_voxels[kd];
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
    child_node->pillar_visited_order = node->pillar_visited_order;
    child_node->disassembly_order = node->disassembly_order;
    child_node->disassembling_directions = node->disassembling_directions;

    child_node->pillar_visited_order[cpillar->index] = child_node->num_pillar_finished - 1;
    child_node->disassembly_order.push_back(cpillar);
    child_node->disassembling_directions.push_back(direction);

    //push into node
    node->children.push_back(child_node);

    for(int id = 0; id < child_node->disassembly_order.size(); id++)
        std::cout << child_node->disassembly_order[id]->index <<", ";
    std::cout << std::endl;
}

void FrameInterlockingTree::sort_children(TreeNode *node)
{
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


void FrameInterlockingTree::generate_key_plan(TreeNode *node,
                                              FramePillar *key_pillar,
                                              vector<VPuzRemainVolumePartitionDat> &concept_plan)
{

    //add already exist pillars
    std::map<int, int> already_exist_pillars;
    already_exist_pillars.insert(std::make_pair(key_pillar->index, true));

    //get voxels which must belong to new/remain part
    vecVector3i remain_fixed_voxels[2];
    vecVector3i new_fixed_voxels[2];
    for (int kd = 0; kd < 2; kd++)
    {
        get_pillars_fixed_voxels(new_fixed_voxels[kd],
                                 remain_fixed_voxels[kd],
                                 key_pillar->cube_id[kd],
                                 key_pillar->index,
                                 already_exist_pillars);
    }

    //set key relation
    int relation = 0b110111;

    //compute key conceptual partition plan
    for (int kd = 0; kd < 2; kd++)
    {
        VoxelizedPuzzle *puzzle = node->puzzles[key_pillar->cube_id[kd]].get();
        VPuzRemainVolumePartitionDat plan;

        for (int id = 0; id < new_fixed_voxels[kd].size(); id++)
            plan.groupA.push_back(puzzle->map_ip_[puzzle->V2I(new_fixed_voxels[kd][id])]);

        for (int id = 0; id < remain_fixed_voxels[kd].size(); id++)
            plan.groupB.push_back(puzzle->map_ip_[puzzle->V2I(remain_fixed_voxels[kd][id])]);

        plan.relation = relation;
        concept_plan.push_back(plan);
    }
}

bool FrameInterlockingTree::generate_key(TreeNode *node)
{
    if(node && !node->candidate_pillar.empty()) {
        //key only have one candidate_pillar usually
        FramePillar *key_pillar = node->candidate_pillar.front();

        vector<VPuzRemainVolumePartitionDat> concept_plan;
        generate_key_plan(node, key_pillar, concept_plan);

        int part_num_voxels[2];
        int voxel_num = interface->voxel_num_;
        vector<FinalPartiResult> final_parti_result[2];
        for (int kd = 0; kd < 2; kd++) {
            VoxelizedPuzzle *puzzle = node->puzzles[key_pillar->cube_id[kd]].get();

            part_num_voxels[kd] =
                    voxel_num * voxel_num * voxel_num / map_joint_pillars_[key_pillar->cube_id[kd]].size();
            VoxelizedPartition partioner(part_num_voxels[kd], part_num_voxels[kd]);
            partioner.maximum_inner_partition_plan_ = 1000;

            partioner.input(puzzle, concept_plan[kd], posY);
            partioner.output(final_parti_result[kd]);
        }

        for (int id = 0; id < final_parti_result[0].size(); id++)
        {
            for (int jd = 0; jd < final_parti_result[1].size(); jd++)
            {
                vector<FinalPartiResult> final_parti;
                final_parti.push_back(final_parti_result[0][id]);
                final_parti.push_back(final_parti_result[1][jd]);
                accept_partition_plan(node, key_pillar, final_parti, part_num_voxels, Vector3d(0, 1, 0));

            }
        }
        sort_children(node);
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
                sort_children(node);
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
        int part_num_voxels[2];

        vector<int> legal_nrm;
        compute_legal_disassembling_direction(node, cpillar, legal_nrm);

        for(int id = 0; id < concept_partition_plans.size(); id++)
        {
            VPuzRemainVolumePartitionDat concept = concept_partition_plans[id];
            VPuzRemainVolumePartitionDat plan[2];
            vector<FinalPartiResult> final_parti_result[2][6];
            for(int kd = 0; kd < 2; kd++)
            {
                seperate_concept_design(kd, concept, plan[kd], cpillar);

                VoxelizedPuzzle* puzzle = node->puzzles[cpillar->cube_id[kd]].get();
                part_num_voxels[kd] = voxel_num * voxel_num * voxel_num/ map_joint_pillars_[cpillar->cube_id[kd]].size();

                if(2 * part_num_voxels[kd] > node->num_voxel_left_in_joints[cpillar->cube_id[kd]]) {
                    part_num_voxels[kd] = node->num_voxel_left_in_joints[cpillar->cube_id[kd]];
                    for(int nrm = 0; nrm < 6; nrm++)
                    {
                        if(legal_nrm[nrm])
                        {
                            FinalPartiResult result;
                            result.new_part_voxels = puzzle->parts_.back()->elist_;
                            final_parti_result[kd][nrm].push_back(result);
                        }

                    }
                }
                else
                {
                    for(int nrm = 0; nrm < 6; nrm++)
                    {
                        if(legal_nrm[nrm])
                        {
                            VoxelizedPartition partioner(part_num_voxels[kd], part_num_voxels[kd]);
                            partioner.input(puzzle, plan[kd],AssemblingDirection(nrm));
                            partioner.output(final_parti_result[kd][nrm]);
                        }
                    }

                }
            }

            int dX[6] = {1, -1, 0, 0, 0, 0};
            int dY[6] = {0, 0, 1, -1, 0, 0};
            int dZ[6] = {0, 0, 0, 0, 1, -1};

            for(int nrm = 0; nrm < 6; nrm++)
            {
                for(int id = 0; id < final_parti_result[0][nrm].size(); id++)
                {
                    for(int jd = 0; jd < final_parti_result[1][nrm].size(); jd++)
                    {
                        vector<FinalPartiResult> final_parti;
                        final_parti.push_back(final_parti_result[0][nrm][id]);
                        final_parti.push_back(final_parti_result[1][nrm][jd]);
                        accept_partition_plan(node, cpillar, final_parti, part_num_voxels, Vector3d(dX[nrm], dY[nrm], dZ[nrm]));
                        if(node->children.size() > 1)
                            return true;
                    }
                }
            }
        }
    }
    if(node->children.empty())
        return false;
    else
        return true;
}

void FrameInterlockingTree::compute_legal_disassembling_direction(TreeNode *node,
                                                                  FramePillar *pillar,
                                                                  vector<int> &legal_nrm) {
    legal_nrm.clear();
    legal_nrm.resize(6, true);

    auto opposite_nrm = [](int nrm) -> int
    {
        int XYZ = nrm / 2;
        int sign = nrm % 2;
        if(sign) sign = 0;
        else sign = 1;
        return 2*XYZ + sign;
    };

    for(int kd = 0; kd < 2; kd++)
    {
        int joint_id = pillar->cube_id[kd];
        bool has_neighbor = false;
        for(int id = 0; id < map_joint_pillars_[joint_id].size(); id++)
        {
            FramePillar *neighbor = map_joint_pillars_[joint_id][id];
            int nindex = neighbor->index;
            if(nindex == pillar->index) continue;
            if(node->pillar_visited_order.find(nindex) == node->pillar_visited_order.end())
            {
                has_neighbor = true;
                for(int jd = 0; jd < 2; jd++)
                {
                    if(neighbor->cube_id[jd] == joint_id)
                    {
                        legal_nrm[neighbor->pos_in_cube_face[jd]] = false;
                        break;
                    }
                }
            }
        }
        if(has_neighbor)
            legal_nrm[opposite_nrm(pillar->pos_in_cube_face[kd])] = false;
    }
    return;
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
    int part_num_voxels[2];
    for(int kd = 0; kd < 2; kd++) {
        VoxelizedPuzzle *puzzle = node->puzzles[cpillar->cube_id[kd]].get();
        //create plan
        part_num_voxels[kd] = std::powl(interface->voxel_num_, 3)/ map_joint_pillars_[cpillar->cube_id[kd]].size();

        if(2 * part_num_voxels[kd] > node->num_voxel_left_in_joints[cpillar->cube_id[kd]])
        {
            voxel_must_be_new_part = puzzle->parts_.back()->elist_;
        }
        else
        {
            for (int id = 0; id < new_fixed_voxels[kd].size(); id++) {
                voxel_must_be_new_part.push_back(puzzle->map_ip_[puzzle->V2I(new_fixed_voxels[kd][id])]);
            }
        }

        for (int id = 0; id < remain_fixed_voxels[kd].size(); id++) {
            voxel_must_be_remain_part.push_back(puzzle->map_ip_[puzzle->V2I(remain_fixed_voxels[kd][id])]);
        }
    }

    input.voxels_must_be_new_part = voxel_must_be_new_part;
    input.voxels_must_be_remain_part = voxel_must_be_remain_part;
    input.pillar_in_orders = node->pillar_visited_order;

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
        DirectedGraphNode* u = in_graph.nodeLists_[id].get();
        for(int jd = 0; jd < u->neighborList_.size(); jd++)
        {
            DirectedGraphNode* v = u->neighborList_[jd].node.lock().get();
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

    //remain -> out
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
                out_graph.add_edge(new_index, v->index_);
            }
        }
    }

    //out -> remain
    for(int id = 0; id < in_graph.nodeLists_.size(); id++)
    {
        DirectedGraphNode* v = in_graph.nodeLists_[id].get();
        for(int kd = 0 ;kd < v->neighborList_.size(); kd++)
        {
            DirectedGraphEdge edge = v->neighborList_[kd];
            if(edge.node.lock()->index_ == u->index_)
            {
                std::shared_ptr<vector<pEmt>> plist = edge.list;
                for(int jd = 0; jd < plist->size(); jd++)
                {
                    pEmt pre_voxel = (*plist)[jd];
                    pEmt voxel = node->puzzles[pre_voxel->joint_id_]->map_ip_[pre_voxel->order_];
                    if(voxel->gid_ == num_pillar)
                    {
                        //belong to remaining volume
                        out_graph.add_edge(v->index_, remain_index, voxel);
                    }
                    else
                    {
                        out_graph.add_edge(v->index_, new_index);
                    }
                }
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
                    out_graph.add_edge(new_index, remain_index, find_it->second);
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

    for(int id = 0; id < interface->pillars_.size(); id++)
    {
        if(candidates_.find(id) != candidates_.end() && cut_points.find(id) == cut_points.end())
        {
            node->candidate_pillar.push_back(interface->pillars_[id].get());
        }
    }

    return;
}








