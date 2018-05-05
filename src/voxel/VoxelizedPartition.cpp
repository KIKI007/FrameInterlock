//
// Created by ziqwang on 06.04.18.
//

#include "VoxelizedPartition.h"

VoxelizedPartition::VoxelizedPartition(int minimum, int maximum)
{
    minimum_voxel_in_one_piece = minimum;
    maximum_voxel_in_one_piece = maximum < 0 ? std::numeric_limits<int>::max() : maximum;

    maximum_inner_partition_plan_ = 1000;

    maximum_final_plan = 1000;

    maximum_failure_time_ = 1;
}

void VoxelizedPartition::input(VoxelizedPuzzle *puzzle, VPuzRemainVolumePartitionDat &outside, AssemblingDirection direction)
{
    puzzle_ = puzzle;
    outside_partition_dat_ = outside;
    part_disassembly_direction_ = direction;
}

void VoxelizedPartition::output(vector<FinalPartiResult> &results) {

    //std::cout << "find_inner_partition" << std::endl;

    find_inner_partition(outside_partition_dat_.relation);

    VPuzFilter<FinalPartiResult> filter;
    vector<VPuzFE<FinalPartiResult>> candidate_list;

    for(VPuzInnerPartitionDat inner_dat : inside_partition_dats_)
    {
        group_A_.clear();
        in_group_A_.clear();
        time_t sta = std::clock();
        VPuzRemainVolumePartitionDat full_partition_dat;

        int disassembled_direction;

        merge_inside_outside_partition_dat(inner_dat, full_partition_dat);

        if(!connect_group_A_anchor_voxels(full_partition_dat)) continue;

        if(!maintain_disassemblability(outside_partition_dat_.relation, full_partition_dat, disassembled_direction)) continue;

        if(!expand_group_A_to_minimum_pieces(full_partition_dat, disassembled_direction)) continue;

        if(group_A_.size() >= minimum_voxel_in_one_piece && group_A_.size() <= maximum_voxel_in_one_piece)
        {
            VPuzFE <FinalPartiResult> ve;
            if(compute_remaining_volume(ve.data_.remain_part_voxels))
            {
                ve.data_.new_part_voxels = group_A_;
                ve.data_.direction = disassembled_direction;
                ve.weight = 0;
                for(int id = 0; id < group_A_.size(); id++)
                    ve.weight -= puzzle_->get_accessibility(group_A_[id]->order_);
                candidate_list.push_back(ve);
            }
        }
        if(candidate_list.size() > 2 * maximum_final_plan)
            break;
    }
    filter.max_candidates_num_ = maximum_final_plan;
    filter.insert(candidate_list);
    for(auto element : filter.candidate_)
    {
        results.push_back(element.data_);
    }
    return;
}

void VoxelizedPartition::find_inner_partition(int relation)
{
    VoxelizedPart* part = puzzle_->parts_.back().get();
    unordered_map<int, int> map_visited;
    for(pEmt em : outside_partition_dat_.groupA) map_visited.insert(std::make_pair(em->order_, 1));
    for(pEmt em : outside_partition_dat_.groupB) map_visited.insert(std::make_pair(em->order_, -1));

    auto compute_fixed_voxels = [&](Vector3i pos, bool &is_success, VPuzInnerPartitionDat &partition)
    {
        pEmt nv = part->in_part(pos);
        if(nv != nullptr)
        {
            unordered_map<int, int>::iterator find_it = map_visited.find(nv->order_);
            if(find_it != map_visited.end() && find_it->second == 1)
            {
                is_success = false;
            }
            else
            {
                partition.fixs.push_back(nv);
                is_success = true;
            }
        }
        else
        {
            is_success = false;
        }
    };

    int dX[3] = {1, 0, 0};
    int dY[3] = {0, 1, 0};
    int dZ[3] = {0, 0, 1};


    for(int id = 0; id < part->elist_.size(); id++)
    {
        pEmt voxel1 = part->elist_[id];
        auto find_it1 = map_visited.find(voxel1->order_);
        if(find_it1 != map_visited.end() && find_it1->second == -1) continue;
        for(int jd = id ; jd < part->elist_.size(); jd++)
        {
            pEmt voxel2 = part->elist_[jd];
            auto find_it2= map_visited.find(voxel2->order_);
            if(find_it2 != map_visited.end() && find_it2->second == -1) continue;

            VPuzInnerPartitionDat partition;
            bool is_success = false;

            partition.anchors.push_back(voxel1);
            if(id != jd) partition.anchors.push_back(voxel2);
            vector<pEmt> anchors; anchors = partition.anchors;
            unordered_map<int, int> tmp_map_visited = map_visited;
            map_visited.insert(std::make_pair(voxel1->order_, 1));
            map_visited.insert(std::make_pair(voxel2->order_, 1));

            for(int XYZ = 0; XYZ < 3; XYZ ++) {
                int state = (relation) >> (2 * XYZ);
                for (int kd = 0; kd < anchors.size(); kd++)
                {
                    Vector3i pos = anchors[kd]->pos_;
                    VPuzInnerPartitionDat tmp_parition = partition;
                    switch(state & 0b11)
                    {
                        case PuzzleConnection_IN_OUT:
                        {
                            pos -= Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                            compute_fixed_voxels(pos, is_success, tmp_parition);
                            break;
                        }
                        case PuzzleConnection_OUT_IN:
                        {
                            pos += Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                            compute_fixed_voxels(pos, is_success, tmp_parition);
                            break;
                        }
                        case PuzzleConnection_NULL:
                        {
                            pos += Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                            compute_fixed_voxels(pos, is_success, tmp_parition);
                            if(!is_success) break;
                            pos -= 2 * Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                            compute_fixed_voxels(pos, is_success, tmp_parition);
                            break;
                        }
                        default:
                        {
                            is_success = true;
                            break;
                        }
                    }
                    if(is_success)
                    {
                        partition = tmp_parition;
                        break;
                    }
                }
                if(!is_success) break;
            }

            if(is_success)
            {
                inside_partition_dats_.push_back(partition);
            }

            map_visited = tmp_map_visited;
        }
    }


    //pickup
    vector<VPuzFE<VPuzInnerPartitionDat>> list;
    for(int id = 0; id < inside_partition_dats_.size(); id++)
    {
        VPuzFE<VPuzInnerPartitionDat> ve;
        ve.data_ = inside_partition_dats_[id];
        ve.weight = 0;
        for(int jd = 0; jd < outside_partition_dat_.groupA.size(); jd++)
        {
            for(pEmt anchor: ve.data_.anchors)
            {
                ve.weight -= (outside_partition_dat_.groupA[jd]->pos_ - anchor->pos_).norm();
            }
            for(pEmt fixed: ve.data_.fixs)
            {
                ve.weight += (fixed->pos_ - outside_partition_dat_.groupA[jd]->pos_).norm();
            }
        }
        for(int jd = 0; jd < outside_partition_dat_.groupB.size(); jd++)
        {
            for(pEmt anchor: ve.data_.anchors)
            {
                ve.weight += (outside_partition_dat_.groupB[jd]->pos_ - anchor->pos_).norm();
            }
            for(pEmt fixed: ve.data_.fixs)
            {
                ve.weight -= (fixed->pos_ - outside_partition_dat_.groupB[jd]->pos_).norm();
            }
        }
        //std::cout << (int)(part->maximum_access_value_ - puzzle_->get_accessibility(ve.data_.anchor->order_)) << std::endl;
        list.push_back(ve);
    }

    VPuzFilter<VPuzInnerPartitionDat> filter;
    filter.max_candidates_num_ = maximum_inner_partition_plan_;
    filter.insert(list);


    inside_partition_dats_.clear();
    for(VPuzFE<VPuzInnerPartitionDat> dat: filter.candidate_)
    {
        inside_partition_dats_.push_back(dat.data_);
    }

    return;
}

bool VoxelizedPartition::connect_group_A_anchor_voxels(VPuzRemainVolumePartitionDat &full_partition_dat)
{
    //last part of puzzle
    VoxelizedPart* part = puzzle_->parts_.back().get();

    unordered_map<int, int> map_voxel_group_id;
    unordered_map<int, int> map_prev_id;
    vector<int> group_present_id;

    map_voxel_group_id.clear(); map_prev_id.clear();

    auto cmp = [&](const VoxelBFSNode &a, const VoxelBFSNode &b){return a.weight > b.weight;};
    std::priority_queue<VoxelBFSNode, vector<VoxelBFSNode>, decltype(cmp) > queue(cmp);

    //Group A
    for(int id = 0; id < full_partition_dat.groupA.size(); id++)
    {
        group_present_id.push_back(id);
        VoxelElement *em = full_partition_dat.groupA[id];

        //build map
        map_voxel_group_id.insert(std::make_pair(em->order_, id));
        map_prev_id.insert(std::make_pair(em->order_, em->order_));
        insert_into_group_A(em);

        //push into queue
        VoxelBFSNode u;
        u.weight = puzzle_->get_accessibility(em->order_);
        u.em = em;
        queue.push(u);
    }

    //Group B
    for(int id = 0; id < full_partition_dat.groupB.size(); id++)
    {
        VoxelElement *em = full_partition_dat.groupB[id];
        map_voxel_group_id.insert(std::make_pair(em->order_, -1));
    }

    //connected component
    int num_non_merging_group = full_partition_dat.groupA.size();
    while(num_non_merging_group > 1 && !queue.empty())
    {
        VoxelBFSNode u = queue.top(); queue.pop();
        //std::cout << "u:\t" << u.em->pos_[0] << ", " << u.em->pos_[1] << ", " << u.em->pos_[2] << std::endl;
        int gid_u = group_present_id[map_voxel_group_id[u.em->order_]];

        for(int nrm = 0; nrm < 6; nrm++)
        {
            pEmt v = part->neighbor(1 << nrm, u.em);
            if(v != nullptr)
            {
                //std::cout <<"\t v: \t" << v->pos_[0] << ", " << v->pos_[1] << ", " << v->pos_[2] << std::endl;
                auto find_it = map_voxel_group_id.find(v->order_);
                if(find_it == map_voxel_group_id.end()) {

                    //push into queue
                    VoxelBFSNode nu;
                    nu.weight = puzzle_->get_accessibility(v->order_);
                    nu.em = v;
                    queue.push(nu);

                    //build map
                    map_voxel_group_id.insert(std::make_pair(v->order_, map_voxel_group_id[u.em->order_]));
                    map_prev_id.insert(std::make_pair(v->order_, u.em->order_));

                    continue;
                }

                if(find_it->second == -1)
                {
                    //visited group b's voxel, cannot continue;
                    continue;
                }
                else
                {
                    int gid_v = group_present_id[find_it->second];
                    if(gid_v != gid_u)
                    {
                        int it_id = u.em->order_;
                        while(it_id != map_prev_id[it_id])
                        {
                            insert_into_group_A(puzzle_->map_ip_[it_id]);
                            it_id = map_prev_id[it_id];
                        }
                        it_id = v->order_;
                        while(it_id != map_prev_id[it_id])
                        {
                            insert_into_group_A(puzzle_->map_ip_[it_id]);
                            it_id = map_prev_id[it_id];
                        };

                        for(int id = 0; id < group_present_id.size(); id++)
                        {
                            if(group_present_id[id] == gid_v)
                                group_present_id[id] = gid_u;
                        }
                        num_non_merging_group --;
                        if(num_non_merging_group <= 1)  break;
                    }
                }

            }
        }
    }

    if(num_non_merging_group != 1 || group_A_.size() > maximum_voxel_in_one_piece)
        return false;
    else
    {
        full_partition_dat.groupA = group_A_;
        return true;
    }
    return true;
}

bool VoxelizedPartition::maintain_disassemblability(int relation, VPuzRemainVolumePartitionDat &full_partition_dat, int &disassembled_direction)
{
    pPart  part = puzzle_->parts_.back().get();

    VPuzRemainVolumePartitionDat disassemblable_plan;
    std::unordered_map<int, int> visited;

    //Group A
    for(int id = 0; id < group_A_.size(); id++)
    {
        visited.insert(std::make_pair(group_A_[id]->order_, 1));
    }

    //Group B
    for(int id = 0; id < full_partition_dat.groupB.size(); id++)
    {
        VoxelElement *em = full_partition_dat.groupB[id];
        visited.insert(std::make_pair(em->order_, -1));
    }

    int dX[3] = {1, 0, 0};
    int dY[3] = {0, 1, 0};
    int dZ[3] = {0, 0, 1};

    auto attach_more_voxels = [&](int XYZ, int direct, VPuzRemainVolumePartitionDat &dat) -> bool
    {
        std::map<int, bool> new_group_A;
        for(pEmt voxel: full_partition_dat.groupA)
        {
            Vector3i pos = voxel->pos_;
            while(true)
            {
                pos += direct * Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                if(puzzle_->V2I(pos) == -1)
                {
                    break;
                }
                pEmt em = part->in_part(pos);
                if(em == nullptr)
                {
                    //break;
                    continue;
                }

                auto find_it = visited.find(em->order_);
                if(find_it != visited.end() && find_it->second == -1) return false;
                if(find_it != visited.end() && find_it->second == 1) continue;
                new_group_A[em->order_] = true;
            }
        }

        dat.groupA.clear();
        dat.groupB.clear();

        dat.relation = full_partition_dat.relation;
        dat.groupB = full_partition_dat.groupB;
        dat.groupA = full_partition_dat.groupA;

        for(auto it = new_group_A.begin(); it!= new_group_A.end(); it++)
        {
            dat.groupA.push_back(puzzle_->map_ip_[it->first]);
        }

        if(dat.groupA.size() > maximum_voxel_in_one_piece) return false;
        else return true;
    };

    int XYZ = part_disassembly_direction_ / 2;
    int sign = part_disassembly_direction_ % 2;
    if(attach_more_voxels(XYZ, sign == 0 ? 1 : -1, disassemblable_plan))
    {
        disassemblable_plan.relation = 1 << (2 * XYZ + sign);
        full_partition_dat = disassemblable_plan;
        for(pEmt voxel: full_partition_dat.groupA)
        {
            insert_into_group_A(voxel);
        }
        disassembled_direction = full_partition_dat.relation;
        return true;
    }
    else
    {
        return false;
    }
}

bool VoxelizedPartition::expand_group_A_to_minimum_pieces(VPuzRemainVolumePartitionDat &full_partition_dat, int &disassembled_direction)
{
    if(group_A_.size() < minimum_voxel_in_one_piece)
    {
        //define area
        pPart part = puzzle_->parts_.back().get();

        auto cmp = [&](const VoxelBFSNode &a, const VoxelBFSNode &b){return a.weight > b.weight;};

        std::priority_queue<VoxelBFSNode, vector<VoxelBFSNode>, decltype(cmp) > queue(cmp);

        std::unordered_map<int, int> visited;

        auto random_choose_from_queue = [&]() -> VoxelBFSNode
        {
            double percentage_choice_rate = 0.9;
            int rand_range = 1000;
            vector<VoxelBFSNode> discard_list;
            VoxelBFSNode u;
            while(!queue.empty())
            {
                u = queue.top();
                queue.pop();
                if(std::rand() % rand_range < (int)(percentage_choice_rate * rand_range))
                {
                    //accept
                    for(VoxelBFSNode node: discard_list) queue.push(node);
                    return u;
                }
                else
                {
                    discard_list.push_back(u);
                }
            }

            u = discard_list.back(); discard_list.pop_back();
            for(VoxelBFSNode node: discard_list) queue.push(node);
            return u;

        };

        auto add_whole_line_voxel = [&](pEmt voxel, Vector3i direction) -> bool
        {
            vector<VoxelBFSNode> add_list;
            Vector3i pos = voxel->pos_;
            do{
                auto find_it = visited.find(voxel->order_);

                if(find_it != visited.end() && find_it->second == -1)
                {
                    return false;
                }
                else if(find_it == visited.end())
                {
                    VoxelBFSNode nu;
                    nu.weight = puzzle_->get_accessibility(voxel->order_);
                    nu.em = voxel;
                    add_list.push_back(nu);
                }

                pos += direction;
                if(puzzle_->V2I(pos) == -1) break;
                voxel = part->in_part(pos);
                if(voxel == nullptr) break;

            }while(true);

            for(VoxelBFSNode node : add_list)
            {
                queue.push(node);
                insert_into_group_A(node.em);
                visited.insert(std::make_pair(node.em->order_, 1));
            }

            return true;
        };

        //program

        for(int id = 0; id < group_A_.size(); id++)
        {
            pEmt em = group_A_[id];
            VoxelBFSNode u;
            u.em = em;
            u.weight = puzzle_->get_accessibility(em->order_);
            queue.push(u);
            visited.insert(std::make_pair(em->order_, 1));
        }
        for(int id = 0; id < full_partition_dat.groupB.size(); id++)
        {
            VoxelElement *em = full_partition_dat.groupB[id];
            visited.insert(std::make_pair(em->order_, -1));
        }

        Vector3i direction;
        int dX[6] = {1, -1, 0, 0, 0, 0};
        int dY[6] = {0, 0, 1, -1, 0, 0};
        int dZ[6] = {0, 0, 0, 0, 1, -1};

        for(int XYZ = 0; XYZ < 6; XYZ++)
        {
            if(disassembled_direction & 1)
            {
                direction = Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                break;
            }
            disassembled_direction >>= 1;
        }

        while(!queue.empty() && group_A_.size() < minimum_voxel_in_one_piece)
        {
            VoxelBFSNode u = random_choose_from_queue();
            for(int nrm = 0; nrm < 6; nrm++) {
                pEmt v = part->neighbor(1 << nrm, u.em);
                if (v != nullptr) {
                    add_whole_line_voxel(v, direction);
                    if(group_A_.size() >= maximum_voxel_in_one_piece) break;
                }
            }
        }
    }

    if(group_A_.size() < minimum_voxel_in_one_piece)
        return false;
    else
        return  true;
}

bool VoxelizedPartition::compute_remaining_volume(vector<pEmt> &group_B) {

    group_B.clear();
    pPart part = puzzle_->parts_.back().get();
    for(pEmt voxel: part->elist_)
    {
        if(!is_in_group_A(voxel))
        {
            group_B.push_back(voxel);
        }
    }

    if(group_B.empty())
        return  true;
    else
    {
        return is_remaining_volume_connected(group_B);
    }
}

void VoxelizedPartition::merge_inside_outside_partition_dat(VPuzInnerPartitionDat &inner_dat, VPuzRemainVolumePartitionDat &full_partition_dat)
{
    VPuzRemainVolumePartitionDat outer = outside_partition_dat_;

    //insert into group B
    for(pEmt voxel : inner_dat.fixs)
    {
        bool is_duplicate = false;
        for(pEmt vB : outer.groupB)
        {
            if(vB == voxel)
            {
                is_duplicate = true;
                break;
            }
        }
        if(!is_duplicate) outer.groupB.push_back(voxel);
    }

    //insert group A
    for(pEmt voxel : inner_dat.anchors)
    {
        bool is_duplicate = false;
        for(pEmt vA : outer.groupA)
        {
            if(vA == voxel)
            {
                is_duplicate = true;
                break;
            }
        }
        if(!is_duplicate) outer.groupA.push_back(voxel);
    }

    full_partition_dat = outer;
}

bool VoxelizedPartition::is_remaining_volume_connected(const vector<pEmt> &vlist) {

    std::unordered_map<int, pEmt> map;
    std::unordered_map<int, bool> visited;
    for(int id = 0; id < vlist.size(); id++)
    {
        map.insert(std::make_pair(vlist[id]->order_, vlist[id]));
    }

    int dX[6] = {-1, 1, 0, 0,  0, 0};
    int dY[6] = { 0, 0, -1, 1, 0, 0};
    int dZ[6] = { 0, 0,  0, 0, -1, 1};

    std::queue<pEmt> queue;
    queue.push(vlist.front());
    visited.insert(std::make_pair(vlist.front()->order_, true));
    int num_visited = 0;
    while(!queue.empty())
    {
        pEmt u = queue.front();queue.pop();
        num_visited ++;
        for(int id = 0; id < 6; id++)
        {
            Vector3i pos(u->pos_[0] + dX[id], u->pos_[1] + dY[id], u->pos_[2] + dZ[id]);
            auto find_it = map.find(puzzle_->V2I(pos));
            if(find_it != map.end())
            {
                pEmt v = find_it->second;
                if(visited.find(v->order_) == visited.end())
                {
                    queue.push(v);
                    visited.insert(std::make_pair(v->order_, true));
                }
            }
        }
    }
    if(num_visited == vlist.size())
        return true;
    else
        return false;
}

bool VoxelizedPartition::insert_into_group_A(pEmt em) {
    if(!is_in_group_A(em))
    {
        group_A_.push_back(em);
        in_group_A_.insert(std::make_pair(em->order_, true));
        return true;
    }
    else
    {
        return false;
    }
}

bool VoxelizedPartition::is_in_group_A(pEmt em) {
    std::unordered_map<int, bool>::iterator find_it = in_group_A_.find(em->order_);
    if(find_it == in_group_A_.end())
    {
        return false;
    }
    else
    {
        return true;
    }
}