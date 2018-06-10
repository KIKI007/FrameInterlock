//
// Created by *** on 16.03.18.
//

#include "VoxelizedPart.h"

VoxelizedPart::VoxelizedPart(const VoxelsList &part_voxels_list,
                             unordered_map<int, pEmt> *map_ip,
                             int part_id,
                             int joint_id,
                             Vector3i size)
{
    part_id_= part_id;
    joint_id_ = joint_id;
    elist_ = part_voxels_list;
    map_ip_ = map_ip;
    size_ = size;
    map_access_ = nullptr;
}

bool VoxelizedPart::in_part(pEmt p)
{
    auto iter = (*map_ip_).find(V2I(p->pos_));

    if(iter == (*map_ip_).end()) return false;
    if(iter->second->gid_ != part_id_) return false;
    if(iter->second->joint_id_ != joint_id_) return false;
    return true;
}

double VoxelizedPart::get_accessibilty(pEmt p)
{
    if(!in_part(p))
    {
        return -1;
    }
    return (*map_access_)[V2I(p->pos_)];

}

void VoxelizedPart::compute_access_map()
{
    if(map_access_ == nullptr) return;

    vector<double> access_values;
    for(pEmt p: elist_)
    {
        p->tmp_ = compute_num_neighbor(p);
    }

    access_values.resize(elist_.size(), 0);
    for(int id = 1; id <= 3; id++)
    {
        for(int jd = 0; jd < elist_.size(); jd++)
        {
            pEmt p = elist_[jd];
            access_values[jd] = p->tmp_;

            //neighbor
            for(int kd = 0; kd < 6; kd++)
            {
                pEmt q = neighbor((1 << kd), p);
                if(q) access_values[jd] += std::powl(0.1, id) * q->tmp_;
            }
        }

        for(int jd = 0; jd < elist_.size(); jd++)
        {
            pEmt p = elist_[jd];
            p->tmp_ = access_values[jd];
        }
    }

    double values;
    maximum_access_value_ = -1;
    sum_access_value_ = 0;
    for(pEmt p : elist_)
    {
        values += p->tmp_;
        sum_access_value_ += p->tmp_;
        if(p->tmp_ > maximum_access_value_) maximum_access_value_ = p->tmp_;
        auto find_it = map_access_->find(p->order_);
        if(find_it == map_access_->end())
        {
            (*map_access_).insert(std::make_pair(V2I(p->pos_), p->tmp_));
        }
        else
        {
            (*map_access_)[p->order_] = p->tmp_;
        }
        p->tmp_ = 0;
    }
}

int VoxelizedPart::compute_num_neighbor(pEmt p) {

    int num_neighbor = 0;
    for(int id = 0; id < 6; id++)
    {
        pEmt q = neighbor((1<<id), p);
        if(q) num_neighbor++;
    }
    return num_neighbor;
}

void VoxelizedPart::compute_access_map(unordered_map<int, double> *map_access)
{
    if(map_access != nullptr)
    {
        map_access_ = map_access;
        compute_access_map();
    }
}

VoxelizedPart::VoxelizedPart(const VoxelizedPart &a)
{
    elist_ = a.elist_;
    size_ = a.size_;
    part_id_ = a.part_id_;
    joint_id_ = a.joint_id_;
    map_ip_ = a.map_ip_;
    map_access_ = a.map_access_;
    maximum_access_value_ = a.maximum_access_value_;
    sum_access_value_ = a.sum_access_value_;
}

pEmt VoxelizedPart::neighbor(int nrm, pEmt p) {
    Vector3i dX(0, 0, 0);
    switch(nrm)
    {
        case 1:
            dX = Vector3i(1, 0, 0);
            break;
        case 2:
            dX = Vector3i(-1, 0, 0);
            break;
        case 4:
            dX = Vector3i(0, 1, 0);
            break;
        case 8:
            dX = Vector3i(0, -1, 0);
            break;
        case 16:
            dX = Vector3i(0, 0, 1);
            break;
        case 32:
            dX = Vector3i(0, 0, -1);
            break;
        default:
            dX = Vector3i(0, 0, 0);
            break;
    }

    if(dX == Vector3i(0, 0, 0)) return nullptr;

    Vector3i nPos = p->pos_ + dX;
    if(nPos[0] < 0 || nPos[1] < 0 || nPos[2] < 0)
        return nullptr;

    if(nPos[0] >= size_[0] || nPos[1] >= size_[1] || nPos[2] >= size_[2])
        return nullptr;

    auto find_it = (*map_ip_).find(V2I(nPos));
    if(find_it != (*map_ip_).end() &&
       find_it->second->gid_ == part_id_)
    {
        return find_it->second;
    }
    else
    {
        return nullptr;
    }

}

pEmt VoxelizedPart::in_part(Vector3i pos) {
    if(pos[0] < 0 || pos[1] < 0 || pos[2] < 0) return nullptr;
    if(pos[0] >= size_[0] || pos[1] >= size_[1] || pos[2] >= size_[2]) return nullptr;
    auto iter = (*map_ip_).find(V2I(pos));
    if(iter == (*map_ip_).end() || iter->second->gid_ != part_id_)
        return nullptr;
    else
        return iter->second;
}

void VoxelizedPart::remove_voxels()
{
    VoxelsList nelist;
    for(pEmt voxel: elist_)
    {
        if(voxel->gid_ == part_id_)
        {
            nelist.push_back(voxel);
        }
    }
    elist_ = nelist;
}