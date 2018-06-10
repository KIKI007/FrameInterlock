 //
// Created by *** on 16.03.18.
//
#include "VoxelizedPuzzle.h"
VoxelizedPuzzle::VoxelizedPuzzle(int nx, int ny, int nz, int joint_id, std::shared_ptr<ColorCoding> coder)
{
    Nx = nx;
    Ny = ny;
    Nz = nz;
    joint_id_ = joint_id;
    array_3i A(boost::extents[Nx][Ny][Nz]);
    for(int ix = 0; ix < Nx; ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                A[ix][iy][iz] = -1;
            }
        }
    }
    colorcoder_ = coder;
    init(A);
}

VoxelizedPuzzle::VoxelizedPuzzle(int nx, int ny, int nz, int joint_id, const array_3b &array, std::shared_ptr<ColorCoding> coder) {
    Nx = nx;
    Ny = ny;
    Nz = nz;
    joint_id_ = joint_id;
    array_3i A(boost::extents[Nx][Ny][Nz]);
    for(int ix = 0; ix < Nx; ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                if(array[ix][iy][iz] == true)
                {
                    A[ix][iy][iz] = 0;
                }
                else
                {
                    A[ix][iy][iz] = -1;
                }
            }
        }
    }
    colorcoder_ = coder;
    init(A);
}

VoxelizedPuzzle::VoxelizedPuzzle(int nx, int ny, int nz,int joint_id, const array_3i &array, std::shared_ptr<ColorCoding> coder)
{
     Nx = nx;
     Ny = ny;
     Nz = nz;
     joint_id_ = joint_id;
     colorcoder_ = coder;
     init(array);
}

void VoxelizedPuzzle::init(const array_3i &array)
{
    std::map<int, bool> map_num_part;
    for(int iz = 0; iz < Nz; iz++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int ix = 0; ix < Nx; ix++)
            {
                if(array[ix][iy][iz] != -1)
                {
                    shared_pEmt p = std::make_shared<VoxelElement>
                            (VoxelElement(Vector3i(ix, iy, iz),
                                         V2I(Vector3i(ix, iy, iz)),
                                         joint_id_));
                    p->gid_ = array[ix][iy][iz];
                    map_ip_.insert(std::make_pair(p->order_, p.get()));
                    voxel_.push_back(p);
                    map_num_part[p->gid_] = true;
                }
            }
        }
    }

    for(auto it = map_num_part.begin(); it != map_num_part.end(); it++)
    {
        int part_id = it->first;
        VoxelsList vlist;
        for(shared_pEmt voxel : voxel_)
        {
            if(voxel->gid_ == part_id)
            {
                vlist.push_back(voxel.get());
            }
        }

        Vector3i size(Nx, Ny, Nz);
        std::shared_ptr<VoxelizedPart> part = std::make_shared<VoxelizedPart>(
                VoxelizedPart(vlist,
                              &map_ip_,
                              part_id,
                              joint_id_,
                              size));

        part->compute_access_map(&map_access_);
        parts_.push_back(part);
    }

    return;
}

int VoxelizedPuzzle::get_part_id(Vector3i pos)
{
     unordered_map<int, pEmt>::iterator find_it = map_ip_.find(V2I(pos));
     if(find_it == map_ip_.end())
         return -1;
     else
         return find_it->second->gid_;
}

std::shared_ptr<VoxelizedInterface> VoxelizedPuzzle::output_assembly_interface(bool accessibility = true, bool vertex_mark = true)
{

    std::shared_ptr<VoxelizedInterface> interface = std::make_shared<VoxelizedInterface>(VoxelizedInterface(Nx, Ny, Nz, colorcoder_));

    //set grid
    for(shared_pEmt voxel : voxel_)
    {
        interface->set_grid_part_index(voxel->pos_, voxel->gid_);
    }

    //set accessibility
    if(accessibility)
    {
        interface->use_extra_value_color(true);
        for(shared_pPart part: parts_)
        {
            for(pEmt voxel : part->elist_)
            {
                interface->set_grid_extra_value(voxel->pos_, map_access_[voxel->order_]);
            }
        }
    }

    //set mark
    //for debuging
    if(vertex_mark)
    {
        interface->use_extra_mark_color(true);
        for(auto it = map_mark.begin(); it != map_mark.end(); it++)
        {
            interface->set_grid_extra_mark(map_ip_[it->first]->pos_, it->second);
        }
    }

    return interface;
}

VoxelizedPuzzle::VoxelizedPuzzle(const VoxelizedPuzzle &A) {
    Nx = A.Nx;
    Ny = A.Ny;
    Nz = A.Nz;

    map_access_ = A.map_access_;
    map_mark = A.map_mark;

    voxel_.clear();
    map_ip_.clear();
    joint_id_ = A.joint_id_;
    for (int id = 0; id < A.voxel_.size(); id++) {
        shared_pEmt p = std::make_shared<VoxelElement>(VoxelElement(*A.voxel_[id]));
        voxel_.push_back(p);
        map_ip_.insert(std::make_pair(p->order_, p.get()));
    }

    parts_.clear();
    for (int id = 0; id < A.parts_.size(); id++)
    {
        VoxelsList part_voxel_list;
        for(pEmt em : A.parts_[id]->elist_)
        {
            pEmt voxel = map_ip_[em->order_];
            part_voxel_list.push_back(voxel);
        }

        shared_pPart part = std::make_shared<VoxelizedPart>(
                VoxelizedPart(part_voxel_list,
                              &map_ip_,
                              A.parts_[id]->part_id_,
                              A.parts_[id]->joint_id_,
                              A.parts_[id]->size_));
        part->map_access_ = &map_access_;
        parts_.push_back(part);
    }

    colorcoder_ = A.colorcoder_;
}


VoxelizedPuzzle::VoxelizedPuzzle()
{
     Nx = Ny = Nz = 0;
}


bool VoxelizedPuzzle::is_same(VoxelizedPuzzle *A) {
    for(int id = 0; id < Nx * Ny * Nz; id++)
    {
        auto find_it = map_ip_.find(id);
        if(find_it == map_ip_.end()) continue;
        if(map_ip_[id]->gid_ != A->map_ip_[id]->gid_)
            return false;
    }
    return true;
}



 void VoxelizedPuzzle::partition_part(const vector<pEmt> &new_part_voxels, int new_part_id)
 {

     vector<pEmt> list;
     for(pEmt voxel : new_part_voxels)
     {
         list.push_back(map_ip_[voxel->order_]);
         map_ip_[voxel->order_]->gid_ = new_part_id;
     }

     parts_.back()->remove_voxels();

     shared_pPart part = std::make_shared<VoxelizedPart>(VoxelizedPart(list,
                                                                       &map_ip_,
                                                                       new_part_id,
                                                                       joint_id_,
                                                                       Vector3i(Nx, Ny, Nz)));
     part->compute_access_map(&map_access_);
     parts_.insert(parts_.end() - 1, part);
     return;
 }

double VoxelizedPuzzle::get_sum_accessbility(vector<pEmt> voxels)
{
    double access = 0;
    for(pEmt em: voxels)
    {
        access += map_access_[em->order_];
    }
    return access;
}

//void VoxelizedPuzzle::build_part_connection(VoxelizedPart *vpart)
////build connection
//{
//    int num_part = parts_.size();
//    int pid = vpart->part_id_;
//
//    //for three direction
//    int dX[3] = {1, 0, 0};
//    int dY[3] = {0, 1, 0};
//    int dZ[3] = {0, 0, 1};
//
//    for(int XYZ = 0; XYZ < 3; XYZ++)
//    {
//        MatrixXi Graph = MatrixXi::Zero(num_part, num_part);
//        vpart->neighbor_[XYZ].clear();
//        VPartNeighbor &neighbor = vpart->neighbor_[XYZ];
//        Vector3i npos;
//        for(pEmt voxel : vpart->elist_.data_)
//        {
//            //X/Y/Z + positive
//            //X/Y/Z_npos >= X/Y/Z_pos
//            npos = voxel->pos_ + Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
//            int npid = get_part_id(npos);
//            if(npid != -1 && npid != pid)
//            {
//                if (Graph(npid, pid) == 0)
//                {
//                    neighbor.in_.push_back(parts_[npid].get());
//                    std::shared_ptr<OrderedVElemList> pOVEL = std::make_shared<OrderedVElemList>(OrderedVElemList());
//                    neighbor.in_blocking_.push_back(pOVEL);
//                    Graph(npid, pid) = neighbor.in_blocking_.size();
//                }
//                OrderedVElemList* pOVEL = neighbor.in_blocking_[Graph(npid, pid) - 1].get();
//                if(pOVEL) pOVEL->force_push(voxel);
//            }
//
//            //X/Y/Z - negative
//            //X/Y/Z_npos <= X/Y/Z_pos
//            npos = voxel->pos_ - Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
//            npid = get_part_id(npos);
//            if(npid != -1 && npid != pid)
//            {
//                if (Graph(pid, npid) == 0)
//                {
//                    neighbor.out_.push_back(parts_[npid].get());
//                    std::shared_ptr<OrderedVElemList> pOVEL = std::make_shared<OrderedVElemList>(OrderedVElemList());
//                    neighbor.out_blocking_.push_back(pOVEL);
//                    Graph(pid, npid) = neighbor.out_blocking_.size();
//                }
//                OrderedVElemList* pOVEL = neighbor.out_blocking_[Graph(pid, npid) - 1].get();
//                if(pOVEL) pOVEL->force_push(voxel);
//            }
//
//        }
//    }
//    return;
//}
