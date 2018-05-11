//
// Created by ziqwang on 06.05.18.
//

#include "FrameInterfaceAnimation.h"


void FrameInterfaceAnimation::draw_animation(MatrixXd &V, MatrixXi &F, MatrixXd &C, double ratio)
{
    int num_pillar = pillars_.size();
    int animation_step = (int)(ratio * num_pillar);

    vector<FrameInterfaceRenderUnit> render_unit;

    std::map<int, bool> would_render;
    int would_move = disassembling_sequences_[animation_step];
    for(int id = animation_step; id < num_pillar; id++)
    {
        would_render.insert(std::make_pair(disassembling_sequences_[id], true));
    }

    double move_ratio = ratio * num_pillar - animation_step;
    for(FrameInterfaceRenderUnit unit : units_)
    {
        if(would_render.find(unit.pillar_index) != would_render.end())
        {
            if(unit.pillar_index == would_move)
            {
                for(int id = 0; id < unit.V.rows(); id++)
                {
                    unit.V.row(id) += directions_[animation_step] * max_length * move_ratio;
                }
            }
            render_unit.push_back(unit);
        }
    }

    //render
    int nV = 0;
    int nF = 0;
    for(FrameInterfaceRenderUnit &unit : render_unit)
    {
        if(unit.visible)
        {
            unit.dF = RowVector3i(nV, nV, nV);
            nV += unit.V.rows();
            nF += unit.F.rows();
        }
    }

    V = MatrixXd(nV, 3);
    C = MatrixXd(nF, 3);
    F = MatrixXi(nF, 3);

    int iV = 0, iF = 0;
    for(FrameInterfaceRenderUnit unit : render_unit)
    {
        for (int id = 0; id < unit.V.rows(); id++) {
            V.row(iV++) = unit.V.row(id);
        }

        for (int id = 0; id < unit.F.rows(); id++) {
            C.row(iF) = unit.C.row(id);
            F.row(iF++) = unit.F.row(id) + unit.dF;
        }
    }
    return;
}

bool FrameInterfaceAnimation::compute_animation_sequences()
{
    //following the order of Y
    disassembling_sequences_.clear();
    directions_.clear();

    std::shared_ptr<DirectedGraph>  graph[3];
    std::shared_ptr<Assembly> assembly = output_assembly();
    InterlockSDChecking checker;
    Vector3d vec[3] = {
            Vector3d(1, 0, 0),
            Vector3d(0, 1, 0),
            Vector3d(0, 0, 1)};
    for(int XYZ = 0; XYZ < 3; XYZ++)
    {
        graph[XYZ] =  checker.init_sd_graph(vec[XYZ], *assembly);
        graph[XYZ]->nodeLists_.pop_back();
    }

    auto is_graph_empty = [&]() ->bool
    {
        for(int XYZ = 0; XYZ < 3; XYZ++)
            if(graph[XYZ]->nodeLists_.empty())
                return true;
        return false;
    };

    std::map<int, bool> visited;
    while(!is_graph_empty())
    {
        vector<AnimationMovablePart> movable_parts;
        for(int XYZ = 0; XYZ < 3; XYZ++)
        {
            vector<DirectGraphNodeValence> valences;
            graph[XYZ]->compute_node_valence(valences);
            compute_movable_parts(valences, visited, movable_parts, XYZ);
        }

        if(movable_parts.empty())
        {
            std::cout << "DeadLock!!!" << std::endl;
            disassembling_sequences_.clear();
            directions_.clear();
            return false;
        }

        std::sort(movable_parts.begin(), movable_parts.end(), [&](const AnimationMovablePart &iA, const AnimationMovablePart &iB)
        {
            Vector3d pA = frame_mesh_->points_[pillars_[iA.part_id]->cube_id[0]] +
                          frame_mesh_->points_[pillars_[iA.part_id]->cube_id[1]];
            Vector3d pB = frame_mesh_->points_[pillars_[iB.part_id]->cube_id[0]] +
                          frame_mesh_->points_[pillars_[iB.part_id]->cube_id[1]];
            return pA[1] > pB[1];
        });

        int index = movable_parts.front().part_id;
        disassembling_sequences_.push_back(index);
        directions_.push_back(movable_parts.front().move_direction);
        visited[index] = true;
        for(int XYZ = 0; XYZ < 3; XYZ++)
            graph[XYZ]->remove_node(index);
    }
    return true;
}

void FrameInterfaceAnimation::compute_movable_parts(const vector<DirectGraphNodeValence> &valences,
                                                    const std::map<int, bool> &visited,
                                                    vector<AnimationMovablePart> &movable_parts,
                                                    int XYZ) {
    int dX[3] = {1, 0 ,0};
    int dY[3] = {0, 1, 0};
    int dZ[3] = {0, 0, 1};
    for(int id = 0; id < valences.size(); id++)
    {
        int index = valences[id].index;
        if(visited.find(index) != visited.end()) continue;
        if(valences[id].valence_in == 0)
        {
            AnimationMovablePart ampart;
            ampart.part_id = index;
            ampart.move_direction = Vector3d(dX[XYZ], dY[XYZ], dZ[XYZ]);
            if(is_collision_free(index, 2 * XYZ, visited))
                movable_parts.push_back(ampart);
        }
        if(valences[id].valence_out == 0)
        {
            AnimationMovablePart ampart;
            ampart.part_id = index;
            ampart.move_direction = -Vector3d(dX[XYZ], dY[XYZ], dZ[XYZ]);
            if(is_collision_free(index, 2 * XYZ + 1, visited))
                movable_parts.push_back(ampart);
        }
    }
    return;
}

bool FrameInterfaceAnimation::is_collision_free(int index, int nrm, const std::map<int, bool> &visited) {
    FramePillar *u = pillars_[index].get();
    for(int id = 0; id < pillars_.size(); id++)
    {
        if(visited.find(pillars_[id]->index) == visited.end())
        {
            FramePillar *v = pillars_[id].get();
            for(int ix = 0; ix < 2; ix++)
            {
                for(int iy = 0; iy < 2; iy++)
                {
                    if(u->cube_id[ix] == v->cube_id[iy] && v->pos_in_cube_face[iy] == nrm)
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}
