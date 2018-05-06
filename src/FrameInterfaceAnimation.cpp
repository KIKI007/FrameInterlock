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
