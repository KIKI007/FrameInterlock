//
// Created by ziqwang on 06.05.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERFACEANIMATION_H
#define FRAMEINTERLOCK_FRAMEINTERFACEANIMATION_H

#include "FrameInterface.h"
class FrameInterfaceAnimation : public FrameInterface{
public:

    FrameInterfaceAnimation(const FrameInterface &frame_interface): FrameInterface(frame_interface)
    {
        draw_frame_mesh(units_);
        draw_joints(units_, false);

        max_length = 1;

        //render
        for(FrameInterfaceRenderUnit &unit : units_) {
            if (unit.visible) {
                for (int id = 0; id < unit.V.rows(); id++) {
                    unit.V.row(id) = unit.V.row(id) + unit.dV;
                }
            }
        }
        return;
    }

public:

    void set_disassembling_sequences(const vector<int> &disassembling)
    {
        disassembling_sequences_ = disassembling;
    };

    void set_disassembling_direction(const vecVector3d &directions)
    {
        directions_ = directions;
    }

    //ratio from 0 ~ 1
    void draw_animation(MatrixXd &V, MatrixXi &F, MatrixXd &C, double ratio);

public:

    vector<int> disassembling_sequences_;

    vecVector3d directions_;

    int max_length;

    vector<FrameInterfaceRenderUnit> units_;
};


#endif //FRAMEINTERLOCK_FRAMEINTERFACEANIMATION_H
