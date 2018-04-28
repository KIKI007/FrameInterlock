//
// Created by ziqwang on 28.04.18.
//

#include "FrameInterlocking.h"

FrameInterlocking::FrameInterlocking(std::shared_ptr<FrameInterface> frame_interface)
{

}

void FrameInterlocking::init_tree()
{

}

std::shared_ptr<FrameInterface> FrameInterlocking::output_interface()
{
    return frame_interface;
}
