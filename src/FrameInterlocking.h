//
// Created by *** on 28.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERLOCKING_H
#define FRAMEINTERLOCK_FRAMEINTERLOCKING_H

#include "FrameInterface.h"
#include "FrameInterlockingTree.h"
#include "voxel/VoxelizedPuzzle.h"

class FrameInterlocking
{
public:
    FrameInterlocking(std::shared_ptr<FrameInterface> frame_interface);

public:

    void init_tree();

    void select_key_pillar();

    void output_information(TreeNode *node);

public:

    std::shared_ptr<FrameInterface> generate_interlocking();

    std::shared_ptr<FrameInterface> output_present_frame();

public:

    std::shared_ptr<FrameInterface> frame_interface_;

    std::shared_ptr<FrameInterlockingTree> tree_;

};

class Timer
{
public:

    Timer(std::string name) {name_ = name;clear();}

public:

    void start(){timer_ = clock();}

    void end(){tot_time += (double)(clock() - timer_)/(CLOCKS_PER_SEC);}

    void clear(){tot_time = 0;}

    double present_time(){return (double)(clock() - timer_)/(CLOCKS_PER_SEC);}

    void print(){std::cout << name_ << ":\t\t" << tot_time << std::endl;}

public:
    std::string name_;
    clock_t timer_;
    double tot_time;
};

#endif //FRAMEINTERLOCK_FRAMEINTERLOCKING_H
