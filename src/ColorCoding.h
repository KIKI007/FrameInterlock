//
// Created by *** on 18.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_COLORCODING_H
#define UNDERSTAND_INTERLOCK_COLORCODING_H
#include <Eigen/Dense>
#include <vector>
#include "igl/viewer/Viewer.h"

using nanogui::Color;
using Eigen::MatrixXd;
using Eigen::Vector3d;

typedef std::vector<std::shared_ptr<Color>> veccolor;

class ColorCoding
{
public:
    ColorCoding(const veccolor &tab)
    {
        frame_color_ = Color(255, 255, 0, 255);
        part_num_ = tab.size();

        for(int id = 0; id < part_num_; id++) {
            colorCode.push_back(tab[id]);
        }
    }

public:
    void request(int num)
    {
        for(int id = part_num_; id < num; id++)
        {
            Eigen::RowVector3d c, oc;
            while(true)
            {
                c = Eigen::RowVector3d(rand()%255/255.0,
                                       rand()%255/255.0,
                                       rand()%255/255.0);
                int jd;
                bool is_duplicate = false;
                for(jd = std::max((int)colorCode.size() - 10, 0); jd < colorCode.size(); jd++)
                {
                    oc = Eigen::RowVector3d((colorCode[jd])->r(), (colorCode[jd])->g(), (colorCode[jd])->b());
                    if((c - oc).norm() < 0.4)
                    {
                        is_duplicate = true;
                        break;
                    }
                }
                if(!is_duplicate)
                {
                    break;
                }
            }
            std::shared_ptr<Color> pC = std::make_shared<Color>(Color(c[0] * 255, c[1] * 255, c[2] * 255, 255));
            colorCode.push_back(pC);
        }
        part_num_ = num;
    }

    Eigen::RowVector3d get(int id)
    {
        if(id < 0 || id >= part_num_) return Eigen::RowVector3d(0, 0, 0);
        else return Eigen::RowVector3d(colorCode[id]->r(), colorCode[id]->g(), colorCode[id]->b());
    }

public:

    void jet( Eigen::VectorXd &value, double min, double max, Eigen::MatrixXd &C)
    {
        igl::jet(value, min, max, C);
    }

public:

    int part_num_;

    veccolor colorCode;

    Color frame_color_;

};

#endif //UNDERSTAND_INTERLOCK_COLORCODING_H
