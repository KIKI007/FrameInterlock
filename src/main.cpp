#include "Utility.h"

igl::viewer::Viewer viewer;

string shared_ptr;

std::shared_ptr<ColorCoding> colorcoder;

int main() {
    init();
    nanogui::init();
    viewer.callback_init = [&](igl::viewer::Viewer &viewer)
    {
        viewer.ngui->addWindow(Eigen::Vector2i(220, 10), "Frame Render");
        viewer.ngui->addGroup("Parameter");


        viewer.ngui->addGroup("Action");
        viewer.ngui->addButton("draw", [](){test();});
        viewer.screen->performLayout();
        return true;
    };
    viewer.launch(true, false);
    return 0;
}