#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include <dart/common/Uri.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <ros/ros.h>

#include "flappy_dart/SimEnv.h"
#include "flappy_dart/SimPanel.h"
#include "flappy_dart/Flappy.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "DartSimNode");
    // Create a world
    dart::simulation::WorldPtr world(new dart::simulation::World);

    // Add a target object to the world
    dart::gui::osg::InteractiveFramePtr target(
        new dart::gui::osg::InteractiveFrame(dart::dynamics::Frame::World()));
    // world->addSimpleFrame(target);

    // Wrap a WorldNode around it
    osg::ref_ptr<SimEnv> node = new SimEnv(world,"/home/kexin/Work/fwmav_sim_custom_ws/config/sim_config.json");

    // Create a Viewer and set it up with the WorldNode
    osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
        = new dart::gui::osg::ImGuiViewer();
    viewer->addWorldNode(node);

    // Add control widget for atlas
    viewer->getImGuiHandler()->addWidget(
    std::make_shared<SimPanel>(viewer, world, node));

    // Active the drag-and-drop feature for the target
    viewer->enableDragAndDrop(target.get());

    // Pass in the custom event handler
    // viewer->addEventHandler(new CustomEventHandler);

    // Set up the window to be 640x480
    viewer->setUpViewInWindow(0, 0, 640, 480);

    // Adjust the viewpoint of the Viewer
    viewer->getCameraManipulator()->setHomePosition(
        ::osg::Vec3(2.57f, 3.14f, 1.64f),
        ::osg::Vec3(0.00f, 0.00f, 0.00f),
        ::osg::Vec3(-0.24f, -0.25f, 0.94f));
    // We need to re-dirty the CameraManipulator by passing it into the viewer
    // again, so that the viewer knows to update its HomePosition setting
    viewer->setCameraManipulator(viewer->getCameraManipulator());

    // Begin running the application loop
    viewer->run();
}