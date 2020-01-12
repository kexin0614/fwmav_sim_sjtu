#ifndef SIM_PANEL_H_
#define SIM_PANEL_H_

#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include "flappy_dart/SimEnv.h"

class SimPanel : public dart::gui::osg::ImGuiWidget
{
public:
  /// Constructor
  SimPanel(
      dart::gui::osg::ImGuiViewer* viewer,
      dart::simulation::WorldPtr world,
      osg::ref_ptr<SimEnv> simenv)
    : mViewer(viewer),
      mWorld(std::move(world)),
      mSimEnv(simenv),
      mGuiGravity(true),
      mGravity(true),
      mGuiHeadlights(true),
      mThrottleSet(0.0),
      mRealTimeFactorSet(0.5)
  {
    // Do nothing
  }

  // Documentation inherited
  void render() override;

protected:
  void setGravity(bool gravity);
  void setThrottle();

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  dart::simulation::WorldPtr mWorld;
  osg::ref_ptr<SimEnv> mSimEnv;
  bool mGuiGravity;
  bool mGravity;
  bool mGuiHeadlights;

  float mThrottleSet;
  float mRealTimeFactorSet;
};

#endif