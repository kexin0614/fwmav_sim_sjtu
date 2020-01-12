#include "flappy_dart/SimPanel.h"

void SimPanel::render()
{
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    ImGui::SetNextWindowSize(ImVec2(320, 320));
    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin(
            "SJTU FWMAV Simulation",
            nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
                | ImGuiWindowFlags_HorizontalScrollbar))
    {
        // Early out if the window is collapsed, as an optimization.
        ImGui::End();
        return;
    }

    // Menu
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Menu"))
        {
        if (ImGui::MenuItem("Exit"))
            mViewer->setDone(true);
        ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help"))
        {
        if (ImGui::MenuItem("About DART"))
            mViewer->showAbout();
        ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    ImGui::Text("Multi-body verification\n of SJTU FWMAV.");
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
    {
        int e = mViewer->isSimulating() ? 0 : 1;
        if (mViewer->isAllowingSimulation())
        {
        if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
            mViewer->simulate(true);
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
            mViewer->simulate(false);
        }
        ImGui::SameLine();
        if(ImGui::Button("Reset")){
            mSimEnv -> reset();
        }

        ImGui::Text("Time: %.3f", mWorld->getTime());
    }

    if (ImGui::CollapsingHeader(
            "World Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Gravity
        ImGui::Checkbox("Gravity On/Off", &mGuiGravity);
        setGravity(mGuiGravity);

        // ImGui::Spacing();

        if(ImGui::SliderFloat("Throttle",&mThrottleSet, 0, 1))
            setThrottle();
        
        if(ImGui::SliderFloat("Sim Speed", &mRealTimeFactorSet, 0.01, 1, "%.2fx")){
            mSimEnv -> setCustomRealTimeFactor(mRealTimeFactorSet);
        }

        // Headlights
            // mGuiHeadlights = mViewer->checkHeadlights();
            // ImGui::Checkbox("Headlights On/Off", &mGuiHeadlights);
            // mViewer->switchHeadlights(mGuiHeadlights);
        
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen))
    {
        osg::Vec3d eye;
        osg::Vec3d center;
        osg::Vec3d up;
        mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

        ImGui::Text("Eye   : (%.2f, %.2f, %.2f)", eye.x(), eye.y(), eye.z());
        ImGui::Text(
            "Center: (%.2f, %.2f, %.2f)", center.x(), center.y(), center.z());
        ImGui::Text("Up    : (%.2f, %.2f, %.2f)", up.x(), up.y(), up.z());
    }

    // if (ImGui::CollapsingHeader("Help"))
    // {
    //     ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
    //     ImGui::Text("User Guide:\n");
    //     ImGui::Text("%s", mViewer->getInstructions().c_str());
    //     ImGui::PopTextWrapPos();
    // }

    ImGui::End();
}

void SimPanel::setGravity(bool gravity)
{
    if (mGravity == gravity)
        return;

    mGravity = gravity;

    if (mGravity)
        mWorld->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
    else
        mWorld->setGravity(Eigen::Vector3d::Zero());
}

void SimPanel::setThrottle()
{
    mSimEnv -> setFWMAVThrottle(mThrottleSet);
}

