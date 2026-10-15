/***********************************************************************************************************************
 * Copyright (c) 2025 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

 /***********************************************************************************************************************
 * File:        sim-system.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the chrono system
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_SYSTEM_HPP_
#define SIM_SYSTEM_HPP_

// System includes
#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

// ACSL chrono sim includes
#include "sim-helpers.hpp"
#include "fkYAML/node.hpp"

// Chrono includes
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

// Chrono FSI SPH includes
#include <cassert>
#include <cstdlib>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/functions/ChFunctionBase.h" 

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/ChGuiComponentVSG.h"
#include <vsgImGui/imgui.h>

namespace _acsl_
{

namespace _system_
{

// Structure for storing the system configurations
struct physicsonfig {
    bool PSOR;
    bool gravity;
    int MaxIterations;
    bool EnableWarmStart;
    bool BULLET;
    bool MULTICORE;
    double DefaultSuggestedEnvelope;
    double DefaultSuggestedMargin;
    double ContactBreakingThreshold;
    double StepSize;
    bool ThrottleRealTime;
    bool ActivateFSI;               // Fluid stuffs starts here
};

// Structure for storing the visualization configurations
struct visionconfig {
    uint width;
    uint height;
    std::string title;
    bool enable_vis;
    bool enable_static_cam;
    bool render_ned_frame;
    bool render_body_frame;
    bool render_shadows;
    bool render_collision_zones;
    double mv_cam_chase_ht;
    double mv_cam_chase_dt;
    bool render_all_COG_frames;
    bool render_prop_frames;
    bool render_trajectory;
    bool render_biplane_frame;
    bool render_chassis_drag_frame;
    bool render_wing_aero_frames;
    bool enable_vulkan;
    bool enable_irrlicht;
    double target_frame_rate;
    bool show_fluid;            // Fluid options
    bool show_bce;
};

// ------------------------------------------------------------------------
// Render Backend
// Lets the simulator switch between VULKAN and IRRLICHT
// ------------------------------------------------------------------------
enum class RenderBackend { VULKAN, IRRLICHT };

// =====================================================================================================================
// simsystem
//
// Purpose:
//   Encapsulates and manages the core ACSL Physics Simulator systems,
//   including the Chrono physics engine and the Irrlicht visualization engine.
//   This class is responsible for reading configuration files, setting up
//   the simulation environment, and providing access to the underlying
//   systems and configuration data.
//
// Usage:
//   - Create an instance of simsystem at program start.
//   - It will automatically load physics and vision settings from YAML configs.
//   - Call SetupPhysicsSystem() and SetupVisualizationSystem() to build systems.
//   - Use getters to access the Chrono and visualization systems.
// =====================================================================================================================
class simsystem
{
public:

    // ------------------------------------------------------------------------
    // Constructor:
    //   - Reads in both physics (.yaml) and visualization (.yaml) config files
    //     when the simulator system object is created.
    // ------------------------------------------------------------------------
    simsystem()
    {
        ReadPhysicsConfigFile();
        ReadVisionConfigFile();
    } 

    // ------------------------------------------------------------------------
    // Sets up the Chrono physics system based on loaded config parameters.
    // Example: gravity, solver type, solver iterations, collision settings.
    // ------------------------------------------------------------------------
    void SetupPhysicsSystem();

    // ------------------------------------------------------------------------
    // Sets up the Chrono physics system based on loaded config parameters.
    // Example: gravity, solver type, solver iterations, collision settings.
    // ------------------------------------------------------------------------
    void SetupFSISystem();

    // ------------------------------------------------------------------------
    // Sets up the Irrlicht visualization system for rendering the simulation
    // based on loaded vision configuration (window size, camera settings, etc.).
    // ------------------------------------------------------------------------
    void SetupVisualizationSystem();

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the underlying Chrono physics system
    // so that external modules can add bodies, links, etc.
    // ------------------------------------------------------------------------
    chrono::ChSystemNSC& GetPhysicsSystem() { return m_physics; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the underlying Chrono SPH system
    // ------------------------------------------------------------------------
    chrono::fsi::ChFluidSystemSPH& GetSPHSystem() { return m_SPH; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the underlying Chrono FSI system
    // ------------------------------------------------------------------------
    chrono::fsi::ChFsiSystemSPH& GetFSISystem() { return m_FSI; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the underlying Irrlicht visualization
    // system for customizing the viewing environment.
    // ------------------------------------------------------------------------
    chrono::irrlicht::ChVisualSystemIrrlicht& GetVisionSystemIrr() { return m_irrlicht; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the underlying Vulkan visualization
    // system for customizing the viewing environment.
    // ------------------------------------------------------------------------
    chrono::vsg3d::ChVisualSystemVSG& GetVisionSystemVsg() { return m_vsg; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the underlying FSI specific Vulkan 
    // visualization system for customizing the viewing environment.
    // ------------------------------------------------------------------------
    std::shared_ptr<chrono::fsi::ChFsiVisualizationVSG> GetVisionSystemFsi() const { return m_visFSI; }

    // ------------------------------------------------------------------------
    // Accessor: Returns the active visualization backend
    // ------------------------------------------------------------------------
    RenderBackend GetActiveBackend() { return m_active_backend; }

    // ------------------------------------------------------------------------
    // Accessor: Returns the loaded physics configuration struct.
    // ------------------------------------------------------------------------
    physicsonfig GetPhyConfig() { return phyconfig; }

    // ------------------------------------------------------------------------
    // Accessor: Returns the loaded vision configuration struct.
    // ------------------------------------------------------------------------
    visionconfig GetVisConfig() { return visconfig; }

    // ------------------------------------------------------------------------
    // Accessor: Returns the camera of the system
    // ------------------------------------------------------------------------
    std::shared_ptr<chrono::utils::ChChaseCamera> GetCamera() { return m_camera; }

private:

    // ------------------------------------------------------------------------
    // Constants: Hardcoded file paths to the YAML configuration files.
    // These define initial physics and visualization settings.
    // ------------------------------------------------------------------------
    const std::string phy_config_filename = "../config/phy-config.yaml";
    const std::string vis_config_filename = "../config/vis-config.yaml";

    // ------------------------------------------------------------------------
    // The main Chrono physics simulation system (non-smooth contact variant).
    // ------------------------------------------------------------------------
    chrono::ChSystemNSC m_physics;

    // ------------------------------------------------------------------------
    // The Chrono fluid simulation system (FSI-SPH based).
    // ------------------------------------------------------------------------
    chrono::fsi::ChFluidSystemSPH m_SPH;
    chrono::fsi::ChFsiSystemSPH   m_FSI{m_physics, m_SPH};

    // ------------------------------------------------------------------------
    // The Chrono fluid and sph parameters (FSI specific structs).
    // ------------------------------------------------------------------------
    chrono::fsi::ChFluidSystemSPH::FluidProperties fluid_props;
    chrono::fsi::ChFluidSystemSPH::SPHParameters sph_params;

    // ------------------------------------------------------------------------
    // Tis variable tracks which backend is active (Default: Vulkan)
    // ------------------------------------------------------------------------
    ::_acsl_::_system_::RenderBackend m_active_backend = ::_acsl_::_system_::RenderBackend::VULKAN;

    // ------------------------------------------------------------------------
    // The Irrlicht-based visualization system for rendering the simulation.
    // ------------------------------------------------------------------------
    chrono::irrlicht::ChVisualSystemIrrlicht m_irrlicht;
    
    // ------------------------------------------------------------------------
    // The Vulkan-based visualization system for rendering the simulation.
    // ------------------------------------------------------------------------
    chrono::vsg3d::ChVisualSystemVSG m_vsg;

    // ------------------------------------------------------------------------
    // Replacement Vulkan visualization system specifically for fluid cases.
    // ------------------------------------------------------------------------
    std::shared_ptr<chrono::fsi::ChFsiVisualizationVSG> m_visFSI;

    // ------------------------------------------------------------------------
    // The Irrlicht-based system camera for viewing the simulation.
    // ------------------------------------------------------------------------
    std::shared_ptr<chrono::utils::ChChaseCamera> m_camera;

    // ------------------------------------------------------------------------
    // Pointer to the physics solver used by the simulation.
    // Configured according to the physics config YAML.
    // ------------------------------------------------------------------------
    std::shared_ptr<chrono::ChSolver> m_solver;

    // ------------------------------------------------------------------------
    // Physics configuration data structure loaded from `phy-config.yaml`.
    // ------------------------------------------------------------------------
    physicsonfig phyconfig;

    // ------------------------------------------------------------------------
    // Visualization configuration data structure loaded from `vis-config.yaml`.
    // ------------------------------------------------------------------------
    visionconfig visconfig;

    // ------------------------------------------------------------------------
    // Reads and loads physics configuration from `phy_config_filename`.
    // ------------------------------------------------------------------------
    void ReadPhysicsConfigFile();

    // ------------------------------------------------------------------------
    // Reads and loads visualization configuration from `vis_config_filename`.
    // ------------------------------------------------------------------------
    void ReadVisionConfigFile();
};



}   // namespace _system_

}   // namespace _acsl_

#endif  // SIM_SYSTEM_HPP_