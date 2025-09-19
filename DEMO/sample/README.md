# DIAMANTS Sample Demonstrations

This directory contains interactive demos for the DIAMANTS drone simulation system.

## 🚀 Quick Start

| Demo Type | File | Status | Description |
|-----------|------|--------|-------------|
| **⚠️ RL Crazyflie** | `Reinforcement_Learning_Guide/DIAMANTS_RL_Crazyflie_Fixed.html` | **NEEDS FIXES** | **Mesh loading issues, mixed languages** |
| **🔍 RL Reference** | `Reinforcement_Learning_Guide/DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html` | Reference | Working reference implementation |
| **Basic Swarm** | `Frontend_Swarm_Intelligence_Demo/swarm_intelligence_demo.html` | Available | Simple flocking behaviors |
| **Advanced Swarm** | `Frontend_Swarm_Intelligence_Demo/advanced_swarm_demo.html` | Available | Complex multi-agent coordination |
| **Collaborative Scouting** | `Frontend_Swarm_Intelligence_Demo/collaborative_scouting_demo.html` | Available | Exploration scenarios |
| **RL Integration Guide** | `Reinforcement_Learning_Guide/reinforcement_learning_integration_guide.html` | Guide | Step-by-step RL implementation |
| **Advanced RL Features** | `Reinforcement_Learning_Guide/advanced_rl_features_guide.html` | Guide | Neural networks & experience replay |
| **RL Collaborative Demo** | `Reinforcement_Learning_Guide/rl_collaborative_learning_demo.html` | Demo | Live multi-agent learning |

## 🎯 Current Status

### 🚁 DIAMANTS_RL_Crazyflie_Fixed.html
**Status**: ⚠️ **REQUIRES FIXES** (Updated: 19/09/2025)

**Known Issues:**
- ⚠️ **Mixed Languages** - French content in HTML comments and interface
- ⚠️ **Mesh Loading** - Path issues with cf2_assembly.dae, cw_prop.dae, ccw_prop.dae
- ⚠️ **Scene Rendering** - Potential WebGL initialization conflicts
- ⚠️ **RL System** - May not start correctly due to initialization issues

**Features (When Working):**
- 🔧 **3D Crazyflie Mesh Loading** - Real cf2_assembly.dae, cw_prop.dae, ccw_prop.dae
- 🔧 **Reinforcement Learning** - Collaborative multi-agent system
- 🔧 **WebGL Rendering** - Three.js with realistic lighting
- 🔧 **Dynamic Targets** - Procedural mission objectives
- 🔧 **Real-time Metrics** - Intelligence and performance tracking

**Remaining Work:**
- 🔧 Translation to English (interface, comments, messages)
- 🔧 Fix mesh loading system
- 🔧 Resolve JavaScript initialization conflicts
- 🔧 Test RL startup sequence

## 📂 Structure

```
sample/
├── Frontend_Swarm_Intelligence_Demo/    # Interactive swarm behavior demos
│   ├── DIAMANTS_Complete_Frontend_Swarm_Intelligence_Demo.html
│   └── README.md
└── Reinforcement_Learning_Guide/        # RL implementation guides & demos
    ├── DIAMANTS_RL_Crazyflie_Fixed.html              # ⚠️ NEEDS FIXES
    ├── DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html  # Reference
    ├── meshes/                                        # 3D Models
    │   ├── cf2_assembly.dae
    │   ├── cw_prop.dae
    │   └── ccw_prop.dae
    ├── DIAMANTS_Multi_Agent_Reinforcement_Learning_Guide.md
    └── README.md
```

## 🛠️ Technical Requirements

**System Requirements:**
- Modern browser with WebGL 2.0 support
- Three.js r128+ compatible
- Local HTTP server (for mesh loading)

**Recommended Setup:**
1. Use VS Code with Live Server extension
2. Open files via `http://localhost:5500/`
3. Ensure mesh files are accessible at `meshes/` path

## 🔧 Troubleshooting

### Current Known Issues:
- **"Mixed Languages"** → `DIAMANTS_RL_Crazyflie_Fixed.html` contains French content
- **"Mesh not loading"** → Check `meshes/` directory and HTTP server setup
- **"RL not starting"** → JavaScript initialization conflicts need resolution
- **"Fallback Renderer"** → WebGL initialization issues

### Debug Commands:
```javascript
// Check scene status
window.scene
window.renderer
window.camera

// Test mesh loading
testMeshPaths()
checkMeshCache()

// Force RL restart
startRLTrainingMission()
```

### Recommended Fixes Needed:
1. **Translate interface to English**
2. **Fix mesh loading paths**
3. **Resolve JavaScript conflicts**
4. **Test complete RL workflow**

## 📈 Version History

### v1.1 (19/09/2025) - Work in Progress
- ⚠️ **DIAMANTS_RL_Crazyflie_Fixed.html** - Still requires fixes
- 🔧 Mixed language content needs translation
- 🔧 Mesh loading system needs debugging
- 🔧 JavaScript initialization conflicts present
- 🔧 RL startup sequence not fully tested
- 📁 Mesh assets present but paths may be incorrect

### v1.0 (Base)
- 🚁 Initial drone simulation demos
- 🧠 Basic RL implementation
- 🎮 Three.js WebGL rendering

## 🎯 Development Guide

1. **Clone repository**
2. **Start local server** (VS Code Live Server recommended)
3. **Open**: `Reinforcement_Learning_Guide/DIAMANTS_RL_Crazyflie_Fixed.html`
4. **Check browser console** for errors and debug information
5. **Report issues** for mesh loading, language, or RL startup problems

---

**Last Updated**: 19 September 2025  
**Status**: ⚠️ **DEVELOPMENT IN PROGRESS** (Multiple fixes needed)  
**Next**: Complete English translation, fix mesh loading, resolve JS conflicts
