# DIAMANTS Sample Demonstrations

This directory contains interactive demos for the DIAMANTS drone simulation system.

## 🚀 Quick Start

| Demo Type | File | Status | Description |
|-----------|------|--------|-------------|
| **✅ RL Crazyflie** | `Reinforcement_Learning_Guide/DIAMANTS_RL_Crazyflie_Fixed.html` | **FUNCTIONAL** | **Complete RL simulation with mesh loading (RECOMMENDED)** |
| **🔍 RL Reference** | `Reinforcement_Learning_Guide/DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html` | Reference | Working reference implementation |
| **Basic Swarm** | `Frontend_Swarm_Intelligence_Demo/swarm_intelligence_demo.html` | Available | Simple flocking behaviors |
| **Advanced Swarm** | `Frontend_Swarm_Intelligence_Demo/advanced_swarm_demo.html` | Available | Complex multi-agent coordination |
| **Collaborative Scouting** | `Frontend_Swarm_Intelligence_Demo/collaborative_scouting_demo.html` | Available | Exploration scenarios |
| **RL Integration Guide** | `Reinforcement_Learning_Guide/reinforcement_learning_integration_guide.html` | Guide | Step-by-step RL implementation |
| **Advanced RL Features** | `Reinforcement_Learning_Guide/advanced_rl_features_guide.html` | Guide | Neural networks & experience replay |
| **RL Collaborative Demo** | `Reinforcement_Learning_Guide/rl_collaborative_learning_demo.html` | Demo | Live multi-agent learning |

## 🎯 Featured Demo

### 🚁 DIAMANTS_RL_Crazyflie_Fixed.html
**Status**: ✅ **FULLY FUNCTIONAL** (Updated: 19/09/2025)

**Features:**
- ✅ **3D Crazyflie Mesh Loading** - Real cf2_assembly.dae, cw_prop.dae, ccw_prop.dae
- ✅ **Reinforcement Learning** - Collaborative multi-agent system
- ✅ **WebGL Rendering** - Three.js with realistic lighting
- ✅ **Dynamic Targets** - Procedural mission objectives
- ✅ **Real-time Metrics** - Intelligence and performance tracking

**Recent Fixes:**
- 🔧 Scene timing issues resolved
- 🔧 Mesh loading system repaired
- 🔧 JavaScript errors eliminated
- 🔧 RL startup sequence optimized

**Quick Test**: Open file → Wait 3 seconds → RL training starts automatically

## 📂 Structure

```
sample/
├── Frontend_Swarm_Intelligence_Demo/    # Interactive swarm behavior demos
│   ├── DIAMANTS_Complete_Frontend_Swarm_Intelligence_Demo.html
│   └── README.md
└── Reinforcement_Learning_Guide/        # RL implementation guides & demos
    ├── DIAMANTS_RL_Crazyflie_Fixed.html              # ✅ MAIN FUNCTIONAL DEMO
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

### Common Issues:
- **"Fallback Renderer"** → Use `DIAMANTS_RL_Crazyflie_Fixed.html` (resolved)
- **Mesh not loading** → Check `meshes/` directory and HTTP server
- **JavaScript errors** → Open DevTools console for diagnostics

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

## 📈 Version History

### v1.1 (19/09/2025) - Major Fixes
- ✅ **DIAMANTS_RL_Crazyflie_Fixed.html** - Fully functional version
- 🔧 Fixed scene timing and initialization conflicts
- 🔧 Resolved mesh loading system from reference file
- 🔧 Eliminated JavaScript runtime errors
- 🔧 Optimized RL startup sequence
- 📁 Added complete mesh assets (cf2_assembly.dae, props)

### v1.0 (Base)
- 🚁 Initial drone simulation demos
- 🧠 Basic RL implementation
- 🎮 Three.js WebGL rendering

## 🎯 Quick Start Guide

1. **Clone repository**
2. **Start local server** (VS Code Live Server recommended)
3. **Open**: `Reinforcement_Learning_Guide/DIAMANTS_RL_Crazyflie_Fixed.html`
4. **Wait 3 seconds** for automatic RL training start
5. **Use control panel** to interact with simulation

---

**Last Updated**: 19 September 2025  
**Status**: ✅ **PRODUCTION READY** (DIAMANTS_RL_Crazyflie_Fixed.html)  
**Next**: Performance optimization and UI improvements
