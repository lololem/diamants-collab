# DIAMANTS - Reinforcement Learning Guide

## 🚁 Description
Crazyflie drone simulation with collaborative reinforcement learning based on Three.js and WebGL.

## 📁 Files

### ⚠️ DIAMANTS_RL_Crazyflie_Fixed.html
**Status:** NEEDS FIXES - Version still requires debugging

**Current Issues (19/09/2025):**
- ⚠️ **Mixed Language Content**: French text in HTML comments and interface elements
- ⚠️ **Mesh Loading System**: Path issues with `cf2_assembly.dae`, `cw_prop.dae`, `ccw_prop.dae`
- ⚠️ **JavaScript Conflicts**: Potential initialization conflicts and timing issues
- ⚠️ **RL System Startup**: May not start correctly due to scene timing problems
- ⚠️ **Interface Language**: User interface contains French labels and messages

**Required Fixes:**
- 🔧 **Complete English Translation**: All comments, interface text, and messages
- 🔧 **Mesh Loading Debug**: Verify mesh file paths and loading sequence
- 🔧 **JavaScript Debug**: Resolve initialization conflicts and timing issues  
- 🔧 **RL Startup Testing**: Ensure reinforcement learning system starts properly
- 🔧 **Scene Rendering**: Verify WebGL rendering works without fallback

### 📋 DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html
**Status:** REFERENCE - Working functional file

**Used as reference for:**
- Crazyflie mesh loading structure
- Three.js renderer configuration
- Component initialization order
- Rendering and animation loop

## 🔧 Installation

### Prerequisites
1. Local HTTP server (e.g., VS Code Live Server)
2. Mesh files in `meshes/` folder:
   ```
   meshes/
   ├── cf2_assembly.dae
   ├── cw_prop.dae
   └── ccw_prop.dae
   ```

### Launch
1. Start local server on port 5500
2. Open `DIAMANTS_RL_Crazyflie_Fixed.html`
3. Check browser console for errors and debug information
4. **Note**: Current version may not work properly due to known issues

## 🎮 Features

### ⚠️ 3D Rendering System
- 🔧 Three.js scene with WebGL (needs debugging)
- 🔧 Crazyflie meshes dynamically loaded (path issues)
- 🔧 Realistic lighting and shadows (may not work)
- 🔧 Perspective camera with controls (needs testing)

### ⚠️ RL System (Reinforcement Learning)
- 🔧 Collaborative agents (startup issues)
- 🔧 Exploration and exploitation (needs verification)
- 🔧 Performance metrics (may not display correctly)
- 🔧 Dynamic targets and obstacles (scene dependency issues)

### ⚠️ User Interface
- 🔧 Mission control panel (mixed languages)
- 🔧 DIAMANTS intelligence metrics (needs translation)
- 🔧 Debug logs (French content)
- 🔧 Diagnostic buttons (inconsistent functionality)

## 🐛 Current Issues

### ⚠️ High Priority Fixes Needed
1. **Language Translation**: All French content needs English translation
2. **Mesh Loading**: Verify paths and loading sequence for .dae files
3. **Scene Rendering**: May still show fallback renderer instead of 3D meshes
4. **RL System**: Startup sequence may fail due to timing issues
5. **JavaScript Errors**: Console may show initialization conflicts

### 🔄 Future Improvements
1. **UI/UX**: Improve user interface consistency
2. **RL Algorithm**: Optimize learning algorithms
3. **Multi-drone**: Extend to more simultaneous drones
4. **Metrics**: Add advanced performance metrics

## 📊 Diagnostics

### Available Debug Commands
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

### Expected vs. Actual Logs
**Expected (if working):**
```
✅ WebGL Renderer created
✅ Global scope assignments completed
✅ Mesh loaded successfully
✅ Scene detected - restarting RL
```

**Actual (current issues):**
```
⚠️ Mixed language content
⚠️ Potential mesh loading failures
⚠️ JavaScript initialization conflicts
⚠️ RL startup timing issues
```

## 🛠️ Development

### Code Structure
```
DIAMANTS_RL_Crazyflie_Fixed.html
├── Configuration (CONFIG)
├── RL Classes (CollaborativeLearningSystem) 
├── Mesh Loading (loadColladaMesh, preloadCrazyflieMeshes)
├── 3D Initialization (initScene, renderer)
├── Animation Loop (animate)
├── UI Interface (buttons, metrics)
└── Event Management (DOMContentLoaded)
```

### Execution Order (Intended)
1. DOMContentLoaded
2. initScene() → Create scene/renderer/camera
3. initLoaders() → Load meshes
4. animate() → Start render loop
5. setTimeout(3s) → startRLTrainingMission()

**Note**: Current implementation may have timing and language issues that prevent proper execution.

## 📝 Changelog

### v1.1 (19/09/2025) - Status Correction
- ⚠️ **Status Updated**: File requires significant fixes, not functional
- 🔧 **Issues Identified**: Mixed language content, mesh loading problems
- 🔧 **Debugging Required**: JavaScript conflicts and RL startup issues
- 🔧 **Translation Needed**: Complete English translation required
- 📋 **Documentation**: README corrected to reflect actual status

### v1.0 (Base)
- 🚁 Crazyflie drone simulation (basic implementation)
- 🧠 Collaborative RL system (needs debugging)
- 🎮 Three.js WebGL interface (requires fixes)

---

**Last Updated:** 19 September 2025
**Status:** ⚠️ **REQUIRES MAJOR FIXES** (translation, debugging, testing needed)
