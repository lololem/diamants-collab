# 🤝 Contributing to DIAMANTS

Thank you for your interest in contributing to **DIAMANTS**! This document provides guidelines for contributing to this MIT-licensed collaborative SLAM project.

## 📋 Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [License Requirements](#license-requirements)
- [Known Issues & Bugs](#known-issues--bugs)
- [Development Process](#development-process)
- [Pull Request Process](#pull-request-process)
- [Issue Guidelines](#issue-guidelines)
- [Development Setup](#development-setup)

## 🛡️ Code of Conduct

By participating in this project, you agree to abide by our Code of Conduct. Be respectful, inclusive, and constructive in all interactions.

## 🚀 Getting Started

1. **Fork** the repository
2. **Clone** your fork: `git clone https://github.com/yourusername/diamants-collab.git`
3. **Set up** development environment (see [Development Setup](#development-setup))
4. **Create** a feature branch: `git checkout -b feature/amazing-feature`
5. **Make** your changes
6. **Test** thoroughly
7. **Submit** a pull request

## 📄 License Requirements

### MIT License Compliance

This project is licensed under the **MIT License**. All contributions must comply with MIT licensing terms:

1. **All new files** must include the appropriate MIT license header
2. **Existing files** should not have their license headers removed
3. **Third-party code** must be compatible with MIT license
4. **Dependencies** must have MIT-compatible licenses

### Adding License Headers

Use the provided utility script to add headers to new files:

```bash
# Add headers to specific file types
./scripts/add_license_header.sh js        # JavaScript/TypeScript
./scripts/add_license_header.sh py        # Python
./scripts/add_license_header.sh cpp       # C++/C

# Add headers to all supported files
./scripts/add_license_header.sh all
```

---

## 🐛 Known Issues & Bugs

**Last Updated:** September 16, 2025

### 🔴 Critical Issues

#### Backend Launch System
1. **TMUX Session Instability**
   - **Issue**: Backend sessions sometimes fail to initialize completely
   - **Symptoms**: `tmux list-sessions` shows session but processes don't start
   - **Workaround**: Manual restart via `echo "1" | ./launch_slam_collaborative.sh`
   - **Status**: ⚠️ Partially Fixed - improved process management

2. **Process Duplication (RESOLVED)**
   - **Issue**: Multiple Gazebo instances consuming 700%+ CPU
   - **Symptoms**: `gz sim server` processes multiplying uncontrollably
   - **Fix**: ✅ Implemented automatic cleanup in `launch_diamants.sh`
   - **Status**: ✅ Fixed - cleanup_simulation_ghosts() function

#### API Integration
3. **WebSocket Bridge Timeout**
   - **Issue**: API fails to connect to ROS2 backend intermittently
   - **Symptoms**: 404 errors on `/api/status`, timeout on startup
   - **Root Cause**: Endpoint mismatch - script expects `/api/status` but API serves `/`
   - **Status**: 🔍 Identified - needs endpoint standardization

4. **Port Conflicts**
   - **Issue**: Services conflict on ports 8000, 5550, 8765
   - **Symptoms**: "Address already in use" errors
   - **Fix**: ✅ Automatic port cleanup implemented
   - **Status**: ✅ Fixed - robust port management

### ⚠️ Medium Priority Issues

#### Frontend Issues
5. **Three.js Module Loading**
   - **Issue**: ES6 import conflicts with global THREE.js
   - **Symptoms**: Module loading errors in browser console
   - **Pattern**: Uses global `window.THREE` instead of ESM imports
   - **Status**: 🔄 Design Decision - maintaining compatibility

6. **Vite Hot Reload Instability**
   - **Issue**: Development server sometimes loses HMR connection
   - **Symptoms**: Manual refresh required for changes
   - **Workaround**: Restart dev server `npm run dev`
   - **Status**: 🔍 Under Investigation

7. **WebGL Context Loss**
   - **Issue**: Long-running sessions may lose WebGL context
   - **Symptoms**: Black screen, rendering stops
   - **Mitigation**: Automatic context recovery needed
   - **Status**: 📝 Planned Enhancement

#### Launch Script Anomalies
8. **Environment Variable Conflicts**
   - **Issue**: Python venv vs ROS2 environment conflicts
   - **Symptoms**: Import errors, package not found
   - **Fix**: ✅ Proper environment isolation implemented
   - **Status**: ✅ Fixed - deactivate before ROS2 setup

9. **TMUX Window Synchronization**
   - **Issue**: Race condition in multi-window creation
   - **Symptoms**: Some windows fail to start their processes
   - **Mitigation**: Added sleep delays, improved timing
   - **Status**: ⚠️ Improved but monitoring needed

10. **ROS2 Workspace Build Issues**
    - **Issue**: Missing Python dependencies for colcon build
    - **Symptoms**: `empy`, `catkin-pkg-modules` not found
    - **Fix**: ✅ Auto-installation in setup scripts
    - **Status**: ✅ Fixed - dependency management

### 🟡 Low Priority Issues

#### Configuration Issues
11. **Node.js Version Enforcement**
    - **Issue**: Version drift from .nvmrc specification
    - **Symptoms**: Compatibility issues with Three.js
    - **Mitigation**: Strict version checking in scripts
    - **Status**: 📝 Enhancement Planned

12. **Log File Rotation**
    - **Issue**: Backend logs grow indefinitely
    - **Location**: `DIAMANTS_BACKEND/logs/backend.log`
    - **Impact**: Disk space consumption
    - **Status**: 📝 Feature Request

13. **Error Handling in Scripts**
    - **Issue**: Some script failures are silent
    - **Symptoms**: Processes fail without clear error messages
    - **Mitigation**: Enhanced logging and error reporting
    - **Status**: 🔄 Ongoing Improvement

### �️ Flight Dynamics Issues

#### Frontend/Backend Synchronization
14. **Dual Flight Controller Conflict**
    - **Issue**: Two independent flight control systems running simultaneously
    - **Frontend**: `AuthenticCrazyflie` with custom PID controllers
    - **Backend**: ROS2 `FlightController` with Gazebo physics
    - **Symptoms**: Inconsistent drone behavior, physics desynchronization
    - **Status**: 🔴 Critical - requires unified control architecture

15. **cmd_vel WebSocket Bridge Incomplete**
    - **Issue**: ROS2 `cmd_vel` commands not properly relayed to Frontend
    - **Missing**: Propeller RPM synchronization Frontend ↔ Backend
    - **Location**: `sas/simple_websocket_bridge.py` incomplete motor data relay
    - **Impact**: Visual animation doesn't match physics simulation
    - **Status**: 🔴 Critical - motor synchronization broken

16. **Motor RPM Calculation Mismatch**
    - **Frontend**: Uses `propellerAnimation.speeds` with custom calculations
    - **Backend**: Authentic Crazyflie thrust coefficients in ROS2
    - **Issue**: Different motor power calculations cause drift
    - **Status**: ⚠️ Requires unified motor model

#### Control System Conflicts
17. **Multiple Safety Systems**
    - **Issue**: Both Frontend and Backend implement collision avoidance
    - **Conflict**: Commands may be modified twice, causing instability
    - **Files**: `flight-behaviors.js` vs `flight_controller.py`
    - **Status**: 🔍 Needs safety system hierarchy

### 🎮 User Interface Issues

#### Control Panel Inconsistencies  
18. **Multiple UI Dashboards**
    - **Issue**: 3 separate control interfaces with different capabilities
    - **Interfaces**: 
      - `DIAMANTS_FRONTEND/Mission_system/index.html` (Main)
      - `DIAMANTS_BACKEND/core/web_interface/` (Backend dashboard)
      - `slam_collaboratif/.../web_interface/` (ROS2 dashboard)
    - **Problem**: No unified control, conflicting commands
    - **Status**: 🔴 Critical - needs interface consolidation

19. **Incomplete Button Implementations**
    - **Issue**: HTML buttons without corresponding JavaScript functions
    - **Missing Functions**:
      - `changePattern()` - referenced but not implemented
      - `resetSwarm()` - exists but not connected to backend
      - `toggleCollisionDebug()` - partial implementation
      - Formation controls in some dashboards
    - **Status**: ⚠️ High Priority - UI/UX breaking

20. **Mission State Desynchronization**
    - **Issue**: Frontend `MissionManager` vs Backend mission state
    - **Problem**: Mission status inconsistent between interfaces
    - **Symptoms**: Buttons show wrong state, conflicting mission reports
    - **Files**: `mission-manager.js` vs backend mission tracking
    - **Status**: 🔍 Needs unified mission state management

#### Event Handler Issues
21. **Global Function Conflicts**
    - **Issue**: Multiple definitions of same function names
    - **Examples**: `startMission()`, `launchMission()`, `emergencyLand()`
    - **Files**: `drone-panel-controller.js` vs `dashboard.js`
    - **Symptoms**: Last definition wins, unpredictable behavior
    - **Status**: ⚠️ Requires namespace organization

22. **WebGL Canvas Management**
    - **Issue**: UI controls sometimes lose connection to 3D scene
    - **Symptoms**: Buttons work but no visual feedback
    - **Root Cause**: Three.js context not properly passed to UI handlers
    - **Status**: 🔍 Architecture issue

### �🔧 Development Environment Issues

#### Setup Challenges
23. **ROS2 Distribution Compatibility**
    - **Issue**: Mixed support for Humble/Jazzy distributions
    - **Recommendation**: Use ROS2 Jazzy for best compatibility
    - **Status**: 📝 Documentation Updated

24. **Sudo Requirements Removed**
    - **Issue**: Original scripts required sudo for process cleanup
    - **Fix**: ✅ Refactored to user-space operations only
    - **Status**: ✅ Fixed - no sudo required

25. **Path Dependencies**
    - **Issue**: Scripts expect specific working directories
    - **Symptoms**: "File not found" errors when run from wrong location
    - **Status**: 🔍 Needs absolute path resolution

### 🎯 Testing Issues

#### Automated Testing Gaps
26. **Integration Test Coverage**
    - **Issue**: No automated tests for full system integration
    - **Impact**: Regressions not caught early
    - **Status**: 📝 High Priority Development

27. **Performance Benchmarking**
    - **Issue**: No systematic performance monitoring
    - **Need**: Baseline metrics for optimization
    - **Status**: 📝 Feature Request

### 📊 Monitoring & Diagnostics

#### Debugging Tools
28. **Process Monitoring Script Enhanced**
    - **Tool**: `./check_ros_processes.sh`
    - **Status**: ✅ Comprehensive process analysis
    - **Features**: Color-coded output, process classification

29. **Real-time Status Dashboard**
    - **Issue**: No unified system health view
    - **Proposal**: Web-based status interface
    - **Status**: 💡 Future Enhancement

### 🚨 Specific Frontend/Backend Issues Requiring Immediate Attention

#### Critical Flight Dynamics Fixes Needed:
1. **Unified Motor Control**: Implement single source of truth for motor commands
2. **WebSocket Enhancement**: Complete `cmd_vel` to propeller RPM relay
3. **Physics Synchronization**: Ensure Frontend visuals match Backend physics exactly

#### Critical UI Fixes Needed:
1. **Function Implementation**: Complete missing button handlers
2. **Interface Consolidation**: Unify the 3 separate control dashboards  
3. **State Management**: Implement centralized mission/drone state

#### Quick Diagnostic Commands:
```bash
# Check motor RPM sync issues
ros2 topic echo /crazyflie/cmd_vel

# Verify WebSocket bridge
netstat -tlnp | grep 8765

# Test UI function availability
curl -s http://localhost:5550 | grep -E "(onclick|function)"

# Monitor flight controller conflicts  
grep -r "FlightController\|AuthenticCrazyflie" DIAMANTS_*/
```

### 🚨 How to Report New Issues

When reporting issues, include:

1. **System Information**:
   ```bash
   # Run diagnostic
   ./check_ros_processes.sh
   
   # Include environment
   echo "ROS_DISTRO: $ROS_DISTRO"
   node --version
   python3 --version
   ```

2. **Steps to Reproduce**
3. **Expected vs Actual Behavior**
4. **Log Files**: `DIAMANTS_BACKEND/logs/backend.log`
5. **TMUX Session State**: `tmux list-sessions`

### 🔧 Quick Fixes for Common Issues

```bash
# Complete system reset
./stop_diamants.sh && sleep 3 && echo "1" | ./launch_diamants.sh

# Process cleanup only
pkill -f "gz sim|gazebo|rviz"

# Port cleanup
lsof -ti:8000,5550,8765 | xargs kill -9 2>/dev/null

# TMUX session reset
tmux kill-session -t slam_collab 2>/dev/null

# Rebuild ROS2 workspace
cd DIAMANTS_BACKEND/slam_collaboratif/ros2_ws
rm -rf build install log
colcon build
```

---

### License Header Templates

Templates are provided in `.github/` for different file types:

- **JavaScript/TypeScript**: `.github/LICENSE_HEADER_JS.txt`
- **Python**: `.github/LICENSE_HEADER_PYTHON.txt`
- **C++/C**: `.github/LICENSE_HEADER_CPP.txt`

### Manual Header Addition

For new file types, manually add the appropriate header:

```javascript
/*
 * DIAMANTS - Distributed Autonomous Multi-agents Systems for Drone Swarms
 * 
 * Copyright (c) 2025 DIAMANTS Project Contributors
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Your code here...
```

## 🔧 Development Process

### Branch Naming Convention

- `feature/description` - New features
- `bugfix/description` - Bug fixes
- `docs/description` - Documentation updates
- `refactor/description` - Code refactoring
- `test/description` - Test improvements

### Commit Message Format

```
type(scope): description

Detailed explanation if needed

Closes #issue_number
```

**Types**: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

**Examples**:
```
feat(slam): add collaborative mapping for 8 drones
fix(frontend): resolve WebSocket connection issues
docs(readme): update installation instructions
```

### Code Quality Standards

1. **Follow existing code style** in each component
2. **Add MIT license headers** to all new files
3. **Write meaningful commit messages**
4. **Include tests** for new functionality
5. **Update documentation** as needed
6. **Ensure backward compatibility** when possible

## 🔄 Pull Request Process

### Before Submitting

1. **Ensure your fork is up to date**:
   ```bash
   git remote add upstream https://github.com/lololem/diamants.git
   git fetch upstream
   git checkout main
   git merge upstream/main
   ```

2. **Run tests**:
   ```bash
   # Backend tests
   cd DIAMANTS_BACKEND/slam_collaboratif/ros2_ws
   colcon test
   
   # Frontend tests
   cd mission_core/Mission_system
   npm test
   ```

3. **Add license headers**:
   ```bash
   ./scripts/add_license_header.sh all
   ```

4. **Update documentation** if needed

### Pull Request Template

When submitting a PR, include:

- **Description**: What changes were made and why
- **Type**: Feature, bugfix, docs, etc.
- **Testing**: How the changes were tested
- **License Compliance**: Confirm MIT license headers are added
- **Breaking Changes**: Any backward incompatible changes
- **Related Issues**: Reference any related issues

### Review Process

1. **Automated checks** must pass (CI/CD)
2. **Code review** by maintainers
3. **License compliance** verification
4. **Testing verification**
5. **Documentation** check
6. **Approval** and merge

## 📝 Issue Guidelines

### Bug Reports

Include:
- **Environment**: OS, ROS2 version, Node.js version
- **Steps to reproduce**
- **Expected vs actual behavior**
- **Error messages/logs**
- **System configuration**

### Feature Requests

Include:
- **Use case description**
- **Proposed solution**
- **Alternative approaches considered**
- **Impact on existing functionality**

### Security Issues

For security vulnerabilities:
1. **Do not** open a public issue
2. **Email** maintainers directly
3. **Provide** detailed information
4. **Allow** time for fix before disclosure

## 🛠️ Development Setup

### Prerequisites

- **Ubuntu 24.04** (recommended)
- **ROS2 Jazzy**
- **Node.js 20+**
- **Python 3.8+**
- **Git** and **tmux**

### Backend Setup

```bash
# Clone repository
git clone https://github.com/yourusername/diamants.git
cd diamants

# Setup ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build workspace
cd DIAMANTS_BACKEND/slam_collaboratif/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Frontend Setup

```bash
# Navigate to frontend
cd mission_core/Mission_system

# Install dependencies
npm install

# Start development server
npm run dev
```

### Testing Setup

```bash
# Test backend
cd DIAMANTS_BACKEND && ./launch_slam_collaborative.sh

# Test frontend
cd mission_core/Mission_system && npm run dev

# Verify integration
curl http://localhost:3000
tmux list-sessions | grep slam_collab
```

## 📚 Resources

### Documentation

- [Project README](README.md)
- [Backend Documentation](DIAMANTS_BACKEND/README.md)
- [Frontend Documentation](mission_core/Mission_system/README.md)
- [Usage Guide](DIAMANTS_BACKEND/USAGE_GUIDE.md)

### External Resources

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [MIT License Guide](https://opensource.org/licenses/MIT)
- [Conventional Commits](https://www.conventionalcommits.org/)

### Community

- **GitHub Issues**: Technical discussion
- **GitHub Discussions**: General questions
- **Pull Requests**: Code contributions

## 🙏 Recognition

Contributors will be recognized in:
- **README.md** acknowledgments section
- **GitHub contributors** page
- **Release notes** for significant contributions

## ❓ Questions?

- 📫 **Open an issue** for technical questions
- 💬 **Start a discussion** for general questions
- 📧 **Contact maintainers** for private matters

---

**Thank you for contributing to DIAMANTS!** 🚁✨

Your contributions help advance collaborative autonomous systems and open-source robotics.
