# 🤝 Contributing Guide

Welcome to the DIAMANTS community! This guide explains how to contribute effectively to the project.

## 🌟 Contribution Philosophy

### Core Values

- **Technical Excellence**: Clean, documented, and tested code
- **Open Collaboration**: Knowledge sharing and mutual support
- **Responsible Innovation**: Ethical and sustainable development
- **Inclusion**: All contributions are valued

### Code of Conduct

We follow the [Contributor Covenant](https://www.contributor-covenant.org/):
- Mutual respect and kindness
- Constructive communication
- Inclusivity and diversity
- Focus on ideas, not individuals

## 🚀 Types of Contributions

### 🐛 Bug Fixes

**Process:**
1. **Reproduce** the bug in your environment
2. **Document** reproduction steps
3. **Create an issue** using the bug template
4. **Propose a solution** via Pull Request

**Bug Issue Template:**
```markdown
## 🐛 Bug Description

### Current Behavior
Describe what currently happens

### Expected Behavior
Describe what should happen

### Reproduction Steps
1. Go to...
2. Click on...
3. Observe...

### Environment
- OS: [Ubuntu 22.04]
- ROS2: [Jazzy]
- Python: [3.10]
- Node.js: [18.x]

### Logs
```bash
[Paste relevant logs here]
```

### Proposed Solution (optional)
If you have a solution idea
```

### ✨ New Features

**Process:**
1. **Discuss** the idea in an issue or discussion
2. **Define** technical specifications
3. **Develop** with unit tests
4. **Document** the functionality
5. **Submit** the Pull Request

**Feature Request Template:**
```markdown
## ✨ New Feature

### Problem to Solve
What problem does this feature solve?

### Proposed Solution
Describe your solution in detail

### Use Cases
- As a [user], I want [action] for [benefit]
- As a [developer], I want [action] for [benefit]

### Technical Specifications
- Required APIs
- Interface changes
- Performance impact

### Alternatives Considered
Other approaches considered and why they were discarded
```

### 📚 Documentation

**Documentation Types:**
- **API**: Technical interface documentation
- **Tutorials**: Step-by-step guides
- **Conceptual**: Concept explanations
- **Reference**: Comprehensive documentation

**Standards:**
- Markdown format
- Functional code examples
- Up-to-date screenshots
- Links to external resources

### 🧪 Tests

**Test Types:**
- **Unit**: Individual functions
- **Integration**: Communication between modules
- **End-to-End**: Complete user scenarios
- **Performance**: Benchmarks and stress tests

### 🎨 Interface and UX

**UI/UX Contributions:**
- Mockups and wireframes
- Ergonomic improvements
- New 3D visualizations
- Performance optimizations

## 🛠 Development Environment Setup

### Developer Installation

```bash
# 1. Fork and clone
git clone https://github.com/[YOUR-USERNAME]/diamants-collab.git
cd diamants-collab

# 2. Configure branches
git remote add upstream https://github.com/lololem/diamants-collab.git
git remote set-url origin https://github.com/[YOUR-USERNAME]/diamants-collab.git

# 3. Development installation
./setup_dev.sh

# 4. Verify installation
./scripts/check_dev_environment.sh
```

### Development Structure

```
diamants-collab/
├── .github/
│   ├── workflows/           # CI/CD GitHub Actions
│   ├── ISSUE_TEMPLATE/      # Issue templates
│   └── PULL_REQUEST_TEMPLATE.md
├── docs/
│   ├── api/                 # API documentation
│   ├── tutorials/           # Tutorials
│   └── architecture/        # Technical documentation
├── tests/
│   ├── unit/                # Unit tests
│   ├── integration/         # Integration tests
│   └── e2e/                 # End-to-end tests
├── scripts/
│   ├── setup_dev.sh         # Developer configuration
│   ├── run_tests.sh         # Test execution
│   └── deploy.sh            # Deployment
└── tools/
    ├── linting/             # Linting tools
    ├── formatting/          # Code formatting
    └── benchmarking/        # Benchmark tools
```

### Development Tools

```bash
# Python linting
pip install black flake8 mypy
black DIAMANTS_API/
flake8 DIAMANTS_API/
mypy DIAMANTS_API/

# JavaScript linting
npm install -g eslint prettier
eslint DIAMANTS_FRONTEND/Mission_system/
prettier --write DIAMANTS_FRONTEND/Mission_system/

# Automated tests
./scripts/run_all_tests.sh

# Pre-commit hooks
pre-commit install
```

## 📋 Contribution Workflow

### 1. Preparation

```bash
# Sync with upstream
git fetch upstream
git checkout main
git merge upstream/main

# Create feature branch
git checkout -b feature/descriptive-name
# or
git checkout -b bugfix/issue-123
# or  
git checkout -b docs/update-readme
```

### 2. Development

```bash
# Develop with atomic commits
git add -p  # Interactive selection
git commit -m "feat: add distributed consensus system

- Implementation of Byzantine algorithm
- Complete unit tests
- Updated API documentation

Fixes #123"
```

### 3. Testing and Validation

```bash
# Local tests
./scripts/run_tests.sh
./scripts/check_code_quality.sh

# Integration tests
./scripts/test_integration.sh

# Performance validation
./scripts/benchmark.sh
```

### 4. Pull Request

```bash
# Push branch
git push origin feature/descriptive-name

# Create PR via GitHub interface
# Use PR template
```

## 🏗 Code Standards

### Python (ROS2 Backend)

```python
"""Drone control module.

This module implements control algorithms for Crazyflie drones
in the ROS2 Jazzy environment.

Example:
    >>> controller = DroneController('cf2x_01')
    >>> controller.takeoff(altitude=1.5)
    >>> controller.move_to(Position(1, 1, 1.5))
"""

from typing import List, Optional, Dict
import rclpy
from geometry_msgs.msg import PoseStamped

class DroneController:
    """Controller for Crazyflie drone.
    
    Attributes:
        drone_id: Unique drone identifier
        current_pose: Current drone position
        is_flying: Drone flight state
    """
    
    def __init__(self, drone_id: str) -> None:
        """Initialize the controller.
        
        Args:
            drone_id: Drone identifier (e.g., 'cf2x_01')
            
        Raises:
            ConnectionError: If unable to connect to drone
        """
        self.drone_id = drone_id
        self._setup_ros_node()
    
    def takeoff(self, altitude: float = 1.0) -> bool:
        """Launch drone takeoff.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if takeoff successful, False otherwise
            
        Raises:
            ValueError: If altitude invalid
        """
        if altitude <= 0 or altitude > 5.0:
            raise ValueError(f"Invalid altitude: {altitude}")
        
        # Implementation...
        return True
```

### JavaScript (Three.js Frontend)

```javascript
/**
 * 3D visualization manager for drone swarms.
 * 
 * Uses Three.js for real-time rendering and WebGL for performance.
 * Supports up to 100 simultaneous drones at 60 FPS.
 * 
 * @example
 * const visualizer = new DroneSwarmVisualizer(container);
 * visualizer.addDrone('cf2x_01', {x: 0, y: 0, z: 1});
 * visualizer.startAnimation();
 */

import * as THREE from 'three';

export class DroneSwarmVisualizer {
    /**
     * Create a new swarm visualizer.
     * 
     * @param {HTMLElement} container - DOM container for rendering
     * @param {Object} options - Configuration options
     * @param {number} [options.maxDrones=50] - Maximum number of drones
     * @param {boolean} [options.enableShadows=true] - Enable shadows
     */
    constructor(container, options = {}) {
        this.container = container;
        this.options = {
            maxDrones: 50,
            enableShadows: true,
            ...options
        };
        
        this.drones = new Map();
        this._initializeScene();
    }
    
    /**
     * Add a drone to the visualization.
     * 
     * @param {string} droneId - Unique drone identifier
     * @param {Object} position - Initial position {x, y, z}
     * @returns {DroneVisual} Visual drone instance
     * @throws {Error} If drone already exists
     */
    addDrone(droneId, position) {
        if (this.drones.has(droneId)) {
            throw new Error(`Drone ${droneId} already exists`);
        }
        
        const droneVisual = new DroneVisual(droneId, position);
        this.drones.set(droneId, droneVisual);
        this.scene.add(droneVisual.mesh);
        
        return droneVisual;
    }
}
```

### YAML (ROS2 Configuration)

```yaml
# Multi-drone configuration for Gazebo simulation
# Follows ROS2 Jazzy standards

/**
 * DIAMANTS multi-drone system configuration
 * 
 * @file multi_drone_params.yaml
 * @version 1.0.0
 * @description Parameters for Crazyflie drone swarm simulation
 * @last_updated 2025-09-19
 */

# General system configuration
system:
  name: "DIAMANTS Multi-Drone System"
  version: "1.0.0"
  max_drones: 10
  update_rate: 50  # Hz
  
# Individual drone parameters
drones:
  cf2x_01:
    type: "crazyflie_2x"
    spawn_position: [0.0, 0.0, 0.1]  # [x, y, z] in meters
    max_velocity: 2.0  # m/s
    max_acceleration: 1.0  # m/s²
    
  cf2x_02:
    type: "crazyflie_2x"
    spawn_position: [1.0, 0.0, 0.1]
    max_velocity: 2.0
    max_acceleration: 1.0

# Physics configuration
physics:
  gravity: -9.81  # m/s²
  air_density: 1.225  # kg/m³
  wind_speed: 0.0  # m/s
  
# Network communication
network:
  ros_domain_id: 42
  qos_profile: "sensor_data"
  timeout: 5.0  # seconds
```

## 🔍 Review Process

### Self-Review Checklist

Before submitting a PR, verify:

- [ ] **Code**: Follows project standards
- [ ] **Tests**: All tests pass
- [ ] **Documentation**: Code documented and README updated
- [ ] **Performance**: No regression
- [ ] **Security**: No introduced vulnerabilities
- [ ] **Compatibility**: Compatible with supported versions

### Peer Review

**Required Reviewers:**
- 1 domain expert of modified area
- 1 project maintainer

**Approval Criteria:**
- Correct functionality
- Maintainable code
- Appropriate tests
- Clear documentation

### CI/CD Process

```yaml
# .github/workflows/test.yml
name: Tests and Validation

on: [push, pull_request]

jobs:
  test-backend:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Setup ROS2 Jazzy
        uses: ros-tooling/setup-ros@v0.3
        with:
          required-ros-distributions: jazzy
      - name: Run Backend Tests
        run: |
          cd DIAMANTS_BACKEND
          colcon build
          colcon test
          
  test-api:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Setup Python 3.10
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      - name: Run API Tests
        run: |
          cd DIAMANTS_API
          pip install -r requirements.txt
          python -m pytest tests/
          
  test-frontend:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Setup Node.js 18
        uses: actions/setup-node@v3
        with:
          node-version: '18'
      - name: Run Frontend Tests
        run: |
          cd DIAMANTS_FRONTEND/Mission_system
          npm install
          npm test
          npm run build
```

## 🏆 Contribution Recognition

### Hall of Fame

Major contributors are recognized in:
- **README.md** of the project
- **About page** of the interface
- **Credits** in academic publications

### Point System

- **Bug Fix**: 10 points
- **Feature**: 50 points
- **Documentation**: 20 points
- **Tests**: 15 points
- **Review**: 5 points

### Contribution Badges

- 🐛 **Bug Hunter**: 10 bugs fixed
- ✨ **Feature Creator**: 5 major features
- 📚 **Documentation Master**: 50 pages documented
- 🧪 **Test Champion**: 90% coverage
- 👁️ **Code Reviewer**: 100 reviews

## 📞 Support and Communication

### Communication Channels

- **GitHub Issues**: Bugs and features
- **GitHub Discussions**: General questions
- **Discord**: Real-time chat
- **Email**: contact@diamants.ai

### Support Hours

**Main Maintainers:**
- Monday-Friday: 9am-5pm CET
- Response time: <24h

**Community:**
- 24/7 via Discord and GitHub
- Response time: variable

### Mentoring

**Newcomer Program:**
- Mentor assignment
- Issues labeled "good first issue"
- Pair programming sessions

**Learning Resources:**
- ROS2 tutorials: [ros2_tutorials](link)
- Three.js guides: [threejs_guides](link)
- Git workflow: [git_workflow](link)

---

**Your contribution matters!** Every line of code, every reported bug, every suggestion improves DIAMANTS. Together, we're building the future of distributed robotics. 🚀