# 🚁 DIAMANTS API Layer - Services Web Centralisés
## Architecture unifiée pour éliminer duplications WebSocket

### Structure
```
DIAMANTS_API/
├── services/          # Services backend unifiés
├── api/              # API REST FastAPI
├── clients/          # Clients WebSocket frontend
└── config/           # Configuration centralisée
```

### Services Créés
- ✅ Structure de base créée
- 🔄 Migration bridges WebSocket en cours
- 📋 Voir TODO_API_ARCHITECTURE.md pour plan complet

### Actions Suivantes
1. Migrer websocket_bridge.py vers services/
2. Créer API REST unifiée
3. Centraliser clients frontend
4. Supprimer duplications identifiées

---

# 🚁 DIAMANTS - Quick Deployment Guide

## ⚡ **Quick Start (2 minutes)**

```bash
# Clone the repository
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab

# One-command setup and launch
./DIAMANTS_API/quick-setup.sh
```

## 🎯 **What You Get**

- ✅ **Frontend**: Mission Control System on http://localhost:5550
- ✅ **Backend**: ROS2 SLAM Collaborative System with TMUX
- ✅ **Gazebo**: 8-drone simulation environment
- ✅ **RViz**: Real-time visualization
- ✅ **All dependencies**: Auto-installed and configured

## 📋 **Prerequisites**

- **OS**: Ubuntu 20.04/22.04 LTS
- **ROS2**: Humble (auto-installed if missing)
- **Node.js**: 16+ (auto-installed if missing)
- **Git**: For cloning
- **RAM**: 8GB recommended
- **Storage**: 5GB free space

## 🚀 **Available Commands**

### Quick Setup
```bash
./DIAMANTS_API/quick-setup.sh              # Full setup + launch
./DIAMANTS_API/setup-dependencies.sh       # Dependencies only
```

### Launch Services
```bash
./DIAMANTS_API/start-frontend.sh           # Frontend only
./DIAMANTS_API/start-backend.sh            # Backend only
./DIAMANTS_API/start-all.sh                # Both services
```

### Management
```bash
./DIAMANTS_API/status.sh                   # Check services status
./DIAMANTS_API/stop-all.sh                 # Stop all services
./DIAMANTS_API/restart.sh                  # Restart everything
```

## 🔧 **Advanced Usage**

### Manual Setup
```bash
# 1. Setup dependencies
./DIAMANTS_API/setup-dependencies.sh

# 2. Build backend
cd DIAMANTS_BACKEND && make build

# 3. Start frontend
cd DIAMANTS_FRONTEND/Mission_system && npm run dev

# 4. Start backend SLAM system
cd DIAMANTS_BACKEND && make launch-tmux
```

### Custom Configuration
- **Frontend Port**: Edit `DIAMANTS_FRONTEND/Mission_system/vite.config.js`
- **Backend Config**: Edit `DIAMANTS_BACKEND/config/`
- **ROS2 Settings**: Edit `DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/`

## 🆘 **Troubleshooting**

### Port Conflicts
```bash
./DIAMANTS_API/fix-ports.sh               # Auto-fix port conflicts
```

### Permission Issues
```bash
sudo chown -R $USER:$USER ./DIAMANTS_*    # Fix permissions
```

### ROS2 Issues
```bash
source /opt/ros/humble/setup.bash         # Re-source ROS2
./DIAMANTS_API/diagnose.sh                # Full diagnostic
```

## 📞 **Support**

- **Issues**: Open GitHub issue
- **Docs**: Check `/docs/` folder
- **Logs**: Check `DIAMANTS_BACKEND/logs/`

---

**🎉 Ready to explore autonomous drone swarms? Run `./DIAMANTS_API/quick-setup.sh` now!**
