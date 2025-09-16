# DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
# 
# Copyright (c) 2025 DIAMANTS Project Contributors
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!/usr/bin/env python3
"""
Test simple du serveur web DIAMANTS V3
======================================
Version simplifiée pour test sans ROS2
"""

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import os
import json
from pathlib import Path

# Configuration
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

# Déterminer les chemins
current_dir = Path(__file__).parent
static_dir = current_dir / "static"
templates_dir = current_dir / "templates"

# Créer l'application FastAPI
app = FastAPI(
    title="🚁 DIAMANTS V3 Dashboard",
    description="Interface web pour contrôle essaim de drones",
    version="1.0.0"
)

# Configuration des fichiers statiques
if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
    print(f"✅ Fichiers statiques montés: {static_dir}")
else:
    print(f"⚠️ Répertoire static non trouvé: {static_dir}")

# Configuration des templates
if templates_dir.exists():
    templates = Jinja2Templates(directory=str(templates_dir))
    print(f"✅ Templates configurés: {templates_dir}")
else:
    print(f"⚠️ Répertoire templates non trouvé: {templates_dir}")
    templates = None

@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):
    """Page dashboard principale"""
    if templates:
        return templates.TemplateResponse("dashboard.html", {"request": request})
    else:
        return HTMLResponse("""
        <html>
            <head><title>DIAMANTS V3 Test</title></head>
            <body>
                <h1>🚁 DIAMANTS V3 Dashboard Test</h1>
                <p>Serveur web démarré avec succès!</p>
                <p>Chemin actuel: {}</p>
                <p>Static dir: {}</p>
                <p>Templates dir: {}</p>
            </body>
        </html>
        """.format(current_dir, static_dir, templates_dir))

@app.get("/api/status")
async def api_status():
    """API status pour test"""
    return {
        "status": "running",
        "service": "DIAMANTS V3 Web Interface",
        "version": "1.0.0",
        "paths": {
            "current": str(current_dir),
            "static": str(static_dir),
            "templates": str(templates_dir)
        }
    }

@app.get("/api/swarm/status")
async def swarm_status():
    """API status essaim (données simulées)"""
    return {
        "active_drones": 3,
        "intelligence_score": 0.85,
        "coverage_area": 0.67,
        "mission_status": "exploration",
        "timestamp": "2025-08-22T19:45:00Z"
    }

@app.get("/test")
async def test_page():
    """Page de test simple"""
    return {"message": "DIAMANTS V3 Web Interface Test OK! 🚁💎"}

if __name__ == "__main__":
    print(f"🚀 Démarrage serveur web DIAMANTS V3")
    print(f"📂 Répertoire courant: {current_dir}")
    print(f"🎨 Static: {static_dir}")
    print(f"📄 Templates: {templates_dir}")
    print(f"🌐 URL: http://{WEB_HOST}:{WEB_PORT}")
    
    uvicorn.run(
        app,
        host=WEB_HOST,
        port=WEB_PORT,
        log_level="info"
    )