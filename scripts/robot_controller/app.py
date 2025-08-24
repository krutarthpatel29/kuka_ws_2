#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

"""
Web-based KR360 Robot Controller with GUI
Self-contained version - no separate template files needed
"""

from flask import Flask, request, jsonify
import threading
import time
import os
from pyModbusTCP.client import ModbusClient
from opcua import Client as OPCUA_Client

app = Flask(__name__)

# Robot controller class (your original logic)
class DirectKR360Controller:
    def __init__(self):
        self.client = ModbusClient(host='127.0.0.1', port=502)
        self.connected = False
        self.current_position = 'Unknown'
        self.last_command = ''
        
        # Expected poses
        self.HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.PICK_POSE = [0.8, -0.5, 0.6, 0.3, 0.4, -0.2]
        self.PLACE_POSE = [-0.9, -0.3, -0.4, 1.5, 1.3, 0.5]
        self.POSE_EPS = 0.05
        
        # OPC UA settings
        self.OPCUA_URL = "opc.tcp://127.0.0.1:4840"
        self.JOINT_NODE_ID = "ns=2;i=2"
        
    def poses_close(self, a, b, eps=None):
        """Check if two poses are close within tolerance"""
        if eps is None:
            eps = self.POSE_EPS
        if len(a) != len(b):
            return False
        return all(abs(x - y) <= eps for x, y in zip(a, b))
    
    def connect(self):
        """Connect to robot controller"""
        try:
            if self.client.open():
                self.connected = True
                self.current_position = 'HOME'
                return True
        except Exception as e:
            print(f"Connection error: {e}")
        return False
    
    def disconnect(self):
        """Disconnect from robot controller"""
        if self.connected:
            self.client.close()
            self.connected = False
            self.current_position = 'Unknown'
    
    def move_home(self):
        """Move robot to HOME position"""
        if not self.connected:
            return False
        try:
            success = self.client.write_single_coil(1024, True)
            if success:
                time.sleep(0.5)
                self.client.write_single_coil(1024, False)
                self.current_position = 'HOME'
                self.last_command = 'Move to HOME'
                return True
        except Exception as e:
            print(f"HOME move error: {e}")
        return False
    
    def move_pick(self):
        """Move robot to PICK position"""
        if not self.connected:
            return False
        try:
            success = self.client.write_single_coil(1025, True)
            if success:
                time.sleep(0.5)
                self.client.write_single_coil(1025, False)
                self.current_position = 'PICK'
                self.last_command = 'Move to PICK'
                return True
        except Exception as e:
            print(f"PICK move error: {e}")
        return False
    
    def move_place(self):
        """Move robot to PLACE position"""
        if not self.connected:
            return False
        try:
            success = self.client.write_single_coil(1026, True)
            if success:
                time.sleep(0.5)
                self.client.write_single_coil(1026, False)
                self.current_position = 'PLACE'
                self.last_command = 'Move to PLACE'
                return True
        except Exception as e:
            print(f"PLACE move error: {e}")
        return False
    
    def manual_joint_input(self, joint_values):
        """Handle manual joint input"""
        if not self.connected:
            return {"success": False, "message": "Not connected to robot"}
        
        # Check against known poses
        if self.poses_close(joint_values, self.HOME_POSE):
            success = self.move_home()
            return {
                "success": success,
                "message": "Values match HOME pose - moving robot",
                "action": "move",
                "position": "HOME"
            }
        elif self.poses_close(joint_values, self.PICK_POSE):
            success = self.move_pick()
            return {
                "success": success,
                "message": "Values match PICK pose - moving robot",
                "action": "move",
                "position": "PICK"
            }
        elif self.poses_close(joint_values, self.PLACE_POSE):
            success = self.move_place()
            return {
                "success": success,
                "message": "Values match PLACE pose - moving robot",
                "action": "move",
                "position": "PLACE"
            }
        else:
            # Send to OPC UA for AI testing only
            try:
                opc_client = OPCUA_Client(self.OPCUA_URL)
                opc_client.connect()
                joint_node = opc_client.get_node(self.JOINT_NODE_ID)
                joint_node.set_value(joint_values)
                opc_client.disconnect()
                
                self.last_command = 'Manual Joint Input (Testing)'
                return {
                    "success": True,
                    "message": "Values sent for testing only - robot NOT moved",
                    "action": "test",
                    "values": joint_values
                }
            except Exception as e:
                return {
                    "success": False,
                    "message": f"Testing system error: {str(e)}"
                }

# Global robot controller instance
robot = DirectKR360Controller()

# HTML content embedded in Python
HTML_CONTENT = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>KUKA KR360 Robot Controller</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Arial', sans-serif;
            background-color: #1a1a1a;
            color: white;
            padding: 20px;
            min-height: 100vh;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
        }

        .header {
            text-align: center;
            margin-bottom: 30px;
        }

        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
        }

        .header p {
            color: #888;
        }

        .status-panel {
            background: #2a2a2a;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
        }

        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 20px;
            margin-top: 15px;
        }

        .status-item {
            font-size: 14px;
        }

        .status-item .label {
            color: #888;
        }

        .status-item .value {
            font-weight: bold;
            margin-left: 10px;
        }

        .main-content {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }

        .control-panel, .log-panel {
            background: #2a2a2a;
            border-radius: 10px;
            padding: 20px;
        }

        .control-panel h2, .log-panel h2 {
            margin-bottom: 20px;
            font-size: 1.5em;
        }

        .btn {
            width: 100%;
            padding: 15px;
            margin-bottom: 10px;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
        }

        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }

        .btn-connect {
            background: #3b82f6;
            color: white;
        }

        .btn-connect:hover:not(:disabled) {
            background: #2563eb;
        }

        .btn-disconnect {
            background: #ef4444;
            color: white;
        }

        .btn-disconnect:hover:not(:disabled) {
            background: #dc2626;
        }

        .btn-home {
            background: #10b981;
            color: white;
        }

        .btn-home:hover:not(:disabled) {
            background: #059669;
        }

        .btn-pick {
            background: #f59e0b;
            color: white;
        }

        .btn-pick:hover:not(:disabled) {
            background: #d97706;
        }

        .btn-place {
            background: #8b5cf6;
            color: white;
        }

        .btn-place:hover:not(:disabled) {
            background: #7c3aed;
        }

        .btn-manual {
            background: #eab308;
            color: black;
        }

        .btn-manual:hover:not(:disabled) {
            background: #ca8a04;
        }

        .warning-box {
            background: #451a03;
            border: 1px solid #f59e0b;
            border-radius: 8px;
            padding: 15px;
            margin-top: 15px;
            font-size: 14px;
            color: #fbbf24;
        }

        .log-container {
            background: #1a1a1a;
            border-radius: 8px;
            padding: 15px;
            height: 320px;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 13px;
        }

        .log-entry {
            margin-bottom: 8px;
        }

        .log-time {
            color: #888;
        }

        .log-success {
            color: #10b981;
        }

        .log-error {
            color: #ef4444;
        }

        .log-warning {
            color: #f59e0b;
        }

        .log-info {
            color: #d1d5db;
        }

        .status-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }

        .status-connected {
            background: #10b981;
        }

        .status-disconnected {
            background: #ef4444;
        }

        .modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.7);
            z-index: 1000;
        }

        .modal-content {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: #2a2a2a;
            border-radius: 10px;
            padding: 30px;
            width: 90%;
            max-width: 500px;
        }

        .modal-header {
            margin-bottom: 20px;
        }

        .modal-header h3 {
            margin-bottom: 10px;
        }

        .modal-header p {
            color: #888;
            font-size: 14px;
        }

        .joint-input {
            display: flex;
            align-items: center;
            margin-bottom: 10px;
        }

        .joint-input label {
            width: 80px;
            font-size: 14px;
        }

        .joint-input input {
            flex: 1;
            padding: 8px;
            background: #1a1a1a;
            border: 1px solid #444;
            border-radius: 5px;
            color: white;
        }

        .preset-buttons {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
        }

        .btn-preset {
            padding: 8px 12px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 12px;
            font-weight: bold;
        }

        .modal-actions {
            display: flex;
            gap: 10px;
        }

        .modal-actions button {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 8px;
            font-weight: bold;
            cursor: pointer;
        }

        @media (max-width: 768px) {
            .main-content {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>KUKA KR360 Robot Controller</h1>
            <p>Professional Industrial Robot Control System</p>
        </div>

        <div class="status-panel">
            <div style="display: flex; align-items: center; justify-content: space-between;">
                <h2>System Status</h2>
                <div style="display: flex; align-items: center;">
                    <span class="status-indicator" id="statusIndicator"></span>
                    <span id="connectionStatus">Disconnected</span>
                </div>
            </div>
            <div class="status-grid">
                <div class="status-item">
                    <span class="label">Connection:</span>
                    <span class="value" id="connectionDetails">Disconnected</span>
                </div>
                <div class="status-item">
                    <span class="label">Position:</span>
                    <span class="value" id="robotPosition">Unknown</span>
                </div>
                <div class="status-item">
                    <span class="label">Last Command:</span>
                    <span class="value" id="lastCommand">None</span>
                </div>
            </div>
        </div>

        <div class="main-content">
            <div class="control-panel">
                <h2>Robot Controls</h2>
                
                <button class="btn btn-connect" id="connectBtn" onclick="toggleConnection()">
                    Connect to Robot
                </button>
                
                <button class="btn btn-home" id="homeBtn" onclick="moveToPosition('HOME')" disabled>
                    HOME Position
                </button>
                
                <button class="btn btn-pick" id="pickBtn" onclick="moveToPosition('PICK')" disabled>
                    PICK Position
                </button>
                
                <button class="btn btn-place" id="placeBtn" onclick="moveToPosition('PLACE')" disabled>
                    PLACE Position
                </button>
                
                <button class="btn btn-manual" id="manualBtn" onclick="openManualInput()" disabled>
                    Manual Joint Input
                </button>
                
                <div class="warning-box">
                    <strong>Notice:</strong> All operations are logged and monitored for optimal performance
                </div>
            </div>

            <div class="log-panel">
                <h2>System Log</h2>
                <div class="log-container" id="logContainer">
                    <div class="log-entry log-info">
                        <span class="log-time">[System]</span> Ready for connection...
                    </div>
                </div>
            </div>
        </div>

        <div class="modal" id="manualModal">
            <div class="modal-content">
                <div class="modal-header">
                    <h3>Manual Joint Input</h3>
                    <p>Enter joint values in radians for precise robot positioning.</p>
                </div>
                
                <div id="jointInputs"></div>
                
                <div class="preset-buttons">
                    <button class="btn-preset btn-home" onclick="loadPreset('HOME')">Load HOME</button>
                    <button class="btn-preset btn-pick" onclick="loadPreset('PICK')">Load PICK</button>
                    <button class="btn-preset btn-place" onclick="loadPreset('PLACE')">Load PLACE</button>
                </div>
                
                <div class="modal-actions">
                    <button class="btn btn-connect" onclick="sendManualCommand()">Send Command</button>
                    <button class="btn" style="background: #6b7280;" onclick="closeManualInput()">Cancel</button>
                </div>
            </div>
        </div>
    </div>

    <script>
        let isConnected = false;
        let currentPosition = 'Unknown';
        let lastCommand = '';

        const HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        const PICK_POSE = [0.8, -0.5, 0.6, 0.3, 0.4, -0.2];
        const PLACE_POSE = [-0.9, -0.3, -0.4, 1.5, 1.3, 0.5];
        const POSE_EPS = 0.05;

        function initJointInputs() {
            const container = document.getElementById('jointInputs');
            for (let i = 0; i < 6; i++) {
                const div = document.createElement('div');
                div.className = 'joint-input';
                div.innerHTML = `
                    <label>Joint ${i + 1}:</label>
                    <input type="number" step="0.1" value="0.0" id="joint${i}">
                `;
                container.appendChild(div);
            }
        }

        function getCurrentTime() {
            return new Date().toLocaleTimeString();
        }

        function addLog(message, type = 'info') {
            const logContainer = document.getElementById('logContainer');
            const entry = document.createElement('div');
            entry.className = `log-entry log-${type}`;
            entry.innerHTML = `<span class="log-time">[${getCurrentTime()}]</span> ${message}`;
            logContainer.appendChild(entry);
            logContainer.scrollTop = logContainer.scrollHeight;
        }

        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    const statusIndicator = document.getElementById('statusIndicator');
                    const connectionStatus = document.getElementById('connectionStatus');
                    const connectionDetails = document.getElementById('connectionDetails');
                    const robotPosition = document.getElementById('robotPosition');
                    const lastCommandEl = document.getElementById('lastCommand');

                    isConnected = data.connected;
                    currentPosition = data.position;
                    lastCommand = data.last_command;

                    if (isConnected) {
                        statusIndicator.className = 'status-indicator status-connected';
                        connectionStatus.textContent = 'Connected';
                        connectionDetails.textContent = 'OpenPLC (127.0.0.1:502)';
                    } else {
                        statusIndicator.className = 'status-indicator status-disconnected';
                        connectionStatus.textContent = 'Disconnected';
                        connectionDetails.textContent = 'Disconnected';
                    }

                    robotPosition.textContent = currentPosition;
                    lastCommandEl.textContent = lastCommand || 'None';
                    updateButtons();
                });
        }

        function updateButtons() {
            const buttons = ['homeBtn', 'pickBtn', 'placeBtn', 'manualBtn'];
            buttons.forEach(btnId => {
                document.getElementById(btnId).disabled = !isConnected;
            });

            const connectBtn = document.getElementById('connectBtn');
            if (isConnected) {
                connectBtn.textContent = 'Disconnect';
                connectBtn.className = 'btn btn-disconnect';
            } else {
                connectBtn.textContent = 'Connect to Robot';
                connectBtn.className = 'btn btn-connect';
            }
        }

        function toggleConnection() {
            fetch('/api/connect', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    addLog(data.message, data.success ? 'success' : 'error');
                    updateStatus();
                });
        }

        function moveToPosition(position) {
            fetch(`/api/move/${position}`, { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    addLog(data.message, data.success ? 'success' : 'error');
                    setTimeout(updateStatus, 2000);
                });
        }

        function openManualInput() {
            document.getElementById('manualModal').style.display = 'block';
        }

        function closeManualInput() {
            document.getElementById('manualModal').style.display = 'none';
        }

        function loadPreset(presetName) {
            let pose;
            switch (presetName) {
                case 'HOME': pose = HOME_POSE; break;
                case 'PICK': pose = PICK_POSE; break;
                case 'PLACE': pose = PLACE_POSE; break;
            }
            
            for (let i = 0; i < 6; i++) {
                document.getElementById(`joint${i}`).value = pose[i].toString();
            }
            
            addLog(`Loaded ${presetName} pose values`, 'info');
        }

        function posesClose(a, b, eps = POSE_EPS) {
            if (a.length !== b.length) return false;
            return a.every((x, i) => Math.abs(x - b[i]) <= eps);
        }

        function sendManualCommand() {
            const jointValues = [];
            for (let i = 0; i < 6; i++) {
                const value = parseFloat(document.getElementById(`joint${i}`).value);
                if (isNaN(value)) {
                    addLog('Error: Invalid joint values entered', 'error');
                    return;
                }
                jointValues.push(value);
            }

            fetch('/api/manual', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ joints: jointValues })
            })
            .then(response => response.json())
            .then(data => {
                addLog(data.message, data.success ? 'success' : 'error');
                closeManualInput();
                setTimeout(updateStatus, 1000);
            });
        }

        window.onclick = function(event) {
            const modal = document.getElementById('manualModal');
            if (event.target === modal) {
                closeManualInput();
            }
        }

        initJointInputs();
        updateStatus();
        setInterval(updateStatus, 5000); // Update status every 5 seconds
    </script>
</body>
</html>'''

# Flask routes
@app.route('/')
def index():
    """Serve the main page"""
    return HTML_CONTENT

@app.route('/api/connect', methods=['POST'])
def api_connect():
    """Handle connection requests"""
    if not robot.connected:
        success = robot.connect()
        return jsonify({
            "success": success,
            "message": "Connected to OpenPLC" if success else "Failed to connect",
            "connected": robot.connected
        })
    else:
        robot.disconnect()
        return jsonify({
            "success": True,
            "message": "Disconnected from OpenPLC",
            "connected": robot.connected
        })

@app.route('/api/move/<position>', methods=['POST'])
def api_move(position):
    """Handle movement commands"""
    if not robot.connected:
        return jsonify({"success": False, "message": "Not connected to robot"})
    
    position = position.upper()
    if position == 'HOME':
        success = robot.move_home()
    elif position == 'PICK':
        success = robot.move_pick()
    elif position == 'PLACE':
        success = robot.move_place()
    else:
        return jsonify({"success": False, "message": "Invalid position"})
    
    return jsonify({
        "success": success,
        "message": f"Moving to {position}" if success else f"Failed to move to {position}",
        "position": robot.current_position
    })

@app.route('/api/manual', methods=['POST'])
def api_manual():
    """Handle manual joint input"""
    data = request.json
    joint_values = data.get('joints', [])
    
    if len(joint_values) != 6:
        return jsonify({"success": False, "message": "Must provide 6 joint values"})
    
    try:
        joint_values = [float(val) for val in joint_values]
    except ValueError:
        return jsonify({"success": False, "message": "Invalid joint values"})
    
    result = robot.manual_joint_input(joint_values)
    return jsonify(result)

@app.route('/api/status', methods=['GET'])
def api_status():
    """Get current robot status"""
    return jsonify({
        "connected": robot.connected,
        "position": robot.current_position,
        "last_command": robot.last_command
    })

@app.route('/api/presets', methods=['GET'])
def api_presets():
    """Get preset poses"""
    return jsonify({
        "HOME": robot.HOME_POSE,
        "PICK": robot.PICK_POSE,
        "PLACE": robot.PLACE_POSE
    })

if __name__ == '__main__':
    print("Starting KR360 Robot Controller Web Interface...")
    print("Open your browser to: http://localhost:5000")
    print("Press Ctrl+C to stop")
    
    # Run Flask app
    app.run(host='0.0.0.0', port=5000, debug=True)
