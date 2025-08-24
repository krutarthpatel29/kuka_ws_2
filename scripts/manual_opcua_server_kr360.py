# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

from opcua import Server
import time
import threading

# Create server
server = Server()
server.set_endpoint("opc.tcp://0.0.0.0:4840")

# Setup namespace and object
name = "KukaKR360TestServer"
uri = f"http://{name}"
idx = server.register_namespace(uri)

# Create object and variables
robot_obj = server.nodes.objects.add_object(idx, "KukaKR360Robot")
joint_data = robot_obj.add_variable(idx, "JointPositions", [0.0]*6)  # KR360 has 6 joints
joint_data.set_writable()

# Add command variables for PLC control
plc_commands = robot_obj.add_object(idx, "PLCCommands")
home_cmd = plc_commands.add_variable(idx, "MoveHome", False)
pick_cmd = plc_commands.add_variable(idx, "MovePick", False)
place_cmd = plc_commands.add_variable(idx, "MovePlace", False)

# Make command variables writable
home_cmd.set_writable()
pick_cmd.set_writable()
place_cmd.set_writable()

# Predefined positions for KR360 (6-axis robot)
POSITIONS = {
    "HOME": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "PICK": [0.8, -0.5, 0.6, 0.3, 0.4, -0.2],
    "PLACE": [-0.9, -0.3, -0.4, 1.5, 1.3, 0.5]
}

# PLC monitoring function
def monitor_plc_commands():
    print("KR360 PLC Command Monitor Started")
    
    while True:
        try:
            if home_cmd.get_value():
                print("PLC Command: Move KR360 to HOME")
                joint_data.set_value(POSITIONS["HOME"])
                print(f"Movement completed: HOME -> {POSITIONS['HOME']}")
                home_cmd.set_value(False)
                
            elif pick_cmd.get_value():
                print("PLC Command: Move KR360 to PICK")
                joint_data.set_value(POSITIONS["PICK"])
                print(f"Movement completed: PICK -> {POSITIONS['PICK']}")
                pick_cmd.set_value(False)
                
            elif place_cmd.get_value():
                print("PLC Command: Move KR360 to PLACE")
                joint_data.set_value(POSITIONS["PLACE"])
                print(f"Movement completed: PLACE -> {POSITIONS['PLACE']}")
                place_cmd.set_value(False)
                
        except Exception as e:
            print(f"Error in PLC monitor: {e}")
            
        time.sleep(0.1)

# Start server
server.start()
print("KR360 OPC UA Server started at opc.tcp://localhost:4840")

# Start PLC monitoring
plc_thread = threading.Thread(target=monitor_plc_commands, daemon=True)
plc_thread.start()

print("Server is monitoring PLC commands for KR360 robot...")
print("Press Ctrl+C to stop")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nShutting down KR360 server...")
    server.stop()
