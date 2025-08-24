#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

"""
Fixed Bridge between OpenPLC (Modbus) and OPC UA Server
Uses correct numeric node IDs for KR360
"""

from pyModbusTCP.client import ModbusClient
from opcua import Client
import time

class PLCOPCUABridge:
    def __init__(self):
        self.modbus_client = ModbusClient(host='127.0.0.1', port=502)
        self.opcua_client = Client("opc.tcp://127.0.0.1:4840")
        self.modbus_connected = False
        self.opcua_connected = False
        self.last_states = {"home": False, "pick": False, "place": False}
        
    def connect(self):
        # Connect to Modbus
        try:
            if self.modbus_client.open():
                self.modbus_connected = True
                print("Connected to OpenPLC via Modbus")
            else:
                print("Failed to connect to OpenPLC")
                return False
        except Exception as e:
            print(f"Modbus connection error: {e}")
            return False
        
        # Connect to OPC UA
        try:
            self.opcua_client.connect()
            self.opcua_connected = True
            print("Connected to KR360 OPC UA Robot Server")
        except Exception as e:
            print(f"OPC UA connection error: {e}")
            return False
            
        return True
    
    def read_plc_outputs(self):
        if not self.modbus_connected:
            return {"home": False, "pick": False, "place": False}
            
        try:
            outputs = self.modbus_client.read_coils(1024, 3)
            if outputs is not None:
                return {
                    "home": outputs[0],
                    "pick": outputs[1],
                    "place": outputs[2]
                }
        except Exception as e:
            print(f"Error reading PLC outputs: {e}")
            
        return {"home": False, "pick": False, "place": False}
    
    def send_opcua_command(self, command):
        if not self.opcua_connected:
            return False
            
        try:
            if command == "home":
                node = self.opcua_client.get_node("ns=2;i=4")  # MoveHome
                node.set_value(True)
                print("Sent HOME command to KR360 OPC UA server")
                
            elif command == "pick":
                node = self.opcua_client.get_node("ns=2;i=5")  # MovePick  
                node.set_value(True)
                print("Sent PICK command to KR360 OPC UA server")
                
            elif command == "place":
                node = self.opcua_client.get_node("ns=2;i=6")  # MovePlace
                node.set_value(True)
                print("Sent PLACE command to KR360 OPC UA server")
                
            return True
            
        except Exception as e:
            print(f"Error sending OPC UA command ({command}): {e}")
            return False
    
    def monitor_loop(self):
        print("Starting PLC to KR360 OPC UA bridge...")
        print("Monitoring OpenPLC outputs and sending to KR360 OPC UA robot server")
        print("Press Ctrl+C to stop")
        
        while True:
            try:
                current_states = self.read_plc_outputs()
                
                for command, active in current_states.items():
                    if active and not self.last_states[command]:
                        print(f"PLC output triggered: {command.upper()}")
                        self.send_opcua_command(command)
                
                self.last_states = current_states.copy()
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("\nKR360 Bridge stopped by user")
                break
            except Exception as e:
                print(f"Monitor error: {e}")
                time.sleep(1)
    
    def disconnect(self):
        if self.modbus_connected:
            self.modbus_client.close()
            print("Disconnected from OpenPLC")
            
        if self.opcua_connected:
            self.opcua_client.disconnect()
            print("Disconnected from KR360 OPC UA server")

def main():
    bridge = PLCOPCUABridge()
    
    if not bridge.connect():
        print("Cannot start KR360 bridge without proper connections")
        return
    
    print("Fixed PLC to KR360 OPC UA Bridge Ready!")
    print("=" * 50)
    
    try:
        bridge.monitor_loop()
    finally:
        bridge.disconnect()

if __name__ == "__main__":
    main()
