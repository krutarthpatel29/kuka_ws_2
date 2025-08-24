#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

"""
Direct KR360 Robot Controller - Bypasses PLC logic issues
Sets outputs directly to control the KR360 robot.

Adds Command 4: manual joint input for AI anomaly testing.
- If user-entered joints match a known target pose (HOME/PICK/PLACE) within tolerance,
  the corresponding move_*() function is called (robot moves through PLC/OPC chain).
- Otherwise, the joint values are written directly to the OPC UA joint node (AI only;
  robot does NOT move).
"""

from pyModbusTCP.client import ModbusClient
from opcua import Client as OPCUA_Client
import time

# ------------------------------------------------------------------
# Expected KR360 target poses (radians) as used in manual_opcua_server_kr360.py
# ------------------------------------------------------------------
HOME_POSE  = [0.0,  0.0,  0.0, 0.0, 0.0, 0.0]
PICK_POSE  = [0.8, -0.5,  0.6, 0.3, 0.4, -0.2]
PLACE_POSE = [-0.9, -0.3, -0.4, 1.5, 1.3, 0.5]

# Matching tolerance (radians). Increase if needed.
POSE_EPS = 0.05  # ~3 degrees

# OPC UA endpoint + joint node ID
OPCUA_URL = "opc.tcp://127.0.0.1:4840"
JOINT_NODE_ID = "ns=2;i=2"


def poses_close(a, b, eps=POSE_EPS):
    """Return True if two joint vectors are element-wise within eps."""
    if len(a) != len(b):
        return False
    return all(abs(x - y) <= eps for x, y in zip(a, b))


class DirectKR360Controller:
    def __init__(self):
        self.client = ModbusClient(host='127.0.0.1', port=502)
        self.connected = False
        
    def connect(self):
        if self.client.open():
            self.connected = True
            print("Connected to OpenPLC (KR360 Direct Mode)")
            return True
        return False
    
    def move_home(self):
        """Move KR360 robot to HOME position via Modbus pulse."""
        try:
            success = self.client.write_single_coil(1024, True)
            if success:
                print("KR360 Robot moving to HOME position...")
                time.sleep(0.5)
                self.client.write_single_coil(1024, False)
                return True
        except Exception as e:
            print(f"KR360 HOME error: {e}")
        return False
    
    def move_pick(self):
        """Move KR360 robot to PICK position via Modbus pulse."""
        try:
            success = self.client.write_single_coil(1025, True)
            if success:
                print("KR360 Robot moving to PICK position...")
                time.sleep(0.5)
                self.client.write_single_coil(1025, False)
                return True
        except Exception as e:
            print(f"KR360 PICK error: {e}")
        return False
    
    def move_place(self):
        """Move KR360 robot to PLACE position via Modbus pulse."""
        try:
            success = self.client.write_single_coil(1026, True)
            if success:
                print("KR360 Robot moving to PLACE position...")
                time.sleep(0.5)
                self.client.write_single_coil(1026, False)
                return True
        except Exception as e:
            print(f"KR360 PLACE error: {e}")
        return False

    # ------------------------------------------------------------------
    # Command 4: Manual joint input (AI anomaly test)
    # ------------------------------------------------------------------
    def inject_manual_joints(self):
        """
        Prompt user for 6 joint values.
        - If values match HOME/PICK/PLACE (within POSE_EPS), move robot through PLC.
        - Else: write values directly to OPC UA joint node (AI only, no motion).
        """
        try:
            print("\nManual Joint Input (radians). This does NOT move robot unless pose matches HOME/PICK/PLACE.")
            entered = []
            for i in range(6):
                val = float(input(f"  Joint {i+1}: "))
                entered.append(round(val, 6))  # Keep precision for matching

            # Check against known poses
            if poses_close(entered, HOME_POSE):
                print("Values match HOME pose (within tolerance). Commanding HOME move...")
                self.move_home()
                return
            elif poses_close(entered, PICK_POSE):
                print("Values match PICK pose (within tolerance). Commanding PICK move...")
                self.move_pick()
                return
            elif poses_close(entered, PLACE_POSE):
                print("Values match PLACE pose (within tolerance). Commanding PLACE move...")
                self.move_place()
                return

            # Otherwise: send to OPC UA ONLY (AI anomaly test)
            print("Values do NOT match any target pose; sending to OPC UA for AI anomaly evaluation ONLY (robot will NOT move).")
            opc_client = OPCUA_Client(OPCUA_URL)
            opc_client.connect()
            try:
                joint_node = opc_client.get_node(JOINT_NODE_ID)
                joint_node.set_value(entered)
                print(f"Sent manual joint pose to OPC UA: {entered}")
            finally:
                opc_client.disconnect()

        except Exception as e:
            print(f"Manual joint injection error: {e}")

    # ------------------------------------------------------------------
    # User UI loop
    # ------------------------------------------------------------------
    def interactive_control(self):
        print("\nKUKA KR360 Direct Robot Control")
        print("Commands:")
        print("  1 = HOME position")
        print("  2 = PICK position") 
        print("  3 = PLACE position")
        print("  4 = MANUAL joint input (AI test)")
        print("  q = quit")
        
        while True:
            try:
                cmd = input("\nEnter command: ").strip()
                
                if cmd == '1':
                    self.move_home()
                elif cmd == '2':
                    self.move_pick()
                elif cmd == '3':
                    self.move_place()
                elif cmd == '4':
                    self.inject_manual_joints()
                elif cmd == 'q':
                    break
                else:
                    print("Unknown command")
                    
            except KeyboardInterrupt:
                break
    
    def disconnect(self):
        if self.connected:
            self.client.close()
            print("Disconnected from OpenPLC")


def main():
    controller = DirectKR360Controller()
    
    if not controller.connect():
        print("Cannot connect to OpenPLC")
        return
    
    print("KUKA KR360 Direct Control Ready!")
    print("=" * 40)
    print("Note: This bypasses PLC logic and controls KR360 robot directly")
    
    try:
        controller.interactive_control()
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()

