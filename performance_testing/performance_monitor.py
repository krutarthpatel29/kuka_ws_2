#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

"""
Performance Monitor for Single KUKA Robot System
Measures latency, throughput, and success rates
"""

import time
import csv
import json
from datetime import datetime
import opcua
from pymodbus.client.sync import ModbusTcpClient
import os

class SingleRobotPerformanceMonitor:
    def __init__(self):
        self.results = {
            "timestamps": [],
            "plc_to_opcua_latency": [],
            "opcua_to_ros_latency": [],
            "total_latency": [],
            "success_count": 0,
            "failure_count": 0,
            "commands_sent": 0,
            "test_duration": 0
        }
        
        # Connect to OPC UA server
        self.opcua_client = opcua.Client("opc.tcp://localhost:4840")
        
        # Connect to PLC
        self.plc_client = ModbusTcpClient('localhost', port=502)
        
        # Output directory
        self.output_dir = "results"
        os.makedirs(self.output_dir, exist_ok=True)
        
    def connect(self):
        """Establish connections"""
        try:
            self.opcua_client.connect()
            self.plc_connected = self.plc_client.connect()
            print("Connected to OPC UA and PLC")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Clean up connections"""
        self.opcua_client.disconnect()
        self.plc_client.close()
    
    def measure_cycle_time(self, command="HOME"):
        """Measure time for one complete command cycle"""
        start_time = time.time()
        
        # Map commands to PLC coils and expected joint positions
        command_map = {
            "HOME": {
                "coil": 1024, 
                "node": "ns=2;i=4",
                "expected_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            "PICK": {
                "coil": 1025, 
                "node": "ns=2;i=5",
                "expected_joints": [0.8, -0.5, 0.6, 0.3, 0.4, -0.2]
            },
            "PLACE": {
                "coil": 1026, 
                "node": "ns=2;i=6",
                "expected_joints": [-0.9, -0.3, -0.4, 1.5, 1.3, 0.5]
            }
        }
        
        if command not in command_map:
            print(f"Unknown command: {command}")
            return None
            
        coil = command_map[command]["coil"]
        expected_pos = command_map[command]["expected_joints"]
        
        # Get initial joint positions
        try:
            joints_node = self.opcua_client.get_node("ns=2;i=2")  # JointPositions
            initial_joints = joints_node.get_value()
        except Exception as e:
            print(f"Error getting joint positions: {e}")
            self.results["failure_count"] += 1
            return None
        
        # Send command via PLC
        self.plc_client.write_coil(coil, True)
        plc_sent_time = time.time()
        
        # Wait for robot to reach target position
        timeout = 5.0
        start_wait = time.time()
        position_reached = False
        
        while (time.time() - start_wait) < timeout:
            try:
                current_joints = joints_node.get_value()
                
                # Check if we've reached the expected position (with tolerance)
                if all(abs(current - expected) < 0.01 for current, expected in zip(current_joints, expected_pos)):
                    opcua_received_time = time.time()
                    position_reached = True
                    break
            except:
                pass
            time.sleep(0.01)
        
        # Reset PLC coil
        self.plc_client.write_coil(coil, False)
        
        if not position_reached:
            print(f"\nTimeout waiting for {command} position")
            self.results["failure_count"] += 1
            return None
        
        # Calculate latencies
        plc_to_opcua = opcua_received_time - plc_sent_time
        total_latency = opcua_received_time - start_time
        
        # Store results
        self.results["timestamps"].append(datetime.now().isoformat())
        self.results["plc_to_opcua_latency"].append(plc_to_opcua * 1000)  # Convert to ms
        self.results["total_latency"].append(total_latency * 1000)  # Convert to ms
        self.results["success_count"] += 1
        self.results["commands_sent"] += 1
        
        return total_latency
    
    def run_performance_test(self, num_cycles=150):
        """Run complete performance test"""
        print(f"\nStarting performance test with {num_cycles} cycles...")
        
        if not self.connect():
            return
        
        test_start = time.time()
        commands = ["HOME", "PICK", "PLACE"]
        
        for i in range(num_cycles):
            command = commands[i % 3]
            print(f"\rCycle {i+1}/{num_cycles}: {command}", end="")
            
            cycle_time = self.measure_cycle_time(command)
            
            # Small delay between commands
            time.sleep(0.5)
        
        self.results["test_duration"] = time.time() - test_start
        
        print(f"\n✓ Test completed in {self.results['test_duration']:.2f} seconds")
        
        self.disconnect()
        self.save_results()
        self.print_summary()
    
    def save_results(self):
        """Save results to CSV and JSON"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save detailed CSV
        csv_file = os.path.join(self.output_dir, f"single_robot_test_{timestamp}.csv")
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "PLC_to_OPCUA_ms", "Total_Latency_ms"])
            
            for i in range(len(self.results["timestamps"])):
                writer.writerow([
                    self.results["timestamps"][i],
                    self.results["plc_to_opcua_latency"][i],
                    self.results["total_latency"][i]
                ])
        
        # Save summary JSON
        json_file = os.path.join(self.output_dir, f"single_robot_summary_{timestamp}.json")
        summary = {
            "test_date": timestamp,
            "total_commands": self.results["commands_sent"],
            "success_count": self.results["success_count"],
            "failure_count": self.results["failure_count"],
            "success_rate": (self.results["success_count"] / self.results["commands_sent"] * 100) if self.results["commands_sent"] > 0 else 0,
            "avg_plc_to_opcua_ms": sum(self.results["plc_to_opcua_latency"]) / len(self.results["plc_to_opcua_latency"]) if self.results["plc_to_opcua_latency"] else 0,
            "avg_total_latency_ms": sum(self.results["total_latency"]) / len(self.results["total_latency"]) if self.results["total_latency"] else 0,
            "max_latency_ms": max(self.results["total_latency"]) if self.results["total_latency"] else 0,
            "min_latency_ms": min(self.results["total_latency"]) if self.results["total_latency"] else 0,
            "test_duration_s": self.results["test_duration"]
        }
        
        with open(json_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"\nResults saved to:")
        print(f"   - {csv_file}")
        print(f"   - {json_file}")
    
    def print_summary(self):
        """Print performance summary"""
        if not self.results["total_latency"]:
            print("\nNo data collected")
            return
        
        print("\n" + "="*50)
        print("PERFORMANCE SUMMARY - SINGLE ROBOT")
        print("="*50)
        print(f"Total Commands Sent: {self.results['commands_sent']}")
        print(f"Successful: {self.results['success_count']}")
        print(f"Failed: {self.results['failure_count']}")
        print(f"Success Rate: {(self.results['success_count'] / self.results['commands_sent'] * 100):.1f}%")
        print(f"\nLatency Statistics (ms):")
        print(f"  - Average PLC→OPC UA: {sum(self.results['plc_to_opcua_latency']) / len(self.results['plc_to_opcua_latency']):.2f}")
        print(f"  - Average Total: {sum(self.results['total_latency']) / len(self.results['total_latency']):.2f}")
        print(f"  - Maximum: {max(self.results['total_latency']):.2f}")
        print(f"  - Minimum: {min(self.results['total_latency']):.2f}")
        print(f"\nThroughput: {self.results['commands_sent'] / self.results['test_duration']:.2f} commands/second")
        print("="*50)

if __name__ == "__main__":
    monitor = SingleRobotPerformanceMonitor()
    
    # Run test with 150 cycles (will take about 3-5 minutes)
    monitor.run_performance_test(num_cycles=150)
