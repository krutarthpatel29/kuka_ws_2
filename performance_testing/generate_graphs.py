#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

"""
Graph Generator for KUKA Robot Performance Results
Creates professional graphs for thesis presentation
"""

import matplotlib.pyplot as plt
import pandas as pd
import json
import numpy as np
import os
from datetime import datetime
import seaborn as sns

# Set style for professional looking graphs
try:
    plt.style.use('seaborn-darkgrid')
except:
    plt.style.use('ggplot')  # Fallback style
sns.set_palette("husl")

class PerformanceGraphGenerator:
    def __init__(self, results_dir="results", graphs_dir="graphs"):
        self.results_dir = results_dir
        self.graphs_dir = graphs_dir
        os.makedirs(self.graphs_dir, exist_ok=True)
        
    def load_latest_results(self, robot_type="single"):
        """Load the most recent test results"""
        # Find latest CSV file
        csv_files = [f for f in os.listdir(self.results_dir) if f.startswith(f"{robot_type}_robot_test") and f.endswith(".csv")]
        if not csv_files:
            print(f"No {robot_type} robot test results found!")
            return None, None
            
        latest_csv = sorted(csv_files)[-1]
        
        # Find corresponding JSON
        json_name = latest_csv.replace("test", "summary").replace(".csv", ".json")
        
        # Load data
        df = pd.read_csv(os.path.join(self.results_dir, latest_csv))
        
        with open(os.path.join(self.results_dir, json_name), 'r') as f:
            summary = json.load(f)
            
        return df, summary
    
    def create_latency_timeline(self, df, robot_type="single"):
        """Create latency over time graph"""
        plt.figure(figsize=(12, 6))
        
        # Convert index to numpy array to avoid pandas compatibility issues
        x_values = df.index.to_numpy()
        
        # Plot both latencies
        plt.plot(x_values, df['PLC_to_OPCUA_ms'].to_numpy(), label='PLC to OPC UA', linewidth=2, marker='o', markersize=4)
        plt.plot(x_values, df['Total_Latency_ms'].to_numpy(), label='Total Latency', linewidth=2, marker='s', markersize=4)
        
        # Add average lines
        avg_plc = df['PLC_to_OPCUA_ms'].mean()
        avg_total = df['Total_Latency_ms'].mean()
        plt.axhline(y=avg_plc, color='blue', linestyle='--', alpha=0.5, label=f'Avg PLC→OPC UA: {avg_plc:.2f}ms')
        plt.axhline(y=avg_total, color='orange', linestyle='--', alpha=0.5, label=f'Avg Total: {avg_total:.2f}ms')
        
        plt.xlabel('Command Number', fontsize=12)
        plt.ylabel('Latency (ms)', fontsize=12)
        plt.title(f'{robot_type.capitalize()} Robot System - Latency Over Time', fontsize=14, fontweight='bold')
        plt.legend(loc='best')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        filename = f"{robot_type}_latency_timeline.png"
        plt.savefig(os.path.join(self.graphs_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        print(f"✓ Saved: {filename}")
    
    def create_latency_distribution(self, df, robot_type="single"):
        """Create latency distribution histogram"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Histogram for PLC to OPC UA
        ax1.hist(df['PLC_to_OPCUA_ms'], bins=20, alpha=0.7, color='skyblue', edgecolor='black')
        ax1.axvline(df['PLC_to_OPCUA_ms'].mean(), color='red', linestyle='--', linewidth=2, label=f'Mean: {df["PLC_to_OPCUA_ms"].mean():.2f}ms')
        ax1.set_xlabel('Latency (ms)', fontsize=12)
        ax1.set_ylabel('Frequency', fontsize=12)
        ax1.set_title('PLC to OPC UA Latency Distribution', fontsize=13)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Histogram for Total Latency
        ax2.hist(df['Total_Latency_ms'], bins=20, alpha=0.7, color='lightcoral', edgecolor='black')
        ax2.axvline(df['Total_Latency_ms'].mean(), color='red', linestyle='--', linewidth=2, label=f'Mean: {df["Total_Latency_ms"].mean():.2f}ms')
        ax2.set_xlabel('Latency (ms)', fontsize=12)
        ax2.set_ylabel('Frequency', fontsize=12)
        ax2.set_title('Total System Latency Distribution', fontsize=13)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.suptitle(f'{robot_type.capitalize()} Robot System - Latency Distributions', fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        filename = f"{robot_type}_latency_distribution.png"
        plt.savefig(os.path.join(self.graphs_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        print(f"✓ Saved: {filename}")
    
    def create_performance_summary(self, summary, robot_type="single"):
        """Create a summary performance metrics visualization"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        
        # 1. Success Rate Pie Chart
        success_rate = summary['success_rate']
        failure_rate = 100 - success_rate
        ax1.pie([success_rate, failure_rate], labels=['Success', 'Failure'], 
                autopct='%1.1f%%', colors=['#90EE90', '#FFB6C1'], startangle=90)
        ax1.set_title('Command Success Rate', fontsize=13, fontweight='bold')
        
        # 2. Latency Comparison Bar Chart
        latency_data = {
            'PLC→OPC UA': summary['avg_plc_to_opcua_ms'],
            'Total System': summary['avg_total_latency_ms']
        }
        bars = ax2.bar(latency_data.keys(), latency_data.values(), color=['#87CEEB', '#FFB347'])
        ax2.set_ylabel('Average Latency (ms)', fontsize=12)
        ax2.set_title('Average Latency Comparison', fontsize=13, fontweight='bold')
        
        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.2f}ms', ha='center', va='bottom')
        
        # 3. Key Metrics Text
        ax3.axis('off')
        metrics_text = f"""
        System Performance Metrics
        
        Total Commands: {summary['total_commands']}
        Test Duration: {summary['test_duration_s']:.1f} seconds
        Throughput: {summary['total_commands']/summary['test_duration_s']:.2f} cmd/sec
        
        Latency Stats:
        • Min: {summary['min_latency_ms']:.2f} ms
        • Max: {summary['max_latency_ms']:.2f} ms
        • Average: {summary['avg_total_latency_ms']:.2f} ms
        """
        ax3.text(0.1, 0.5, metrics_text, fontsize=12, verticalalignment='center',
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
        
        # 4. Latency Range Visualization
        latency_stats = [summary['min_latency_ms'], summary['avg_total_latency_ms'], summary['max_latency_ms']]
        ax4.barh(['Min', 'Average', 'Max'], latency_stats, color=['green', 'orange', 'red'])
        ax4.set_xlabel('Latency (ms)', fontsize=12)
        ax4.set_title('Latency Range', fontsize=13, fontweight='bold')
        
        # Add value labels
        for i, v in enumerate(latency_stats):
            ax4.text(v + 0.5, i, f'{v:.2f}ms', va='center')
        
        plt.suptitle(f'{robot_type.capitalize()} Robot System - Performance Summary', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        filename = f"{robot_type}_performance_summary.png"
        plt.savefig(os.path.join(self.graphs_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved: {filename}")
    
    def create_comparison_graph(self):
        """Create comparison graph between single and multi-robot (if both exist)"""
        # Load both results if available
        single_df, single_summary = self.load_latest_results("single")
        multi_df, multi_summary = self.load_latest_results("multi")
        
        if not single_summary or not multi_summary:
            print("Need both single and multi-robot results for comparison")
            return
        
        # Create comparison
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Latency comparison
        systems = ['Single Robot', 'Multi Robot']
        avg_latencies = [single_summary['avg_total_latency_ms'], multi_summary['avg_total_latency_ms']]
        max_latencies = [single_summary['max_latency_ms'], multi_summary['max_latency_ms']]
        
        x = np.arange(len(systems))
        width = 0.35
        
        bars1 = ax1.bar(x - width/2, avg_latencies, width, label='Average', color='skyblue')
        bars2 = ax1.bar(x + width/2, max_latencies, width, label='Maximum', color='lightcoral')
        
        ax1.set_ylabel('Latency (ms)', fontsize=12)
        ax1.set_title('Latency Comparison: Single vs Multi-Robot', fontsize=13, fontweight='bold')
        ax1.set_xticks(x)
        ax1.set_xticklabels(systems)
        ax1.legend()
        
        # Add value labels
        for bars in [bars1, bars2]:
            for bar in bars:
                height = bar.get_height()
                ax1.text(bar.get_x() + bar.get_width()/2., height,
                        f'{height:.1f}', ha='center', va='bottom')
        
        # Throughput comparison
        throughputs = [
            single_summary['total_commands']/single_summary['test_duration_s'],
            multi_summary['total_commands']/multi_summary['test_duration_s']
        ]
        
        bars = ax2.bar(systems, throughputs, color=['#90EE90', '#87CEEB'])
        ax2.set_ylabel('Commands per Second', fontsize=12)
        ax2.set_title('Throughput Comparison', fontsize=13, fontweight='bold')
        
        for bar in bars:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.2f}', ha='center', va='bottom')
        
        plt.suptitle('Single vs Multi-Robot System Performance', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        filename = "single_vs_multi_comparison.png"
        plt.savefig(os.path.join(self.graphs_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        print(f"✓ Saved: {filename}")
    
    def generate_all_graphs(self, robot_type="single"):
        """Generate all graphs for the specified robot type"""
        print(f"\nGenerating graphs for {robot_type} robot system...")
        
        df, summary = self.load_latest_results(robot_type)
        if df is None:
            return
        
        self.create_latency_timeline(df, robot_type)
        self.create_latency_distribution(df, robot_type)
        self.create_performance_summary(summary, robot_type)
        
        print(f"\nAll graphs generated in '{self.graphs_dir}' directory")

if __name__ == "__main__":
    generator = PerformanceGraphGenerator()
    
    # Generate graphs for single robot
    generator.generate_all_graphs("single")
    
    # If you have multi-robot results, uncomment:
    # generator.generate_all_graphs("multi")
    # generator.create_comparison_graph()
