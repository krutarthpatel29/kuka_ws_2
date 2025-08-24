# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

from opcua import Client
import time
import csv
import random
from datetime import datetime

client = Client("opc.tcp://127.0.0.1:4840")

# KUKA joint limits (rad)
joint_limits = {
    'joint_1': (-3.23, 3.23),
    'joint_2': (-2.27, 0.35),
    'joint_3': (-1.75, 2.51),
    'joint_4': (-6.11, 6.11),
    'joint_5': (-2.09, 2.09),
    'joint_6': (-6.11, 6.11),
}

def generate_realistic_anomaly(joint_values):
    anomaly_type = random.choice(["small_violation", "one_joint_off", "stuck", "spike"])
    
    if anomaly_type == "small_violation":
        # Slightly exceed one or more limits
        return [v + random.uniform(0.4, 0.7) if i == 1 else v for i, v in enumerate(joint_values)]
    
    elif anomaly_type == "one_joint_off":
        # One joint way out of range
        idx = random.randint(0, 5)
        joint_values[idx] = random.uniform(3.5, 5.0)
        return joint_values

    elif anomaly_type == "stuck":
        # Return same joint values every time (will be repeated in logger loop)
        return joint_values

    elif anomaly_type == "spike":
        # Inject short random spike into one joint
        idx = random.randint(0, 5)
        joint_values[idx] = random.choice([-3.0, 4.0])
        return joint_values

    return joint_values

try:
    client.connect()
    print("Connected to OPC UA Server")
    joint_node = client.get_node("ns=2;i=2")

    with open("kr360_ai_log_realistic.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "label"])

        last_valid = None
        print("Logging with realistic anomaly injection. Ctrl+C to stop.")
        
        while True:
            inject_anomaly = random.random() < 0.1  # 10% chance

            if inject_anomaly:
                # Read last good value, and inject realistic anomaly
                value = joint_node.get_value()
                joint_values = [float(i) for i in value] if not isinstance(value, str) else [float(i) for i in value.split(',')]
                joint_values = generate_realistic_anomaly(joint_values)
                label = 1
                print(f"Injected realistic anomaly: {joint_values}")
            else:
                value = joint_node.get_value()
                joint_values = [float(i) for i in value] if not isinstance(value, str) else [float(i) for i in value.split(',')]
                last_valid = joint_values
                label = 0

            timestamp = datetime.now().timestamp()
            writer.writerow([timestamp] + joint_values + [label])
            time.sleep(0.5)

except Exception as e:
    print(f"Error: {e}")
finally:
    client.disconnect()
    print("Disconnected")

