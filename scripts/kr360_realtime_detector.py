# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

from opcua import Client
import time
import joblib
import numpy as np
from datetime import datetime

# ==== Config ====
OPCUA_URL = "opc.tcp://127.0.0.1:4840"
NODE_ID = "ns=2;i=2"
MODEL_PATH = "/home/krutarth/Desktop/kuka_ws_2/scripts/kr360_realistic_model.pkl"

# ==== Load Model ====
model = joblib.load(MODEL_PATH)
print("Loaded AI model")

# ==== Connect OPC UA ====
client = Client(OPCUA_URL)
client.connect()
print("Connected to OPC UA server")

node = client.get_node(NODE_ID)

try:
    print("Monitoring joint values... Ctrl+C to stop")

    while True:
        raw = node.get_value()
        joint_values = [float(i) for i in raw] if not isinstance(raw, str) else [float(i) for i in raw.split(',')]
        joint_array = np.array(joint_values).reshape(1, -1)

        pred = model.predict(joint_array)[0]

        if pred == 1:
            print(f"[{datetime.now()}] ⚠️ ANOMALY DETECTED | joints: {np.round(joint_values, 2)}")
        else:
            print(f"[{datetime.now()}] NORMAL | joints: {np.round(joint_values, 2)}")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    client.disconnect()
    print("Disconnected from OPC UA.")

