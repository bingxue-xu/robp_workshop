# hardware_ws/src/hardware_test/hardware_test/utils/json_helper.py

import json
import os

def load_all_robot_json(json_folder):
    """
    read all robots' hardware test results .json files, return table
    """
    results = []
    for fname in os.listdir(json_folder):
        if not fname.endswith('.json'):
            continue
        path = os.path.join(json_folder, fname)
        with open(path, 'r') as f:
            data = json.load(f)
            results.append(data)
    return results