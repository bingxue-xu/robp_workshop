import os
import json
import pandas as pd

def extract_servo_pass(servo_results):
    servo_pass = {}
    for i in range(1, 7):
        min_key = f"servo_{i}_min"
        max_key = f"servo_{i}_max"
        min_moved = servo_results.get(min_key, {}).get("moved", False)
        max_moved = servo_results.get(max_key, {}).get("moved", False)
        servo_pass[f"servo_{i}"] = "PASS" if (min_moved and max_moved) else "FAIL"
    return servo_pass

def truncate(text, maxlen=50):
    if not isinstance(text, str):
        text = str(text)
    return text if len(text) <= maxlen else text[:maxlen-3] + "..."

def gather_json_to_table(folder):
    rows = []
    for fname in os.listdir(folder):
        if not fname.endswith(".json"):
            continue
        with open(os.path.join(folder, fname), "r") as f:
            data = json.load(f)
        last_updated = data.get("last_updated")
        if isinstance(last_updated, str) and "." in last_updated:
            last_updated = last_updated.split(".")[0]
        row = {
            "robot_name": data.get("robot_name"),
            "domain_id": data.get("domain_id"),
            "last_updated": last_updated,
        }
        results = data.get("results", {})

        # Only show PASS/FAIL for each hardware
        row["nuc"] = results.get("Nuc", {}).get("status", "")
        row["usb_cam"] = results.get("USBCam", {}).get("status", "")
        row["phidgets"] = results.get("Phidgets", {}).get("status", "")
        row["rplidar"] = results.get("RPLidar", {}).get("status", "")
        row["realsense"] = results.get("RealSense", {}).get("status", "")
        row["arm"] = results.get("Arm", {}).get("status", "")

        # Servo pass/fail
        arm = results.get("Arm", {})
        arm_detail = arm.get("detail", {})
        if isinstance(arm_detail, dict):
            servo_results = arm_detail.get("servo_results", {})
            servo_pass = extract_servo_pass(servo_results)
            row.update(servo_pass)
        else:
            for i in range(1, 7):
                row[f"servo_{i}_pass"] = ""

        # Add rplidar_detail (always show)
        row["rplidar_detail"] = results.get("RPLidar", {}).get("detail", "")

        # Add fail_detail (truncated for overview)
        row["fail_detail"] = ""
        for key in ["nuc", "usb_cam", "phidgets", "rplidar", "realsense", "arm"]:
            if row.get(key, "") == "FAIL":
                detail = results.get(key.capitalize(), {}).get("detail", "")
                row["fail_detail"] += f"{key}: {truncate(detail)}; "

        rows.append(row)
    df = pd.DataFrame(rows)
    return df

if __name__ == "__main__":
    folder = os.path.join(os.path.dirname(__file__), "..", "..", "test_results")
    folder = os.path.abspath(folder)
    df = gather_json_to_table(folder)
    print(df.to_markdown(index=False))
    df.to_csv(os.path.join(folder, "summary.csv"), index=False)
    df.to_excel(os.path.join(folder, "summary.xlsx"), index=False)
    df.to_markdown(os.path.join(folder, "summary.md"), index=False)