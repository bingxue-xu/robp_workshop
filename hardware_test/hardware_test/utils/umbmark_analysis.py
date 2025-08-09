import json
import matplotlib.pyplot as plt
import sys
import os
from datetime import datetime
import math

def load_points(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("JSON file must contain a dictionary of points.")

    cw, ccw = [], []
    robot_name = list(data.keys())[0] 
    robot_data = data[robot_name]
    square_size = robot_data.get('square_size', 4.0)

    if 'ccw' in robot_data:
        for lap_num, lap_data in robot_data['ccw'].items():
            detail = lap_data.get('detailed_results', {})
            if 'dx' in detail and 'dy' in detail:
                point_data = {
                    'dx': float(detail['dx']),
                    'dy': float(detail['dy']),
                    'closure_error': float(detail.get('closure_error', 0.0)),
                    'dtheta_deg': float(detail['dtheta_deg'] if detail.get('dtheta_deg') is not None else 0.0),
                    'lap': int(lap_num)
                }
                ccw.append(point_data)
    
    if 'cw' in robot_data:
        for lap_num, lap_data in robot_data['cw'].items():
            detail = lap_data.get('detailed_results', {})
            if 'dx' in detail and 'dy' in detail:
                point_data = {
                    'dx': float(detail['dx']),
                    'dy': float(detail['dy']),
                    'closure_error': float(detail.get('closure_error', 0.0)),
                    'dtheta_deg': float(detail['dtheta_deg'] if detail.get('dtheta_deg') is not None else 0.0),
                    'lap': int(lap_num)
                }
                cw.append(point_data)
    
    return robot_name, cw, ccw, square_size


def center_of_gravity(points):
    if not points:
        return (0.0, 0.0)
    xs = [p['dx'] for p in points]
    ys = [p['dy'] for p in points]
    return (sum(xs) / len(xs), sum(ys) / len(ys))

def save_analysis_to_json(json_file, robot_name, analysis_data):
    """save an analysis result back to JSON file"""
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)
        
        if robot_name in data:
            data[robot_name].update(analysis_data)
        
        with open(json_file, 'w') as f:
            json.dump(data, f, indent=2)
            
    except Exception as e:
        print(f"Failed to save analysis results: {e}")

def analysis_system_error(cg_cw, cg_ccw, square_size=4.0):
    systematic_x = (cg_cw[0] + cg_ccw[0]) / 2.0
    systematic_y = (cg_cw[1] + cg_ccw[1]) / 2.0
    non_systematic_x = (cg_cw[0] - cg_ccw[0]) / 2.0
    non_systematic_y = (cg_cw[1] - cg_ccw[1]) / 2.0
    total_path = 4 * square_size

    wheel_radius_error = (systematic_x + systematic_y) / total_path * 100  # in percentage
    encoder_mismatch = (abs(non_systematic_x) + abs(non_systematic_y)) / total_path * 100  # in percentage

    print("\n Systematic Error Analysis:")
    print(f"Systematic X Error: dx={systematic_x:.2f} m, dy={systematic_y:.2f} m")
    print(f"Non-Systematic X Error: dx={non_systematic_x:.2f} m, dy={non_systematic_y:.2f} m")
    print(f"Wheel Radius Error: {wheel_radius_error:.2f}%")
    print(f"Encoder Mismatch: {encoder_mismatch:.2f}%")
    return {
        'systematic': (systematic_x, systematic_y),
        'non_systematic': (non_systematic_x, non_systematic_y),
        'wheel_radius_error': wheel_radius_error,
        'encoder_mismatch': encoder_mismatch
    }

def plot_umbmark(json_file, save_dir=None):
    robot_name, cw_points, ccw_points, square_size = load_points(json_file)

    cg_cw = center_of_gravity(cw_points)
    cg_ccw = center_of_gravity(ccw_points)
    r_cw = math.hypot(*cg_cw)
    r_ccw = math.hypot(*cg_ccw)
    emax = max(r_cw, r_ccw)

    error_analysis = analysis_system_error(cg_cw, cg_ccw, square_size=square_size)

    print(f"\n === UMBmark test: {robot_name} {square_size} m square path ===")
    print(f"CW Center of Gravity: {cg_cw}, Radius: {r_cw:.3f} m")
    print(f"CCW Center of Gravity: {cg_ccw}, Radius: {r_ccw:.3f} m")
    print(f"Max Radius (E_max): {emax:.3f} m")
    
    save_analysis_to_json(json_file, robot_name, {
        'umbmark_analysis': {
            'cg_cw': cg_cw,
            'cg_ccw': cg_ccw, 
            'r_cw': r_cw,
            'r_ccw': r_ccw,
            'emax': emax,
            'emax_percentage': (emax / (4 * square_size)) * 100,
            'total_path_length': 4 * square_size,
            'umbmark_pass': emax < (4 * square_size * 0.01),  # 1% 标准
            'analysis_timestamp': datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            **error_analysis  # 包含所有系统误差分析结果
        }
    })

    plt.figure(figsize=(7, 7))
    if cw_points:
        xs = [p['dx'] for p in cw_points]
        ys = [p['dy'] for p in cw_points]
        plt.scatter(xs, ys, c='blue', label='CW runs')
    if ccw_points:
        xs = [p['dx'] for p in ccw_points]
        ys = [p['dy'] for p in ccw_points]
        plt.scatter(xs, ys, c='red', label='CCW runs')

    plt.scatter(*cg_cw, c='blue', marker='x', s=100, label='CW CoG')
    plt.scatter(*cg_ccw, c='red', marker='x', s=100, label='CCW CoG')
    
    plt.title(f"{robot_name} UMBmark (E_max = {emax:.3f} m)")
    plt.xlabel('ΔX [m]')
    plt.ylabel('ΔY [m]')
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)
    plt.grid(True)
    plt.axis('equal')
    plt.legend()

    if save_dir is None:
        save_dir = os.path.dirname(json_file)
        save_dir = os.path.join(save_dir, "odometry_test")

    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    png_filename = os.path.join(save_dir, f"{robot_name}_umbmark_{timestamp}.png")
    plt.savefig(png_filename, dpi=300)
    print(f"Saved figure to {png_filename}")

    # Show interactive plot
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 umbmark_plot.py <json_file> [save_dir]")
    else:
        json_file = sys.argv[1]
        save_dir = sys.argv[2] if len(sys.argv) >= 3 else None
        plot_umbmark(json_file, save_dir)
