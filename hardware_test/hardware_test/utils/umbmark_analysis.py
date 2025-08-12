"""
UMBmark analysis and calibration following Borenstein & Feng (1996)

Author: Bingxue Xu (2025)
"""

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


def compute_emax(cg_cw, cg_ccw):
    r_cw = math.hypot(*cg_cw)
    r_ccw = math.hypot(*cg_ccw)
    emax = max(r_cw, r_ccw)
    return r_cw, r_ccw, emax

def compute_alpha_beta(cg_cw, cg_ccw, L=4.0):
    """
    Compute alpha (per-corner turn error, rad) and beta (straight-line curvature term)
    Paper forms (sign per their convention):
      alpha_x = (x_cg,CW + x_cg,CCW)/(-4L)   (4.24a, x-version)
      alpha_y = (y_cg,CW + y_cg,CCW)/(-4L)   (4.24b, y-version)
      beta_x  = (x_cg,CW - x_cg,CCW)/(-4L)   (from 4.17-4.20)
      beta_y  = (y_cg,CW - y_cg,CCW)/(-4L)
    We compute both x/y and average (as suggested in the paper’s practice).
    """
    xcw, ycw = cg_cw
    xccw, yccw = cg_ccw

    alpha_y = (ycw + yccw) / (-4.0 * L)
    beta_y  = (ycw - yccw) / (-4.0 * L)
    alpha_x = (xcw + xccw) / (-4.0 * L)
    beta_x  = (xcw - xccw) / (-4.0 * L)

    alpha = 0.5*(alpha_x + alpha_y)
    beta = 0.5*(beta_x + beta_y)

    return alpha, beta

def wheel_diameter_ratio_from_beta(beta):
    if abs(beta) < 0.001:
        return 1.0
    return (1.0+beta) / (1.0-beta)

def wheelbase_corrected(nominal_b, alpha_rad):
    return nominal_b * ((math.pi/2) / ((math.pi/2) - alpha_rad))


def plot_umbmark(json_file, save_dir=None, nominal_wheelbase=0.311/2, run_calibration=True):
    robot_name, cw_points, ccw_points, square_size = load_points(json_file)
    cg_cw = center_of_gravity(cw_points)
    cg_ccw = center_of_gravity(ccw_points)
    r_cw, r_ccw, emax = compute_emax(cg_cw, cg_ccw)

    print(f"\n === UMBmark test: {robot_name} {square_size}*{square_size} m square path ===")
    print(f"CW Center of Gravity: {cg_cw}, Radius: {r_cw:.3f} m")
    print(f"CCW Center of Gravity: {cg_ccw}, Radius: {r_ccw:.3f} m")
    print(f"Max Radius (E_max): {emax:.3f} m")
    
    analysis_data = {
        'umbmark_analysis': {
            'cg_cw': cg_cw, 'cg_ccw': cg_ccw, 
            'r_cw': r_cw, 'r_ccw': r_ccw, 'emax': emax,
            'emax_percentage': f"{(emax / (4 * square_size)) * 100:.2f}%",
            'total_path_length': 4 * square_size,
            'umbmark_pass': emax < (4 * square_size * 0.01),  # 1% standard
            'analysis_timestamp': datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        }
    }

    if run_calibration and len(cw_points) > 0 and len(ccw_points) > 0:
        print(f"\n === Running Calibration ===")

        alpha, beta = compute_alpha_beta(cg_cw, cg_ccw, square_size)
        Ed = wheel_diameter_ratio_from_beta(beta)
        wb_corr = wheelbase_corrected(nominal_wheelbase, alpha)

        print(f"Alpha (wheelbase error): {alpha:.8f} rad")
        print(f"Beta (wheel diameter difference): {beta:.8f} rad")
        print(f"Effective wheel diameter ratio: {Ed:.6f}")
        print(f"wheelbase nominal: {nominal_wheelbase:.6f} m")
        print(f"Wheelbase correction: {wb_corr:.6f}")

        analysis_data['umbmark_calibration'] = {
            'alpha_rad': alpha,
            'beta': beta,
            'Ed_ratio_DR_over_DL': Ed,
            'wheelbase_nominal': nominal_wheelbase,
            'wheelbase_corrected': wb_corr,
            'calibration_timestamp': datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        }
    elif run_calibration:
        print("Need both CW and CCW data for calibration.")


    save_analysis_to_json(json_file, robot_name, analysis_data)
    
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

    plt.title(f"{robot_name} Odometry Error (E_max = {emax:.3f}m / {4*square_size}m)", pad=30)
    plt.figtext(0.5, 0.9, f"UMBmark: {square_size}x{square_size}m bidirectional square path", ha='center', fontsize=10)
    plt.xlabel('ΔX [m]')
    plt.ylabel('ΔY [m]')
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)
    plt.grid(True)
    plt.axis('equal')
    plt.legend(loc='upper left')

    if save_dir is None:
        save_dir = os.path.dirname(json_file)

    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    png_filename = os.path.join(save_dir, f"{robot_name}_umbmark_{timestamp}.png")
    plt.savefig(png_filename, dpi=300)
    print(f"Saved figure to {png_filename}")

    plt.show()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Run UMBmark analysis & calibration")
    parser.add_argument('json_file', help='UMBmark test data JSON file')
    parser.add_argument('--save-dir', '-d', help='Directory to save plots')
    parser.add_argument('--wheelbase', '-w', type=float, default=0.311/2,
                       help='Nominal wheelbase in meters (default: 0.311/2)')
    parser.add_argument('--no-calibration', action='store_true',
                       help='Skip calibration analysis')
    
    args = parser.parse_args()
    
    plot_umbmark(args.json_file, args.save_dir, args.wheelbase, not args.no_calibration)
