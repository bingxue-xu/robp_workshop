import json
import matplotlib.pyplot as plt
import sys
import os
from datetime import datetime

def plot_umbmark(json_file, save_dir=None):
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Extract CW and CCW points
    cw_points = [(d['dx'], d['dy']) for d in data if d.get('direction') == 'cw']
    ccw_points = [(d['dx'], d['dy']) for d in data if d.get('direction') == 'ccw']

    # Extract analysis info
    analysis = next((d['analysis'] for d in data if 'analysis' in d), None)
    robot_name = analysis.get('robot_name', 'robot') if analysis else 'robot'

    # Start plotting
    plt.figure(figsize=(7, 7))

    if cw_points:
        xs, ys = zip(*cw_points)
        plt.scatter(xs, ys, c='blue', label='CW runs')
    if ccw_points:
        xs, ys = zip(*ccw_points)
        plt.scatter(xs, ys, c='red', label='CCW runs')

    # Plot cluster centers
    if analysis:
        cw_cg = analysis['cw_cluster_center']
        ccw_cg = analysis['ccw_cluster_center']
        plt.scatter(*cw_cg, c='blue', marker='x', s=100, label='CW cluster center')
        plt.scatter(*ccw_cg, c='red', marker='x', s=100, label='CCW cluster center')
        plt.title(f"UMBmark Result (E_max,syst = {analysis['E_max_syst_m']:.3f} m)")

    # Axes and grid
    plt.xlabel('ΔX [m]')
    plt.ylabel('ΔY [m]')
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)
    plt.grid(True)
    plt.axis('equal')
    plt.legend()

    # Automatically save PNG
    if save_dir is None:
        save_dir = os.path.dirname(json_file)
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    png_filename = os.path.join(save_dir, f"{robot_name}_umbmark_{timestamp}.png")
    plt.savefig(png_filename, dpi=300)
    print(f"Saved figure to {png_filename}")

    # Show interactive plot
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python umbmark_plot.py <json_file> [save_dir]")
    else:
        json_file = sys.argv[1]
        save_dir = sys.argv[2] if len(sys.argv) >= 3 else None
        plot_umbmark(json_file, save_dir)
