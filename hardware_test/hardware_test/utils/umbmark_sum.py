#!/usr/bin/env python3

import os
import json
import pandas as pd
import argparse
from pathlib import Path

def extract_odometry_results(folder):
    """Extract odometry results from all umbmark JSON files in folder, split before/after calibration."""
    robots = {}
    for file_path in Path(folder).glob("*_umbmark.json"):
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            robot_name = file_path.stem.replace('_umbmark', '').replace('_after_calibration', '')
            is_after = 'after_calibration' in file_path.stem
            robot_data = next(iter(data.values())) if data else {}
            analysis = robot_data.get('umbmark_analysis', {})
            emax = analysis.get('emax', float('inf'))
            emax_formatted = f"{emax:.3f}" if emax != float('inf') else "N/A"
            png_pattern = f"{file_path.stem}*.png"
            png_files = list(Path(folder).glob(png_pattern))
            latest_png = sorted(png_files)[-1] if png_files else None

            # Extract comment parameters from robot_data level (not analysis level)
            # Only extract for after_calibration files
            comment = ""
            if is_after:
                wheel_base = robot_data.get('wheel_base', 'N/A')
                wheel_radius = robot_data.get('wheel_radius', 'N/A') 
                winding_loops_left = robot_data.get('winding_loops_left', 'N/A')
                comment = f"wb:{wheel_base}, wr:{wheel_radius}, wll:{winding_loops_left}"

            if robot_name not in robots:
                robots[robot_name] = {
                    'before': {'emax': None, 'emax_fmt': None, 'png': None, 'comment': None},
                    'after': {'emax': None, 'emax_fmt': None, 'png': None, 'comment': None}
                }
            key = 'after' if is_after else 'before'
            robots[robot_name][key]['emax'] = emax
            robots[robot_name][key]['emax_fmt'] = emax_formatted
            robots[robot_name][key]['png'] = latest_png
            robots[robot_name][key]['comment'] = comment

        except Exception as e:
            print(f"Error processing {file_path}: {e}")
            continue

    # Sort robots by before calibration emax, then after calibration emax
    sorted_robots = sorted(
        robots.items(),
        key=lambda item: (
            item[1]['before']['emax'] if item[1]['before']['emax'] is not None else float('inf'),
            item[1]['after']['emax'] if item[1]['after']['emax'] is not None else float('inf')
        )
    )
    return sorted_robots

def generate_markdown_with_images(sorted_robots, output_path, image_folder):
    """Generate markdown table with before/after calibration columns and comments."""

    markdown_content = f"""## Measurement of encoder only Odometry systematic errors

### UMBmark: 3×3 m bidirectional square path

| Robot | E_max_syst [m] over 12m <br> Before calibration | E_max_syst [m] over 12m <br> After calibration | Comment |
|-------|-----------------------------------------------|----------------------------------------------|---------|
"""
    for robot_name, result in sorted_robots:
        before = result['before']['emax_fmt'] if result['before']['emax_fmt'] else ""
        after = result['after']['emax_fmt'] if result['after']['emax_fmt'] else ""
        
        # Use comment from after calibration if available, otherwise from before
        comment = ""
        if result['after']['comment']:
            comment = result['after']['comment']
        elif result['before']['comment']:
            comment = result['before']['comment']
            
        markdown_content += f"| {robot_name} | {before} | {after} | {comment} |\n"

    # Add default parameters note
    markdown_content += "\n* default parameter before calibration: wheel_base:0.311, wheel_radius:0.04921, winding_loops_left:0\n"
    markdown_content += "\n### Odometry Error Plots\n\n"

    # Collect robots and their plot paths (before/after)
    robots_with_plots = []
    for robot_name, result in sorted_robots:
        for key in ['before', 'after']:
            png = result[key]['png']
            if png:
                rel_path = os.path.relpath(png, os.path.dirname(output_path))
                robots_with_plots.append((f"{robot_name} ({key})", rel_path))

    # 2 rows × 4 columns grid (adjust as needed)
    num_cols = 3
    num_rows = 3
    total_cells = num_cols * num_rows
    robots_with_plots += [("", "")] * (total_cells - len(robots_with_plots))

    for row_idx in range(num_rows):
        start = row_idx * num_cols
        end = start + num_cols
        row_robots = robots_with_plots[start:end]
        markdown_content += "| " + " | ".join([robot for robot, _ in row_robots]) + " |\n"
        markdown_content += "|" + "---|" * num_cols + "\n"
        markdown_content += "| "
        for robot, rel_path in row_robots:
            if rel_path:
                markdown_content += f'<img src="{rel_path}" alt="{robot}" width="400"/> | '
            else:
                markdown_content += " | "
        markdown_content += "\n\n"

    with open(output_path, 'w') as f:
        f.write(markdown_content)

    print(f"Markdown summary saved to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Generate odometry test summary with plots")
    parser.add_argument('folder', 
                       help='Directory containing *_umbmark.json files')
    parser.add_argument('--save-dir', '-s', 
                       help='Output file path or directory (default: odometry_summary.md in JSON folder)')
    parser.add_argument('--csv', 
                       help='Also save as CSV file')
    
    args = parser.parse_args()
    
    folder_path = Path(args.folder)
    
    # Handle file vs directory for output_path
    if args.save_dir:
        output_path = Path(args.save_dir)
        if output_path.exists() and output_path.is_dir():
            output_path = output_path / 'odometry_summary.md'
        elif not output_path.suffix:
            output_path = output_path / 'odometry_summary.md'
    else:
        output_path = folder_path / 'odometry_summary.md'
    
    print(f"Processing odometry results in: {folder_path}")
    print(f"Output will be saved to: {output_path}")
    
    sorted_robots = extract_odometry_results(folder_path)
    
    if not sorted_robots:
        print("No umbmark JSON files found!")
        return 1
    
    # Print table to console
    print("\n" + "="*50)
    print("Measurement of encoder only Odometry systematic errors")
    print("UMBmark: 3×3 m bidirectional square path")
    print("="*50)
    print("| Robot | Before calibration | After calibration | Comment |")
    print("|-------|---------------------|--------------------|---------| ")
    for robot_name, result in sorted_robots:
        before = result['before']['emax_fmt'] if result['before']['emax_fmt'] else ""
        after = result['after']['emax_fmt'] if result['after']['emax_fmt'] else ""
        
        # Use comment from after calibration if available, otherwise from before
        comment = ""
        if result['after']['comment']:
            comment = result['after']['comment']

        print(f"| {robot_name} | {before} | {after} | {comment} |")

    generate_markdown_with_images(sorted_robots, output_path, folder_path)
    
    json_folder_copy = folder_path / 'odometry_summary.md'
    if output_path != json_folder_copy:
        import shutil
        shutil.copy2(output_path, json_folder_copy)
        print(f"Copy also saved to: {json_folder_copy}")
    # Save CSV if requested
    if args.csv:
        csv_path = Path(args.csv)
        # Save as CSV using the sorted_robots list
        import csv
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Robot', 'Before calibration', 'After calibration', 'Comment'])
            for robot_name, result in sorted_robots:
                before = result['before']['emax_fmt'] if result['before']['emax_fmt'] else ""
                after = result['after']['emax_fmt'] if result['after']['emax_fmt'] else ""
                
                # Use comment from after calibration if available, otherwise from before
                comment = ""
                if result['after']['comment']:
                    comment = result['after']['comment']
                elif result['before']['comment']:
                    comment = result['before']['comment']
                    
                writer.writerow([robot_name, before, after, comment])
        print(f"CSV summary saved to: {csv_path}")
    
    print(f"\nProcessed {len(sorted_robots)} robots successfully!")
    return 0


if __name__ == "__main__":
    exit(main())
