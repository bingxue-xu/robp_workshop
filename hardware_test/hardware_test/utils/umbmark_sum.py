#!/usr/bin/env python3

import os
import json
import pandas as pd
import argparse
from pathlib import Path

def extract_odometry_results(folder):
    """Extract odometry results from all umbmark JSON files in folder."""
    rows = []
    
    for file_path in Path(folder).glob("*_umbmark.json"):
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            # Extract robot name from filename or JSON
            robot_name = file_path.stem.replace('_umbmark', '')
            
            # Get the first (and usually only) robot data
            robot_data = next(iter(data.values())) if data else {}
            
            # Extract analysis data
            analysis = robot_data.get('umbmark_analysis', {})
            
            # Find corresponding PNG file with timestamp pattern
            png_pattern = f"{robot_name}_umbmark_*.png"
            png_files = list(Path(folder).glob(png_pattern))
            
            # Get E_max value for sorting
            emax = analysis.get('emax', float('inf'))  # Use infinity for missing values
            emax_formatted = f"{emax:.3f}" if emax != float('inf') else "N/A"
            
            row = {
                'Robot': robot_name,
                'E_max_syst [m]': emax_formatted,
                'emax': emax,  # For sorting only
                'png_files': png_files  # Store for later use
            }
            
            rows.append(row)
            
        except Exception as e:
            print(f"Error processing {file_path}: {e}")
            continue
    
    # Sort by E_max value (smaller is better) and remove sort column
    df = pd.DataFrame(rows).sort_values('emax', ascending=True)
    return df

def generate_markdown_with_images(df, output_path, image_folder):
    """Generate markdown table with embedded images in 2 rows × 4 columns grid layout."""

    markdown_content = f"""## Measurement of encoder only Odometry systematic errors

### UMBmark: 3×3 m bidirectional square path


| Robot | E_max_syst [m] over 12m |
|-------|------------------------|
"""
    for _, row in df.iterrows():
        markdown_content += f"| {row['Robot']} | {row['E_max_syst [m]']} |\n"

    markdown_content += "\n### Odometry Error Plots\n\n"

    # Collect robots and their plot paths
    robots_with_plots = []
    for _, row in df.iterrows():
        robot_name = row['Robot']
        png_files = row['png_files']
        if png_files:
            latest_png = sorted(png_files)[-1]
            rel_path = os.path.relpath(latest_png, os.path.dirname(output_path))
            robots_with_plots.append((robot_name, rel_path))

    # 2 rows × 4 columns grid
    num_cols = 3
    num_rows = 3
    total_cells = num_cols * num_rows

    # Pad robots_with_plots to fill the grid if needed
    robots_with_plots += [("", "")] * (total_cells - len(robots_with_plots))

    for row_idx in range(num_rows):
        start = row_idx * num_cols
        end = start + num_cols
        row_robots = robots_with_plots[start:end]

        # Robot names row
        markdown_content += "| " + " | ".join([robot for robot, _ in row_robots]) + " |\n"
        markdown_content += "|" + "---|" * num_cols + "\n"

        # Images row
        markdown_content += "| "
        for robot, rel_path in row_robots:
            if rel_path:
                markdown_content += f'<img src="{rel_path}" alt="{robot}" width="500"/> | '
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
    
    df = extract_odometry_results(folder_path)
    
    if df.empty:
        print("No umbmark JSON files found!")
        return 1
    
    # Create display dataframe without helper columns
    display_df = df[['Robot', 'E_max_syst [m]']].copy()
    
    # Print table to console
    print("\n" + "="*50)
    print("Measurement of encoder only Odometry systematic errors")
    print("UMBmark: 3×3 m bidirectional square path")
    print("="*50)
    print(display_df.to_string(index=False))

    generate_markdown_with_images(df, output_path, folder_path)
    
    json_folder_copy = folder_path / 'odometry_summary.md'
    if output_path != json_folder_copy:
        import shutil
        shutil.copy2(output_path, json_folder_copy)
        print(f"Copy also saved to: {json_folder_copy}")
    # Save CSV if requested
    if args.csv:
        csv_path = Path(args.csv)
        display_df.to_csv(csv_path, index=False)
        print(f"CSV summary saved to: {csv_path}")
    
    print(f"\nProcessed {len(df)} robots successfully!")
    return 0

if __name__ == "__main__":
    exit(main())
