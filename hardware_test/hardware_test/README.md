### Running the Odometry test 

Example 
```bash
# Terminal 1: Start odometry system
ros2 launch hardware_test odometry_test_launch.launch.py robot_name:=L domain_id:=0

# Terminal 2: Run test
ros2 run hardware_test odometry_test --ros-args -p robot_name:=L -p square_size:=1.0 -p square_size:=4.0 -p speed:=0.2 -p laps_per_direction:=5

# Follow prompts:
Press Enter when ready to start lap 1 (cw)...
# Robot moves in square automatically
Enter measured x_abs [m]: 0.05
Enter measured y_abs [m]: -0.12
Enter measured yaw [deg] (Enter to skip): 23

Reposition robot to origin and press Enter to start next run...
# Repeat for all laps

# Plot results
python3 src/hardware_test/hardware_test/utils/umbmark_plot.py \
    src/hardware_test/test_results/odometry_test/L_umbmark_2025-08-06_17-55-58.json
```

