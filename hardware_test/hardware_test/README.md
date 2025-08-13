### Running the Odometry test 

Example 
```bash
# Terminal 1: Start odometry system
ros2 launch hardware_test odometry_test_launch.launch.py robot_name:=Sneezy domain_id:=0

# Terminal 2: Run test
ros2 run hardware_test odometry_test --ros-args -p robot_name:=Sneezy -p domain_id:=0 -p direction:=ccw -p square_size:=3.0 -p lap:=4 -p angular_speed:=1.2

# After each run, input measurement:
Enter measured x_abs [m]: 1.75
Enter measured y_abs [m]: -0.785
Enter measured theta_abs [deg] (optional, Enter to skip): 

# results saved
[INFO] [1755056587.144777617] [umbmark_odometry_test]: Saved lap 4 ccw → /home/bingxue/dd2419/workshop_ws/src/hardware_test/test_results/odometry_test/Sneezy_umbmark.json

# Repeat for all 5 laps counter-clockwise(ccw) and clockwise(cw)

# Analysis and plot results
cd /home/bingxue/dd2419/workshop_ws/src

python3 hardware_test/hardware_test/utils/umbmark_analysis.py hardware_test/test_results/odometry_test/Sneezy_umbmark.json  --save-dir hardware_test/test_results/odometry_test/
```

