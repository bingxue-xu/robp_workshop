## Measurement of encoder only Odometry systematic errors

### UMBmark: 3×3 m bidirectional square path

| Robot | E_max_syst [m] over 12m <br> Before calibration | E_max_syst [m] over 12m <br> After calibration | Comment |
|-------|-----------------------------------------------|----------------------------------------------|---------|
| Dopey | 0.165 |  |  |
| Grumpy | 0.186 |  |  |
| Prince | 0.214 |  |  |
| Happy | 0.234 |  |  |
| Bashful | 0.257 |  |  |
| Doc | 0.264 |  |  |
| SnowWhite | 0.301 |  |  |
| Sneezy | 0.303 |  |  |
| Queen | 0.358 |  |  |
| Sleepy | 0.481 | 0.159 | wb:0.312, wr:0.04921, wll:5 |

* default parameter before calibration: wheel_base:0.311, wheel_radius:0.04921, winding_loops_left:0

### Odometry Error Plots

| Dopey (before) | Grumpy (before) | Prince (before) |
|---|---|---|
| <img src="../odometry_test/Dopey_umbmark_2025-11-23_01-18-24.png" alt="Dopey (before)" width="400"/> | <img src="../odometry_test/Grumpy_umbmark_2025-08-14_08-49-39.png" alt="Grumpy (before)" width="400"/> | <img src="../odometry_test/Prince_umbmark_2025-08-14_10-50-07.png" alt="Prince (before)" width="400"/> | 

| Happy (before) | Bashful (before) | Doc (before) |
|---|---|---|
| <img src="../odometry_test/Happy_umbmark_2025-08-17_18-22-46.png" alt="Happy (before)" width="400"/> | <img src="../odometry_test/Bashful_umbmark_2025-11-23_14-44-07.png" alt="Bashful (before)" width="400"/> | <img src="../odometry_test/Doc_umbmark_2025-08-13_10-41-15.png" alt="Doc (before)" width="400"/> | 

| SnowWhite (before) | Sneezy (before) | Queen (before) |
|---|---|---|
| <img src="../odometry_test/SnowWhite_umbmark_2025-08-12_12-50-33.png" alt="SnowWhite (before)" width="400"/> | <img src="../odometry_test/Sneezy_umbmark_2025-08-13_06-46-43.png" alt="Sneezy (before)" width="400"/> | <img src="../odometry_test/Queen_umbmark_2025-08-13_08-42-09.png" alt="Queen (before)" width="400"/> | 

