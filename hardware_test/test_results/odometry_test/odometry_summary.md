## Measurement of encoder only Odometry systematic errors

### UMBmark: 3×3 m bidirectional square path

| Robot | E_max_syst [m] over 12m <br> Before calibration | E_max_syst [m] over 12m <br> After calibration | Comment |
|-------|-----------------------------------------------|----------------------------------------------|---------|
| Grumpy | 0.186 |  |  |
| Prince | 0.214 |  |  |
| Happy | 0.234 |  |  |
| Doc | 0.264 |  |  |
| SnowWhite | 0.301 |  |  |
| Sneezy | 0.303 |  |  |
| Queen | 0.358 |  |  |
| Sleepy | 0.481 | 0.159 | wb:0.312, wr:0.04921, wll:5 |

* default parameter before calibration: wheel_base:0.311, wheel_radius:0.04921, winding_loops_left:0

### Odometry Error Plots

| Grumpy (before) | Prince (before) | Happy (before) |
|---|---|---|
| <img src="../odometry_test/Grumpy_umbmark_2025-08-14_08-49-39.png" alt="Grumpy (before)" width="400"/> | <img src="../odometry_test/Prince_umbmark_2025-08-14_10-50-07.png" alt="Prince (before)" width="400"/> | <img src="../odometry_test/Happy_umbmark_2025-08-17_18-22-46.png" alt="Happy (before)" width="400"/> | 

| Doc (before) | SnowWhite (before) | Sneezy (before) |
|---|---|---|
| <img src="../odometry_test/Doc_umbmark_2025-08-13_10-41-15.png" alt="Doc (before)" width="400"/> | <img src="../odometry_test/SnowWhite_umbmark_2025-08-12_12-50-33.png" alt="SnowWhite (before)" width="400"/> | <img src="../odometry_test/Sneezy_umbmark_2025-08-13_06-46-43.png" alt="Sneezy (before)" width="400"/> | 

| Queen (before) | Sleepy (before) | Sleepy (after) |
|---|---|---|
| <img src="../odometry_test/Queen_umbmark_2025-08-13_08-42-09.png" alt="Queen (before)" width="400"/> | <img src="../odometry_test/Sleepy_umbmark_2025-08-13_04-07-41.png" alt="Sleepy (before)" width="400"/> | <img src="../odometry_test/Sleepy_after_calibration_umbmark_2025-08-23_17-59-26.png" alt="Sleepy (after)" width="400"/> | 

