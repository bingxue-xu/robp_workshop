# Detection

### Executable
`detection.py`

### Service 
_EstPose.srv_ can be found in `ida_interfaces`

#### Definition:
- **Request:** None
- **Response:** `geometry_msgs/PointStamped`

### Subscribes to
- `/camera/depth/color/points`
### Publishes to 
- `/estimated_pose` Estimated pose of object. Average of object points
- `/filtered_points` (Does not work)
- `/camera/depth/color/ds_points` (Not used)

