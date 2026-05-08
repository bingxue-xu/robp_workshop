
## Pacage name <path_planning>
### Executable: global_planning       
Does global path planning with A* algorithm on grid map, explore the next node within nearest 8 grids. Takes in occupency grid from global map, goal position from request and robot position from tf, publish the global path every 0.02m and response it to the client.
    
All these are done in map frame and lastest time stamp.
Takes in 1D occupency grid from global map
### Service: GlobalPlanning /global_planning

    - request: PointStamped goal
    - response: Path /global_path

### Subscrib, Publish and Parameters

    * subscription: OccupancyGrid /map
        - convert from 1D o a 2D grid for A* usage

    * inflated_publisher: OccupancyGrid /map_inflated
        - inflated radius: 15cm

    * path_publisher: Path /global_path 
        - a list of waypoints with x, y value

    * parameters: 
        - offset = -10
        - resolution = 0.02
        - inflated radius = 0.15

