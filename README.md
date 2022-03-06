# Dijkstra-Search
Use of Dijkstra algorithm to search for optimal path between a given initial and final position in a custom-generated map.

## To run the code, execute `python3 Dijkstra-pathplanning-Nitesh-Jha.py --start_location_x 50 --start_location_y 150 --goal_location_x 200 --goal_location_y 300` where (50,150) will be start_location and (200,300) will be the goal_location. Enter these values in the command as desired.
The current search is set by default to start from (50,150) and go to (200,300)

### Implementation details:
#### Map space(Obstacle space):
Note that the map array is of size 252 x 402, where the extra boundaries on all sides(1 pixel) are colored black to resemble obstacles and contain the point robot within the map. Hence, the correspondence of points of this map with respect to the original map of 250 x 400 is as follows:
start_modified_x = start_original_x + 1
start_modified_y = start_original_y + 1

goal_modified_x = goal_original_x + 1
goal_modified_y = goal_original_y + 1
Meaning, if you desire to start from (10,10) in the 250 x 400 space, enter start_location as (11,11).

If the start or goal locations are set outside the bounds of the map, the program ends with the message "Obstacle/boundaries near start or end locations OR not in map_range". Modify start and/or end locations if this is encountered.

#### Clearance
The code keeps a clearance of 5 pixels(default, modify as required). Thus, if the start_location or end_location have any black pixel (representing obstacle) in vicinity of 5 pixels, the program will end and display the selected start and goal points, along with the message "Obstacle/boundaries near start or end locations". 
Modify the start and goal locations if this is encountered.

#### Visualization
To speed up visualization of the search, a frame skip of 50 is set by default.

![viz](https://github.com/niteshjha08/Dijkstra-Search/blob/main/media/viz.gif)
