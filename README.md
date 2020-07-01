## Code Comparison Between `backyard_flyer_solution.py` (Previous) and `motion_planning.py` (Current)
Previous: the drone moved from the ARMING state to the TAKEOFF state.<br>
Current: there is a new state called PLANNING between ARMING and TAKEOFF.<br>
<p>
Previous: the path was determined in the TAKEOFF state by the calculate_box() function.<br>
Current: the path is determined in the PLANNING state by the plan_path() function.<br>
<p>
Previous: the square path was hardcoded.<br>
Current: the A* algorithm is used to plan a path between a start and end point. This algorithm is implemented by the a_star() function.<br>

## Implementing Your Path Planning Algorithm
### Use lat0 and lon0 from the CSV as global home
Read longitude and latitude from the first line of `colliders.csv`

```python
with open('colliders.csv') as f:
    first_line = f.readline()

lat_long = first_line.split(",")
lat0 = float(lat_long[0][5:])
lon0 = float(lat_long[1][5:])
```

Set those coordinates as the start global position

```python
self.set_home_position(lon0, lat0, 0)
```

### Determine local position relative to global home
Convert current global position to local position

```python
geodetic_current_coordinates = [self._longitude, self._latitude, self._altitude]

local_position = global_to_local(geodetic_current_coordinates, self.global_home)
```

### Use current local position as `grid_start`
Use the current local position as the grid start

```python
grid_start = (local_position[0] - north_offset, local_position[1] - east_offset)
```

### Use arbitrary position as goal
Set any arbitrary position as the global goal by specifying its longitude and latitude. These values are used to determine the local goal position in the grid.

```python
global_goal = [-122.397276, 37.795191, 0.0]
local_goal = global_to_local(global_goal, self.global_home)
grid_goal = (int(local_goal[0] - north_offset),
             int(local_goal[1] - east_offset))
```

### Improve A* implementation
Add new directions to allow the quadcopter to fly in diagonals: northwest, northeast, southwest, southeast.<p>
Inside the class Action(Enum) we add the new directions (including delta position relative to current grid position and the cost)

```python
NORTH_WEST = (-1, -1, sqrt(2.0))
NORTH_EAST = (-1, 1, sqrt(2.0))
SOUTH_WEST = (1, -1, sqrt(2.0))
SOUTH_EAST = (1, 1, sqrt(2.0))
```

Inside the function valid_actions(grid, current_node) we also check when any of the new directions have to be removed from the list of allowed ones (given a position in the grid).

```python
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```

### Prune unnecessary waypoints from the path
We use the collinearity check algorithm to remove unnecessary waypoints from the path. The function takes all the path points in groups of three and check if they are collinear or not. If a group of three point is collinear, the middle point is removed.

```python
def prune_path(path):

    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(p1, p2, p3, epsilon=1e-6):
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])

        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path
```
