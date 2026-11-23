# Map obstacles

## Description

Obstacles of the offline map: convex polytopes given by half-planes, computed offline from a point cloud map and
loaded from a CSV file. The package holds the map files (`maps/`) and a plain C++ library without ROS
(`map_obstacles_core`) with the obstacle type, the CSV loader and a check that every obstacle is a closed shape.

It is shared by the packages that need the map, so that none of them depends on another:

- `cbf_safety_filter`: collision avoidance with the map obstacles (parameter `path_map`)
- `clustering`: removes the LiDAR points of known obstacles (parameter `map.path`)

## Map files

Place maps in the `maps/` folder to load offline computed obstacles. It is installed to `share/map_obstacles/maps`,
and both nodes resolve a relative map path against `share/map_obstacles`, e.g.
`maps/2026-10-20_13-24_Office2_obstacles.csv` (an absolute path is used as it is). After adding or changing a map,
rebuild the package (`colcon build --packages-select map_obstacles`) so the installed copy is updated.

| file | content |
|---|---|
| `2026-10-20_13-24_Office2_obstacles.csv` | Office2 map (default of both nodes), 256 obstacles (one, ID 141, is not closed and dropped) |
| `test_two_boxes.csv` | two boxes, for tests |

The maps are generated from a `.pcd` map with `pcd_to_obstacles.py` (in `final/` of the map_clustering repository,
outside of this workspace):

```bash
cd ~/test/map_clustering/final
.venv/bin/python pcd_to_obstacles.py <map>.pcd --output-dir ~/test/map_clustering/ros2_ws/src/map_obstacles/maps --show
```

It writes `<map>_obstacles.csv` (used here) and `<map>_obstacles.yaml` (the same obstacles with their kind and number
of points, for inspection).

To look at a map, and optionally at the lidar in it (pose, range, scan, the obstacles it sees and their active faces
as the CBF filter chooses them), use `show_obstacles.py` in the same folder:

```bash
.venv/bin/python show_obstacles.py                                        # the default map of the nodes
.venv/bin/python show_obstacles.py <map>_obstacles.csv --hide-ceiling --lidar 1.25 11.69 1.28 --range 2
```

## CSV format

The maps are saved as CSV files with the header

```
nx,ny,nz,offset,obstacle
```

and one half-plane per row:

- `nx,ny,nz`: normal of the plane, pointing **out of the obstacle towards the free space** (map frame). It is scaled to
  unit length when loading (and the offset with it).
- `offset`: $b$ in the plane equation $n^\top p + b = 0$, i.e. the signed distance from the origin of the map frame to
  the plane ($b > 0$ if the origin is on the free side). The free side of the plane is $n^\top p + b \ge 0$.
- `obstacle`: integer ID of the obstacle the plane belongs to. All rows with the same ID form one convex obstacle; they
  do not need to be next to each other.

A point $p$ is inside an obstacle iff $n_i^\top p + b_i < 0$ for **all** its planes $i$, i.e. outside as soon as it is
on the free side of one of them. Empty lines and lines starting with `#` are skipped.

Example: a box of 1 × 1 × 1 m centred at (2, 0, 0.5):

```
nx,ny,nz,offset,obstacle
# x from 1.5 to 2.5, y from -0.5 to 0.5, z from 0 to 1
1.0,0.0,0.0,-2.5,0
-1.0,0.0,0.0,1.5,0
0.0,1.0,0.0,-0.5,0
0.0,-1.0,0.0,-0.5,0
0.0,0.0,1.0,-1.0,0
0.0,0.0,-1.0,0.0,0
```

## Closed obstacles

When loading, every obstacle is checked to be a closed (bounded) shape, and obstacles that are not are dropped with a
warning listing their IDs. An obstacle is closed if it has

- a top plane: normal within `horizontal_tol_deg` (default 5°) of $+z$,
- a bottom plane: normal within `horizontal_tol_deg` of $-z$,
- side planes (all other planes, sloped ones included) whose normals, seen from above, surround the obstacle: no gap of
  180° − `horizontal_tol_deg` or more between neighbouring directions. This needs at least 3 non-parallel sides, but
  3 are not always enough (sides facing 0°, 30° and 60° leave the obstacle open towards 210°).

Only the directions of the normals are checked, not whether the planes enclose a non-empty region. A single half-plane
(e.g. a virtual wall) is not closed and is dropped.

## Usage

```cmake
# CMakeLists.txt
find_package(map_obstacles REQUIRED)
target_link_libraries(<target> map_obstacles::map_obstacles_core)
```

```xml
<!-- package.xml -->
<depend>map_obstacles</depend>
```

```cpp
#include "map_obstacles/map_obstacles.hpp"

// drops obstacles that are not closed (warning on std::cerr)
map_obstacles::MapObstacles map = map_obstacles::loadMapObstacles(path);
for (const map_obstacles::MapObstacle& obs : map.map_obstacle) {
    // obs.ID, obs.planes (pcd_constraints::HalfPlane: normal, offset)
}
```

| function | description |
|---|---|
| `loadMapObstacles(path, horizontal_tol_deg = 5)` | read the CSV, normalise the planes, drop obstacles that are not closed; throws `std::runtime_error` on a missing file, a wrong header or an invalid row |
| `isClosedObstacle(obstacle, horizontal_tol_deg = 5)` | closed-shape check (see above) |
| `obstacleFeasibilityCheck(map, horizontal_tol_deg = 5)` | remove the obstacles that are not closed, returns their IDs |

The `HalfPlane` type is `pcd_constraints::HalfPlane` (`pcd_constraints/constraints.hpp`), the one half-plane type of the
workspace.
