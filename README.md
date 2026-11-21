# F1TENTH Algorithms

ROS 2 (Python) driving algorithms for an F1TENTH autonomous racing car, developed against the F1TENTH Gym simulator and LiDAR hardware. Everything runs in Docker.

## Packages (`f1tenth_algorithms/src`)

| Package | Description |
|---|---|
| `disparity_extender` | Reactive LiDAR driving using the disparity-extender method |
| `f1tenth_cdt` | Planner built on constrained Delaunay triangulation of the LiDAR free space, with path planning and driving nodes |
| `f1tenth_pursuer` | Additional controller package |
| `f1tenth_racer` | Additional racing controller package |

## Run

```bash
cd f1tenth_algorithms
docker compose up --build
# inside the container
colcon build --symlink-install && source install/setup.bash
```

The compose file expects an external Docker network named `f1tenth_network` (`docker network create f1tenth_network`) shared with the simulator.

## Stack

ROS 2, Python, NumPy, Shapely, Ackermann drive messages, RViz visualisation, Docker.
