#!/bin/bash

# Start the algorithm container
echo "Starting F1TENTH algorithm container..."
docker-compose up -d

# Connect to the container
echo "Connecting to the algorithm container..."
docker exec -it f1tenth_algorithms bash -c "cd /root/algorithm_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch f1tenth_cdt cdt_planner.launch.py"