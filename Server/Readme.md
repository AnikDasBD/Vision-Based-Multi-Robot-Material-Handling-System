###Add Static IPs of the robots in the config.py file
The Server and Robots must be under same network. 

###Config file preserves key informations such as no of robots, obj, zones and their assigned marker IDs, robot size, offset etc.
Code is modular.
All of the 8 server file should be in the same folder 
# Server Files

* `main.py` — Main program that coordinates the overall server-side operation.
* `collision_checker.py` — Performs geometric inter-robot collision detection and avoidance checks.
* `config.py` — Stores configuration parameters such as marker IDs, robot dimensions, offsets, and other constants.
* `controller.py` — Handles robot motion control, including P-control, motor speed/PWM control, stopping conditions, dead zones, and object approach/drop actions.
* `detector.py` — Detects and processes ArUco markers for vision-based localization.
* `planner.py` — Generates A* paths for robot navigation.
* `robot_comms.py` — Handles communication between the server and robots and sends movement and task commands.
* `task_allocator.py` — Assigns objects to robots using the task allocation algorithm.

###OpenCV library used here is not the regular library. It is the User Contribution version. 


