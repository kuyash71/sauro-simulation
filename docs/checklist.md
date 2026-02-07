# SIMULATION PROGRESSION CHECKLIST

### HOW LIST WORKS?

- Build: The result of the action will be on final product
- Polish: The goal of the action is fix or improve the results of "Build"s
- Test: The result of the action is not about final product, Action is just about testing the functions/sim
- Optional: The result of the action is not necessary for the project

## PHASE 1: Preperation

- [x] Test: Get Camera Data via ROS2 node
- [x] Test: Fly drone via MAV Terminal on Gazebo
- [x] Test: Basic OpenCV Terminal Functions
- [x] Build: Create Models and World of the models in ~/environments/models file
- [x] Test: Publish fake mission states (IDLE, SEARCH, FAILSAFE) over ROS2
- [x] Test: Publish fake telemetry data (pose, altitude, mode)
- [x] Test: Start testing the object detections for the models created on previous task.

# PHASE 2: Drawing the "infinity" - First Mission of TEKNOFEST

- [ ] Build: Scenario of the first mission of TEKNOFEST (scenarios/normal_mission/first)
- [ ] Build: Draw the "infinity" with the drone by using pre-created location creator system
- [ ] Test: Create stable and healty way to detect the White Cylinder in order to follow it by OpenCV
- [ ] Build: Draw the "infinity" with the object-detection algorithm
- [ ] Test: Visualize infinity progress as % or loop count

# PHASE 3: Dropping the Object - Second Mission of TEKNOFEST

- [ ] Build: Scenario of the second mission of TEKNOFEST (scenarios/normal_mission/second)
- [ ] Test: Find a stable and healty way to detect the landing zones
- [ ] Build: Locate the target positions to drop the cargo
- [ ] Build: Add second camera to bottom of the drone
- [ ] Test: Create a camera switcher script and test to make simulation more realistic
- [ ] Build: Create an algorithm for positioning to drone with using the data of bottom camera
- [ ] Test: Publish alignment error (dx, dy, confidence) for GCS
- [ ] Build: Complete the Second Mission on Simulation

# PHASE 4: Taking and Dropping the Object - Third Mission of TEKNOFEST

- [ ] Build: Scenario of the third mission of TEKNOFEST (scenarios/normal_mission/third)
- [ ] Build: Create an algorithm to take the cargo (wont use gripper for this stage. Not in MVP product)
- [ ] Test: Manual override from GCS during pickup/drop

# PHASE 5: Handling the Possible Errors in thr Process

- [ ] Test: Analyze possible errors in Process by simulating the sim on different conditions
- [ ] Build: Create scenario for Communication loss on flight (scenarios/comm_loss)
- [ ] Build: Create scenario for GPS loss on flight (scenarios/gps_loss)
- [ ] Build: Create scenario for Weather Degration on flight (scenarios/weather_degraded)
- [ ] Test: Verify fail-safe reason is reported to GCS

# PHASE 6: Polishing

- [ ] Polish: Test and fix all of the error handling systems with real simulation
- [ ] Build: Optional: add Gripper for more realistic simulation
- [ ] Polish: Increase the stability and speed of normal missions
- [ ] Polish: Increase efficency of the algorithms

# PHASE 7: Extra - Changing the GCS from Ardupilot to sauro-terminal(pre-alpha of ULAK GCS)

### ALL OF THIS PHASE IS COMPLETELY EXTRA, THESE WILL NOT EFFECT THE SIMULATION'S PROGRESSION

- [ ] Test: Replace Mission Planner telemetry with ROS2-fed telemetry
- [ ] Test: Test ULAK GCS for the simulation
- [ ] Polish: Adjust the GCS features by comparing it with Ardupilot Mission Planner
- [ ] Polish: Increase the stability of telemetry and ROS2 nodes
- [ ] Build: Finish the Transition of GCSs
