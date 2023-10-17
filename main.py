from robot_simulator import *  # Starts simulator

# Linear (forward) speed of robot
v = 1

# Angular (turning) speed of robot
w = 0

# Updates the simulation (moves the robot)
update_simulation(v, w)

save_simulation_data()  # Outputs simulation as video and text file