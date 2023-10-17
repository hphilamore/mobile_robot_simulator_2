from math import *
import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2
from os import listdir, remove
from os.path import isfile, join

class Obstacle_c:

  def __init__(self, x, y, radius=10):

    self.radius = radius
    self.x = x #(arena_size/2) + rand_dist*np.cos(rot_ang)
    self.y = y # (arena_size/2) + rand_dist*np.sin(rot_ang)


# The model of the proximity sensor.
class ProxSensor_c:

  # current global xy position of sensor
  x = 0
  y = 0
  theta = 0

  # To store the last sensor reading
  reading = 0

  # To set sensor local position around the robot body.
  offset_dist = 0
  offset_angl = 0

  # maximum scan range:
  max_range = 10

  # angular range
  angular_range = np.pi / 2

  # constructor. by default, sensor sits 10 distance forward
  # and faces 0 radians with respect to robot origin (0,0).
  def __init__(self, offset_dist=5, offset_angl=0):
    self.offset_dist = offset_dist
    self.offset_angl = offset_angl


  def updateGlobalPosition(self, robot_x, robot_y, robot_theta ):

    # Current direction of the sensor is the rotation
    # of the robot body plus the fixed rotation of the
    # sensor around that same body.
    self.theta = self.offset_angl + robot_theta

    # With the rotation, we now work out where the
    # sensor sits in cartesian space (x,y) by projecting
    # out by offset_distance.
    # Note, we do this as if the sensor was at origin (0,0)
    sensor_x = (self.offset_dist*np.cos(self.theta))
    sensor_y = (self.offset_dist*np.sin(self.theta))

    # commit new position to memory, translating to the
    # robots current x,y position.
    self.x = sensor_x + robot_x
    self.y = sensor_y + robot_y

    # If we've reset position, the last sensor reading
    # is now wrong.
    self.reading = -1

  def scanFor( self, obstruction, stall ):

    stall_condition = 0

    # See if the obstruction is within the detection
    # range of the sensor.
    distance = np.sqrt( ((obstruction.x - self.x)**2) + ((obstruction.y - self.y)**2) )
    distance = distance - obstruction.radius

    # compute this sensors angle toward the obstruction
    # (e.g. where is the object relative to the sensor?)
    a2o = atan2( ( obstruction.y - self.y), (obstruction.x-self.x ))

    # compute the smallest angle between the line of
    # sight of the sensor, and the current angle to the
    # obstruction.
    # [insert url here]
    angle_between = atan2( sin(self.theta-a2o),  cos(self.theta-a2o) )
    angle_between = abs( angle_between )

    # If the robot has stalled, detect if the object is in front of
    # or behind the robot:
    if stall==1:
      if self.offset_angl == 0: # If the sensor is at the front
        if angle_between < pi/2:
          stall_condition = 1   # Obstruction in front
        else:
          stall_condition = -1  # Obstruction behind

    # If the detection is outside of the field of view
    # of the sensor, then we return and do nothing else.
    # This will either leave the reading as -1 (invalid)
    # for the next call.  Or leave the reading as the
    # last valid reading (>-1) it had.
    if angle_between < self.angular_range:
      # return

      # Detected object within range
      # note: real sensors aren't this easy
      if distance < self.max_range:
        # return

        # If the current reading is -1 then that means
        # this is the first valid reading, and we update
        # the sensor.
        if self.reading < 0:
          self.reading = distance

        # If the sensor already has a valid reading (>-1)
        # from another obstacle then we only store the new
        # reading if it is closer.
        # (closer obstructions block the field of view)
        if self.reading > 0:
          if distance < self.reading:
            self.reading = distance
    return stall_condition
    # self.reading = distance

# The model of the robot.
class Robot_c:

  # We could do something like, manually add 2 sensors
  #prox_sensors.append( ProxSensor_c(2, np.pi/8) )
  #prox_sensors.append( ProxSensor_c(2, -np.pi/8) )

  def __init__(self, x=50,y=50,theta=np.pi):
    self.x = x
    self.y = y
    self.theta = theta
    self.stall = -1 # to check for collisions
    self.stall_condition = 0 # to check for direction of collision
    self.score = 0
    self.radius = 5 # 5cm radius
    self.wheel_sep = self.radius*2 # wheel on either side
    self.v = 0 # linear velocity
    self.w = 0 # angular velocity
    self.arena_width = 200
    self.t = 0 # simulation timestep
    self.output_data_filename = 'simulation_data.txt'
    self.output_video_filename = 'simulation_video.mp4'
    self.x_path = [] # a series of x coordinates of all points visited
    self.y_path = []  # a series of x coordinates of all points visited

    self.sensor_dirs = [0,
                        pi/2,
                        pi*3/2,
                        pi,
                        ]
    self.n_sensors = len(self.sensor_dirs)
    self.sensor_readings = []

    self.prox_sensors = [] #= ProxSensor_c()
    for i in range(0,self.n_sensors):
      self.prox_sensors.append( ProxSensor_c(self.radius, self.sensor_dirs[i]) )

    # Create a file to store output data
    with open(self.output_data_filename, mode="w") as f:
      pass

    # Remove all frames used to generate previous video
    path_in = 'img/'
    try:
      files = listdir(path_in)
      for file in files:
        file_path = join(path_in, file)
        if isfile(file_path):
          remove(file_path)
      print("All files deleted successfully.")
    except OSError:
      print("Error occurred while deleting files.")

    # remove(self.output_video_filename)
    try:
      remove(self.output_video_filename)
    except OSError:
      print("Error occurred while deleting video")
    self.update_visualisation()




  def updatePosition( self, v, w ):

    # clear stall flags
    self.stall = -1
    self.stall_condition = 0

    # linear velocity can only be -1, 1 or 0
    if v not in [1, -1]:
      v = 0

    # v can only be -1, 1 or 0
    if w not in [1, -1]:
      w = 0
    else:
      # convert w to 1 degree in rads
      w = -w * pi/180

    if v and w:
      v = 0
      w = 0

    # # Save parameters for later.
    # self.v = v
    # self.w = w
    if obstacles:
      for obstacle in obstacles:
        # Detect collision
        self.collisionCheck(obstacle)
        # Update sensors and direction of obstacle
        self.updateSensors(obstacle)

    self.sensor_readings = []

    for i in range(self.n_sensors):
      # print(f'sensor {i}= {round(self.prox_sensors[i].reading, 2)}', end='\t')
      self.sensor_readings.append(round(self.prox_sensors[i].reading, 2))
    # print()

    # Prevent robot from moving if obstructed
    if v == 1 and self.stall_condition == 1:
      v = 0
    elif v == -1 and self.stall_condition == -1:
      v = 0

    # Save parameters for later.
    self.v = v
    self.w = w

    
    # robot matrix, contributions to motion x,y,theta
    r_matrix = [v, 0, w]

    # kinematic matrix
    k_matrix = [
                [ np.cos(self.theta),-np.sin(self.theta),0],
                [ np.sin(self.theta), np.cos(self.theta),0],
                [0,0,1]
               ]

    result_matrix = np.matmul(k_matrix, r_matrix)

    # Attempt move
    self.x += result_matrix[0]
    self.y += result_matrix[1]
    self.theta -= result_matrix[2]

    # Normalise angle to range -pi to pi
    self.theta = atan2(sin(self.theta),cos(self.theta))

    # If motion goes outside of arena, revert x and y to stay
    # at current position
    if self.x > self.arena_width - self.radius:
      self.stall = 1
      self.x -= result_matrix[0]
      self.y -= result_matrix[1]

    if self.x < self.radius:
      self.stall = 1
      self.x -= result_matrix[0]
      self.y -= result_matrix[1]

    if self.y > self.arena_width - self.radius:
      self.stall = 1
      self.y -= result_matrix[1]
      self.x -= result_matrix[0]

    if self.y < self.radius:
      self.stall = 1
      self.y -= result_matrix[1]
      self.x -= result_matrix[0]

    # Once we have updated the robots new global position
    # we should also update the position of its sensor(s)
    for prox_sensor in self.prox_sensors:
      prox_sensor.updateGlobalPosition( self.x, self.y, self.theta )

    self.update_visualisation()

  def update_visualisation(self):

    # for obstacle in obstacles:
    #   self.collisionCheck(obstacle)
    #   self.updateSensors(obstacle)
    #
    # for i in range(self.n_sensors):
    #   print(f'sensor {i}= {round(self.prox_sensors[i].reading, 2)}', end='\t')
    # print()


    # Setup plot
    fig = plt.figure(dpi=120)
    ax = fig.add_subplot(111, aspect='equal',
                         autoscale_on=False,
                         xlim=(0, self.arena_width),
                         ylim=(0, self.arena_width))

    # Add x,y coordinates to series of points visited
    self.x_path.append(self.x)
    self.y_path.append(self.y)

    # Initialise plotted robot
    gui_robot, = ax.plot([], [], 'bo', ms=self.radius * 2.5)
    gui_dir, = ax.plot([], [], 'k-')
    gui_path, = ax.plot([], [], 'r:')
    gui_sensor = ax.plot(*[[],[]]*self.n_sensors,'r-')

    if obstacles:
      obstacles_x = []
      obstacles_y = []
      for obstacle in obstacles:
        obstacles_x.append(obstacle.x)
        obstacles_y.append(obstacle.y)

      obstacle_radius = obstacles[0].radius
      gui_obstacles, = ax.plot([], [], 'mo', ms=obstacle_radius * 2.5)
      gui_obstacles.set_data(obstacles_x, obstacles_y)

    if markers:
      markers_x = []
      markers_y = []
      for marker in markers:
        markers_x.append(marker[0])
        markers_y.append(marker[1])

      gui_markers, = ax.plot([], [], 'go', ms=10)
      gui_markers.set_data(markers_x, markers_y)


    # Draw path taken so far
    gui_path.set_data(self.x_path, self.y_path)

    # Draw robot at position x, y
    gui_robot.set_data(self.x, self.y)
    if self.stall == 1:
      gui_robot.set_color("red")
    else:
      gui_robot.set_color("blue")

    # Draw little indicator to show which direction robot is facing
    tx = self.x + (self.radius * 2 * np.cos(self.theta))
    ty = self.y + (self.radius * 2 * np.sin(self.theta))
    gui_dir.set_data((self.x, tx), (self.y, ty))

    # Draw the sensor beams
    for i in range(self.n_sensors):
      prox_sensor = self.prox_sensors[i]
      ox = prox_sensor.x
      oy = prox_sensor.y
      tx = prox_sensor.x + np.cos(prox_sensor.theta)
      ty = prox_sensor.y + np.sin(prox_sensor.theta)

      gui_sensor[i].set_data((ox, tx), (oy, ty))

    msg = f'time = {self.t} \t \t x = {round(self.x, 3)} \t \t y = {round(self.y, 3)} \t \t theta = {round(self.theta, 3)}'
    msg = (f'time = {self.t}')
    print(msg)
    # print(msg, end=')\t')

    with open(self.output_data_filename, mode="a") as f:
      f.write(msg + '\n')

    plt.title(f'time = {self.t}      x = {round(self.x, 3)}      y = {round(self.y, 3)}      theta = {round(self.theta, 3)}')

    # Output
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.savefig("img/{:05d}_{}".format(self.t, '.png')) # Use 02d modifierfor 5-digit numbers (zero-padded)
    plt.close("all")

    # Update timestep
    self.t += 1

  # The sensor checks if it is in range to an obstruction,
  # and if yes, calculates the simulated proximity reading.
  # if no, determines and error result.
  def collisionCheck(self, obstruction ):
    distance = np.sqrt( ((obstruction.x - self.x)**2) + ((obstruction.y - self.y)**2) )
    distance -= self.radius
    distance -= obstruction.radius

    # print('distance= ', round(distance,2), end='\t')

    # If distance from robot to object becomes negative,
    # transform to positive distance from robot to object
    # so the robot never comes into contact with the object.
    if distance < 0:
      # Set a flag to show obstacle encountered
      self.stall = 1
    #   angle = atan2( obstruction.y - self.y, obstruction.x - self.x)
    #   self.x += distance * np.cos(angle)
    #   self.y += distance * np.sin(angle)

  def updateSensors(self, obstruction):

    # self.stall_condition = 0

    # for each sensor
    # for each obstruction
    for prox_sensor in self.prox_sensors:
      stall_condition = prox_sensor.scanFor( obstruction, self.stall )
      if self.stall_condition == 0:
        self.stall_condition = stall_condition

    # print('stall condition= ', self.stall_condition, end='\t')

  def updateScore(self):

    # 1 minus the difference between wheel speed
    # to encourage straight line travel.
    # square root rewards small differences
    diff = np.abs(((self.vl+1) - (self.vr + 1))) * 0.5

    if diff > 0.0:
      diff =  1 - np.sqrt( diff )
    else:
      diff = 1 # - 0


    # Reward motor activation / penalise no movement
    vel = (np.abs(self.vl) + np.abs(self.vr))/2

    new_score = vel * diff

    if self.stall == 1:
      new_score -= 3

    self.score += new_score

  def make_video(self):

    path_out = self.output_video_filename
    path_in = 'img/'

    fps = 15
    files = [f for f in listdir(path_in) if isfile(join(path_in, f)) and f.endswith(".png")]
    files.sort()
    frame_array = []

    for i in range(len(files)):
      filename = path_in + files[i]
      # Read each file
      img = cv2.imread(filename)
      height, width, layers = img.shape
      size = (width, height)

      # Insert the frames into an image array
      frame_array.append(img)

    # Generate video file
    out = cv2.VideoWriter(path_out, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)

    for i in range(len(frame_array)):
      # Write each frame to the video
      out.write(frame_array[i])

    # Publish video
    out.release()


def add_obstacles(obstacles_x, obstacles_y):
  for x, y in zip(obstacles_x, obstacles_y):
    obstacles.append(Obstacle_c(x, y))

def add_markers(markers_x, markers_y):
  for x, y in zip(markers_x, markers_y):
    markers.append((x, y))


obstacles = []
markers = []

# Create an instance of the simulated Robot at initial position
x_init = 100
y_init = 50
theta_init = np.pi/2
robot = Robot_c(x_init, y_init, theta_init)

# Rename Robot methods for easy use in main program
update_simulation = robot.updatePosition
save_simulation_data = robot.make_video