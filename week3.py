#!/usr/bin/python
# -*- Encoding: utf-8 -*
#
# This weeks euv robot contains the following enhancements:
#
#   * Created a baseclass Controller that defines a
#     behavior prototype, like in simiam.
#   * Created a GoToGoal controller that moves to a goal.
#   * Used pylab to plot a graph of the assymptotic angle
#   * Introduced multi goals to make robot take a more complex path.
#
# Dov Grobgeld <dov.grobgeld@gmail.com>
# 2013-02-05 Tue

from math import *
import Euv.Frame as Frame
import Euv.EuvGtk as Euv
import Euv.Color as Color
import Euv.Shapes as Shapes
import time
import pylab

# Constants for encoder arrays
LEFT = 0
RIGHT = 1

def len2(x,y):
  return x**2+y**2

def normalize_angle(theta):
  return atan2(sin(theta),cos(theta))

class State:
  """The state (pose) of the robot"""
  def __init__(self,x,y,phi):
    self.x = x
    self.y = y
    self.phi = phi

  def get_pose(self):
    return self.x,self.y,self.phi

  def set_pose(self,x,y,phi):
    self.x,self.y,self.phi = x,y,phi

class WheelEncoder:
  """An wheel encoder with descrete ticks"""
  def __init__(self,
               radius,
               length,
               ticks_per_rev=1000):
    self.radius = radius
    self.length = length
    self.ticks_per_rev = ticks_per_rev
    self.ticks = 0

  def update_ticks(self,
                   angular_velocity,
                   dt):
    self.ticks += self.distance_to_ticks(angular_velocity * dt)

  def reset_ticks(self):
    self.ticks = 0

  def distance_to_ticks(self, distance):
    return ceil((distance*self.ticks_per_rev)/(2*pi))

  def ticks_to_distance(self, ticks):
    return (ticks*2*pi)/self.ticks_per_rev

  def get_ticks(self):
    return self.ticks

  def get_ticks_per_rev(self):
    return self.ticks_per_rev

# Define controllers.
#
# As python does not enforce an interface relationship between
# a baseclass and a derived class, we will just create the
# classes to have the interface. This is known as duck-typing.
# If it looks like a duck and quacks like a duck...
#
# A controller only needs to implement the function
#
#   execute(self, robot, state_estimate, inputs)
#
# that returns outputs.
#
class ControllerGoToGoal:
  """A controller taking as input (x_g, y_g, v) and returns (v,w)"""
  def __init__(self, kp = 10, kd = 0, ki = 0, d_stop = 0.02):
    self.kp = kp
    self.kd = kd
    self.ki = ki
    self.sum_e_k = 0
    self.prev_e_k = None
    self.phi_g = 0

    # Square distance from goal where to stop
    self.epsilon2 = d_stop**2

  def get_dir_to_goal(self):
    """Return the last measured direction to the goal"""
    return self.phi_g

  def get_e_k(self):
    """Return the last error"""
    return self.prev_e_k

  def execute(self, robot, state_estimate, inputs, dt):
    (x_g,y_g,v) = inputs
    (x,y,phi) = state_estimate

    # Direction to goal
    dx = x_g-x
    dy = y_g-y
    phi_g = atan2(dy,dx)
    self.phi_g = phi_g

    # heading error
    e_k = normalize_angle(phi_g-phi)
            
    # Zero out prev_ek for first time
    if self.prev_e_k==None:
      self.prev_e_k = e_k

    # PID for heading
    w = self.kp*e_k + self.ki*(self.sum_e_k+e_k*dt) + self.kd*(e_k-self.prev_e_k)/dt
    
    # save errors
    self.sum_e_k += e_k*dt
    self.prev_e_k = e_k

    # stop when sufficiently close
    delta = len2(dx,dy)

    if delta < self.epsilon2:
      return (0,0)

    return (v,w)
  
class Robot:
  """The robot model contains two differential wheels"""
  def __init__(self,
               axis_length,
               wheel_radius,
               viewer=None,
               init=None,
               goals=None,
               ticks_per_rev = 50,
               initial_state = State(0,0,0)   # Estimated state of robot
               ):
    self.axis_length = axis_length  # L
    self.wheel_radius = wheel_radius  # R
    self.viewer = viewer
    self.path = []
    self.time = 0
    self.init = init
    self.goals = goals
    self.state = initial_state
    self.left_encoder = WheelEncoder(wheel_radius,axis_length,ticks_per_rev)
    self.right_encoder = WheelEncoder(wheel_radius,axis_length,ticks_per_rev)
    
  def uni_to_diff(self,
                  velocity,
                  rotation_speed):
    """Translate from unicycle dynamicls to differential
    drive dynamics"""
    L = self.axis_length
    R = self.wheel_radius
    w = rotation_speed
    v = velocity
    v_l = 1.0*(2*v-w*L)/(2*R)
    v_r = 1.0*(2*v+w*L)/(2*R)
    return v_l,v_r
    
  def diff_to_uni(self,
                  velocity_left,
                  velocity_right):
    """Translate from differential drive angular velocities to
    unicycle velocity and direction."""
    L = self.axis_length
    R = self.wheel_radius
    v_l = velocity_left                 
    v_r = velocity_right
    v = 1.0*R/2*(v_r + v_l)
    w = 1.0*R/L*(v_r - v_l)
    return v,w

  def set_pose(self,x,y,phi):
    self.state.set_pose(x,y,phi)
    self.path = [(x,y)]  # Reset the path

  def set_wheel_speeds(self, velocity_left, velocity_right):
    self.velocity_left = velocity_left
    self.velocity_right = velocity_right

  def draw_robot(self, draw_path=True, text_info=''):
    """Create a new frame and draw a representation of the robot in it"""
    x,y,phi = self.state.get_pose()
    if self.viewer:
      f = Frame.Frame()

      if draw_path:
        f.add_lines(color='black',
                    alpha=0.5,
                    lines=[self.path[:]],
                    linewidth=0.02)
      
      # Draw start and goal
      f.add_circle(self.init,
                   color='green',
                   alpha=0.5,
                   radius=0.05)
      for g in self.goals:
        f.add_circle(g,
                     color='blue',
                     alpha=0.5,
                     radius=0.05)

      # Our robot representation is built of an arrow head
      # with two wheels.
      poly = Shapes.arrow_head_polygon((x, y),
                                       phi,
                                       scale=0.02)
      f.add_polygons([poly],
                     color="red",
                     alpha=0.5)
      wheels = Shapes.rectangle_pair((x,y),
                                     5.,2.,7.,
                                     angle=phi,
                                     scale=0.02)
      f.add_polygons(wheels,
                     color="black",
                     alpha=0.5)

      # Add some anotation
      f.add_text(pos=(-0.5,-0.5), size=0.1, text="Time=%.2fs"%self.time,color="darkgreen")
      dy = -0.6
      for i,s in enumerate(text_info.split('\n')):
        f.add_text(pos=(-0.5,dy-0.1*i), size=0.07,
                   text=s,color="darkgreen")

      self.viewer.add_frame(f)
      time.sleep(0.01)
    
  def step(self,
           dt):
    """Take a timestep and draw the new pose"""
    prev_left_ticks = self.left_encoder.get_ticks()
    prev_right_ticks = self.right_encoder.get_ticks()

    self.left_encoder.update_ticks(self.velocity_left,dt)
    self.right_encoder.update_ticks(self.velocity_right,dt)

    left_ticks = self.left_encoder.get_ticks()
    right_ticks = self.right_encoder.get_ticks()

    rad_per_tick = 2*pi/self.left_encoder.get_ticks_per_rev()
    m_per_rad = rad_per_tick * self.wheel_radius

    # Distances traveled by left and right wheels
    drad_left = (left_ticks - prev_left_ticks)*rad_per_tick
    drad_right = (right_ticks - prev_right_ticks)*rad_per_tick
    dm_left = drad_left * m_per_rad
    dm_right = drad_right * m_per_rad

    # Convert to uni cycle model
    dm_center = 0.5*(dm_left+dm_right)
    dphi = (dm_right - dm_left)/self.axis_length

    # Get previous pose
    x,y,phi = self.state.get_pose()

    # Do linear integration. simiam uses ode45, which seems to
    # be an overkill!
    phi += dphi
    dx = dm_center * cos(phi)
    x+= dx
    dy = dm_center * sin(phi)
    y += dy

    self.state.set_pose(x,y,phi)
    self.path += [(x,y)]
    self.time+=dt

  def get_pose(self):
    """Return the current robot direction based on the position
    and direction of the differential wheels"""    
    return self.state.get_pose()

  def get_time(self):
    return self.time

  def get_v_and_w(self):
    return self.diff_to_uni(self.velocity_left,
                            self.velocity_right)
  def get_dir_to_goal(self, goal_index):
    x,y,phi = self.get_pose()
    return atan2(self.goals[goal_index][1]-y,self.goals[goal_index][0]-x)
    
  def reached_goal(self, goal_index):
    """Returns true if we have reached the goal"""
    x,y,phi = self.state.get_pose()
    epsilon = 0.02**2
    return len2(x-self.goals[goal_index][0],y-self.goals[goal_index][1]) < epsilon


# Create the viewer window
viewer = Euv.Viewer(size=(600,600),
                    view_port_center = (0,0),
                    view_port_width = 3,
                    flip_y = True
                    )

# Create the robot
wheel_radius = 0.1
wheel_apart = 0.2
init = (0,0)
goals = [(0.7,0.7),(0.7,-0.2),(-0.5,0),(-0.7, 0.7),(0,0)]
robot = Robot(wheel_apart,
              wheel_radius,
              viewer=viewer,
              init=init,
              goals=goals) 

# Set the initial position of the robot
robot.set_pose(0.,0.,0)
velocity = 0.1
vel_l, vel_r = robot.uni_to_diff(velocity, 0)
robot.set_wheel_speeds(vel_l,vel_r)

# Do PID loop
dt = 1.0

steps_num = 0
epsilon = 1e-3

# Create our controller
go_to_goal = ControllerGoToGoal(kp=0.5)

# Loop until we reach the goal or we reach 400 steps.
plot_t = []
plot_phi = []
plot_phi_g = []

w = 0

# Have the robot change its goal along this path
for i,goal in enumerate(goals):
  vel_l, vel_r = robot.uni_to_diff(velocity, w)
  robot.set_wheel_speeds(vel_l,vel_r)
  while steps_num < 800:
    (v,w) = robot.get_v_and_w()
    (v,w) = go_to_goal.execute(robot, robot.get_pose(), (goal[0],goal[1],v), dt)
  
    # Update the angular speed but not the velocity
    vel_l, vel_r = robot.uni_to_diff(v, w)
  
    # Set the new direction
    robot.set_wheel_speeds(vel_l,vel_r)
    (x,y,phi) = robot.get_pose()
  
    # Get textual info to display
    phi_g = go_to_goal.get_dir_to_goal()
    e_k = go_to_goal.get_e_k()
    robot.draw_robot(text_info= (
                     u"φ=%.2f\n"
                     u"φ_g=%.2f\n"
                     'e_k=%.3f\n'
                     u'ω =%.3f\n'
                     'v = %.3f\n'
                     'vel_l=%.3f\n'
                     'vel_r=%.3f'
                     %(phi,phi_g,e_k,w,v,vel_l,vel_r)))
  
    steps_num+=1
    if robot.reached_goal(i):
      break
    robot.step(dt)
  
    # Accumulate info to plot
    plot_t += [robot.get_time()]
    plot_phi += [normalize_angle(phi)]
    plot_phi_g += [normalize_angle(phi_g)]

pylab.plot(plot_t,plot_phi,label=u'φ' )
pylab.plot(plot_t,plot_phi_g,label=u'φ_g')
pylab.legend(loc=4)
pylab.title('Robot angle')
pylab.show()

