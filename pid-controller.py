# Illustrate a robot traveling from the starting point at 0,0
# at phi=0 and traveling to 1,1 using a P-controller.
#
# Illustrations made through https://github.com/dov/EuclideanGeometryViewer/
#
# Dov Grobgeld <dov.grobgeld@gmail.com>
# 2013-02-05 Tue

from math import *
import Euv.Frame as Frame
import Euv.EuvGtk as Euv
import Euv.Color as Color
import Euv.Shapes as Shapes
import time

def len2(x,y):
  return x**2+y**2

def normalize_angle(theta):
  return atan2(sin(theta),cos(theta))

class Robot:
  """The robot model is more general than is needed for this
  assignment. But it is probably useful for further assignments
  in the course and should be put in a separate library"""
  def __init__(self,
               axis_length,
               wheel_radius,
               viewer=None,
               init=None,
               goal=None):
    self.axis_length = axis_length  # L
    self.wheel_radius = wheel_radius  # R
    self.viewer = viewer
    self.path = []
    self.set_pose(0,0,0,10,5)
    self.time = 0
    self.init = init
    self.goal = goal
    
  def physical_to_robot(self,
                        velocity,
                        rotation_speed):
    L = self.axis_length
    R = self.wheel_radius
    w = rotation_speed
    v = velocity
    v_l = 1.0*(2*v-w*L)/(2*R)
    v_r = 1.0*(2*v+w*L)/(2*R)
    return v_l,v_r
    
  def robot_to_physical(self,
                        velocity_left,
                        velocity_right):
    L = self.axis_length
    R = self.wheel_radius
    v_l = velocity_left                 
    v_r = velocity_right
    v = 1.0*R/2*(v_r + v_l)
    w = 1.0*R/L*(v_r - v_l)
    return v,w

  def draw_robot(self, draw_path=True):
    """Create a new frame and draw a representation of the robot in it"""
    if self.viewer:
      f = Frame.Frame()
      if draw_path:
        f.add_lines(color=Color.Color("black"),
                    lines=[self.path[:]],
                    linewidth=0.01)
      
      # Draw start and goal
      for p,color in ( (self.init, "green"),
                       (self.goal, "blue") ):
        if p:
          f.add_circle(p,
                       color=color,
                       alpha=0.5,
                       radius=0.05)

      # Our robot representation is built of an arrow head
      # with two wheels.
      poly = Shapes.arrow_head_polygon((self.x, self.y),
                                       self.phi,
                                       scale=0.02)
      f.add_polygons([poly],
                     color=Color.Color("red"),
                     alpha=0.5)
      wheels = Shapes.rectangle_pair((self.x,self.y),
                                     5.,2.,7.,
                                     angle=self.phi,
                                     scale=0.02)
      f.add_polygons(wheels,
                     color=Color.Color("black"),
                     alpha=0.5)

      # Add some anotation
      f.add_text(pos=(-0.5,-0.5),
                 size=0.1,
                 text="Time=%.2fs"%(self.time),
                 color=Color.Color("darkgreen"))

      self.viewer.add_frame(f)
      time.sleep(0.01)
    
  def set_pose(self, x,y,phi, v=1,w=0,reset_path=True):
    """Set the position and speed and rotation of the robot"""
    self.x = x
    self.y = y
    self.phi = phi
    self.v = v
    self.w = w
    # Calculate dependent internal coordinates
    self.v_l,self.v_r = self.physical_to_robot(v,w)
    if reset_path:
      self.path = []
    
  def step(self,
           dt):
    """Take a timestep and draw the new pose"""
    dx = self.v * cos(self.phi) * dt
    dy = self.v * sin(self.phi) * dt
    self.x += dx
    self.y += dy
    self.phi += self.w * dt
    self.path += [(self.x,self.y)]
    self.time+=dt
    self.draw_robot()

  def get_pos(self):
    """Return the current robot direction"""
    return (self.x,self.y,self.phi)
    
  def set_direction(self,phi):
    """Set a new direction of our robot"""
    self.phi = phi

# Create the viewer window
viewer = Euv.Viewer(size=(600,600),
                    view_port_center = (0,0),
                    view_port_width = 3,
                    )

# Create the robot
wheel_radius = 0.1
wheel_apart = 0.2
init = (0,0)
goal = (0,1)
robot = Robot(wheel_apart,wheel_radius,
              viewer=viewer,
              init=init,
              goal=goal) 

# Do PID loop
dt = 0.1

robot.set_pose(0,0,0, 0.1, 0)
steps_num = 0
epsilon = 1e-3

# Loop until we reach the goal or we reach 1000 steps.
while steps_num < 1000:
  x,y,phi = robot.get_pos()
  if len2(x-goal[0],y-goal[1]) < epsilon:
    break
  robot.step(dt)

  # The direction to the goal
  dir_to_goal = atan2(goal[1]-y,goal[0]-x)

  # Our current error in direction
  error = normalize_angle(dir_to_goal - phi)

  # Correction strength
  Kp = 0.02
  correction = Kp*error

  # Set the new direction
  robot.set_direction(phi + correction)
  steps_num+=1

viewer.wait()
