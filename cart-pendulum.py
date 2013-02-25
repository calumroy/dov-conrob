#!/usr/bin/python
# -*- encoding: utf-8 -*-

# This demonstration shows the state-space equations and the "place"
# function to choose negative eigenvalues can be used to stabilize
# the inverted pendulum.
#
# References:
#    - http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
#         Description of the mathematics that this demo is based on.
#    - python control module - http://python-control.sourceforge.net
#    - https://github.com/avventi/Slycot.git - A prerequisite for python control.

import Euv.Frame as Frame
import Euv.EuvGtk as Euv
import Euv.Color as Color
import Euv.Shapes as Shapes
import ode
import time
import math
import control
from pylab import *

# Describe the geometry of the system
cart_width = 0.5
cart_height = 0.1
cart_mass = 5
rod_length = 1
ball_mass = 0.1
gravity = 9.8

def calc_control_parameters(M,m,b,l,I):
  """Given a system description, calculate a control matrix"""
  p = I*(M+m)+M*m*l**2 # denominator for the A and B matrices
  g = gravity

  A = array([[0,      1,              0,           0],
             [0, -(I+m*l**2)*b/p,  (m**2*g*l**2)/p,   0],
             [0,      0,              0,           1],
             [0, -(m*l*b)/p,       m*g*l*(M+m)/p,  0]])
  B = array([[0],
             [(I+m*l**2)/p],
             [0],
             [m*l/p]])

  # Place arbitrary negative poles in the control matrix as described.
  K = control.place(A,B, [-1,-1,-1,-1])

  return K[0]

def normalize_angle(theta):
  return math.atan2(math.sin(theta),math.cos(theta))

def get_rotation_around_z(body):
  """Return the rotation of a body around the z axis"""
  rotation = body.getRotation()

  # Negative sign because of the selection of y-axis.
  return -math.atan2(rotation[1],rotation[0])

def draw_world(cart,ball,rod, goal, info=''):
  """Draw a snapshot of the world"""
  f = Frame.Frame()
  x,y,z = cart.getPosition()
  cart_shape = Shapes.rotated_rectangle( (x,y),
                                         0,
                                         cart_width,
                                         cart_height)
  f.add_polygons([cart_shape],
                 color="green",
                 alpha=0.5)

  x,y,z = rod.getPosition()
  angle = get_rotation_around_z(rod)
  rod_shape = Shapes.rotated_rectangle( (x,y),
                                        angle,
                                        0.03,
                                        1)
  f.add_polygons([rod_shape],
                 color="gray50")
  
  x,y,z = ball.getPosition()
  f.add_circle((x,y),
               color='red3',
               alpha=0.8,
               radius=0.1)

  f.add_text(pos=(-2,2),
             face='Serif 36',
             scale=0.003,
             markup=text_info,
             color=Color.Color("darkgreen"))

  # Draw start and goal
  f.add_circle((goal,0),
               color='blue',
               alpha=0.5,
               radius=0.05)

  viewer.add_frame(f)
  time.sleep(0.01)
  
def build_world():
  # Create the world through ode
  density = 1
  start_up = True
  
  world = ode.World()
  world.setGravity( (0,-gravity,0) )
  
  # Make the cart
  cart = ode.Body(world)
  M = ode.Mass()
  M.setBox(density, cart_width, cart_height,cart_height)
  M.mass = cart_mass
  cart.setMass(M)
  cart.setPosition((0,0,0))
  print 'cart mass = ', cart.getMass().mass
  
  # Make a heavy ball
  ball = ode.Body(world)
  M = ode.Mass()
  M.setSphere(density,0.5)
  M.mass = ball_mass
  ball.setMass(M)
  ball.setPosition((0,1,0))
  
  # And a rod with a negligible weight
  rod = ode.Body(world)
  M = ode.Mass()
  M.setCylinder(0.01, 2, 0.01,rod_length)
  M.mass = 1e-2
  rod.setMass(M)
  rod.setPosition((0,0.5,0))
  
  ## Connect the cart to the world through a slider joint
  cart_joint = ode.SliderJoint(world)
  cart_joint.setAxis((1,0,0))
  cart_joint.attach(cart, ode.environment)
  
  # Connect the rod with the cart through a Hinge joint.
  pendulum_joint = ode.HingeJoint(world)
  pendulum_joint.attach(rod, cart)
  pendulum_joint.setAnchor( (0,0,0) )
  pendulum_joint.setAxis((0,0,1))
  
  # Connect rod with ball with a fixed joint
  rod_ball_joint = ode.FixedJoint(world)
  rod_ball_joint.attach(rod, ball)
  rod_ball_joint.setFixed()

  return world,cart,ball,rod,pendulum_joint, cart_joint,rod_ball_joint

# Create the viewer window
viewer = Euv.Viewer(size=(600,600),
                    view_port_center = (0,0),
                    view_port_width = 5,
                    flip_y = True
                    )


max_time = 30
dt = 0.025
Kf = 0.1         # Friction force
goal = 0.5       # Goal position for cart
do_push = True   # Whether to push the ball to test stabilization
push_time = 5.0  # when to push
off_control_time = 15   # turn off control

# Calculate optimal control parameters
K = calc_control_parameters(M = cart_mass,
                            m = ball_mass,
                            b = 0,
                            l = rod_length,
                            I = ball_mass * rod_length**2)

# Build the world and do the simulation
world,cart,ball,rod,pendulum_joint,cart_joint,rod_ball_joint = build_world()
total_time = 0.0
while total_time<max_time:
  message = ''
  if do_push:
    # push the ball!
    if total_time >= push_time and total_time < push_time+dt:
      ball.addRelForce( (-3,0,0) )

    # Make the push_message stay a bit longer
    if total_time >= push_time and total_time < push_time+1:
      message = '<span fgcolor=\"#800000\"><b>pushing</b></span>'
  
  angle = pendulum_joint.getAngle()
  angle_rate_of_change = pendulum_joint.getAngleRate()
  cart_pos = cart.getPosition()[0]
  cart_vel = cart.getLinearVel()[0]
  X = np.array([cart_pos-goal, cart_vel, angle, angle_rate_of_change])

  # Friction
  friction_torque = -angle_rate_of_change * Kf
  pendulum_joint.addTorque(friction_torque)

  if total_time < off_control_time:
    control_push = -X.dot(K)

    # push the cart
    cart.addForce( (control_push, 0,0) )
  else:
    control_push = 0
    message = '<span fgcolor=\"#800000\"><b>Control OFF</b></span>'

  text_info= (u"Time=%.2fs\n"
              u"φ=%.2f rad\n"
              u'ω =%.3f rad/s\n'
              'p = %.3f m\n'
              'v = %.3f m/s\n'
              'F = %.3f N\n'
              '%s\n'
              %(total_time,
                angle,angle_rate_of_change,
                cart_pos, cart_vel,
                control_push,
                message))
  # draw it
  draw_world(cart,ball,rod, goal, text_info)

  # Step the world
  world.step(dt)
  total_time+=dt

viewer.wait()
