#!/usr/bin/python
# -*- Encoding: utf-8 -*-

# A second pendulum test. This example starts off adding energy to
# the pendulum until we have enough energy to keep it upright. Only
# then does the system switch to a control state.
#
# Question: Is it possible to control for the energy? Probably not
#           since it involves a square term.
#
# Dov Grobgeld <dov.grobgeld@gmail.com>
# 2013-02-24 Sun

import Euv.Frame as Frame
import Euv.EuvGtk as Euv
import Euv.Color as Color
import Euv.Shapes as Shapes
import ode
import time
import math

def normalize_angle(theta):
  return math.atan2(math.sin(theta),math.cos(theta))

# Create the world through ode
world = ode.World()
gravity = 1
world.setGravity( (0,-gravity,0) )

# Make a heavy ball
ball = ode.Body(world)
M = ode.Mass()
M.setSphere(1000,0.01)
M.mass = 1.0
ball.setMass(M)
ball.setPosition((0,-1,0))

# And a rod without any weight
rod = ode.Body(world)
rod_length = 1
M = ode.Mass()
M.setCylinder(0.01, 2, 0.01,rod_length)
M.mass = 1e-3
rod.setMass(M)
rod.setPosition((0,-0.5,0))

# Connect the rod with the world through a Hinge joint.
world_joint = ode.HingeJoint(world)
world_joint.attach(rod, ode.environment)
world_joint.setAnchor( (0,0,0) )
world_joint.setAxis((0,0,1))

# Connect rod with ball with a fixed joint
rod_ball_joint = ode.FixedJoint(world)
rod_ball_joint.attach(rod, ball)
rod_ball_joint.setFixed()

# Create the viewer window
viewer = Euv.Viewer(size=(600,600),
                    view_port_center = (0,0),
                    view_port_width = 2.5,
                    flip_y = True
                    )

# Do the simulation

total_time = 0.0
dt = 0.04
Kf = 0.1 # Friction force
y0 = -1
requested_energy = 2*rod_length*gravity
first = True
state = 0
while total_time<60:
    angle = world_joint.getAngle()
    angle_rate_of_change = world_joint.getAngleRate()

    # Friction
    friction_force = -angle_rate_of_change * Kf


    state_info = ''
    message = ''
    if state == 0:
      state_info = 'Adding energy'

      # Calculate the energy of the ball
      energy = (ball.getPosition()[1] - y0) * gravity + (angle_rate_of_change*rod_length)**2/2

      # Check if it is high enough to switch to the control state
      if energy < requested_energy:
        # Add energy by pushing at the bottom like pushing someone
        # in a swing in the direction of traveling
        if abs(angle) < math.pi/16:
          control_torque = 2
          if angle_rate_of_change < 0:
            control_torque *= -1
          message = 'Pushing!'
        else:
          # Otherwise turn off the current torque
          control_torque *= -1
      else:
        # Turn off current torque
        control_torque=-control_torque

        # Switch to the control state
        state = 1

    # Control the pendulum by applying a torque at the world
    # pendulum joint.
    else:
      state_info = 'Controlling'
      Kp = 2
      Kd = 1
      control_torque = -normalize_angle(math.pi + angle) * Kp - angle_rate_of_change * Kd 

    torque = friction_force + control_torque

    # Here is the control feedback
    world_joint.addTorque(torque)

    # Show text, the rod, and the ball
    f = Frame.Frame()
    f.add_text(pos=(-0.9,0.5),
               face='Serif 18',
               scale=0.003,
               markup=("<b>%s</b>\n"
                       u"Time=%.2fs\n"
                       u"φ=%.2f\n"
                       u"ω=%.2f\n"
                       u"τ=%.2f\n"
                       "energy=%.2f\n"
                       "%s\n"
                       ) %
               (state_info,total_time, angle, angle_rate_of_change, torque, energy,message),
               color=Color.Color("darkgreen"))

    x,y,z = rod.getPosition()
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

    viewer.add_frame(f)
    time.sleep(0.01)

    # Step the world
    world.step(dt)
    total_time+=dt
    
viewer.wait()
