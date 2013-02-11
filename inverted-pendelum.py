#!/usr/bin/python
# -*- Encoding: utf-8 -*-

# This is a simulation of an inverted pendulum through ode.
# A controller allows changing the torque at the joint of the
# rod with the world.
#
# Dov Grobgeld <dov.grobgeld@gmail.com>
# 2013-02-11 Mon

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
world.setGravity( (0,-1,0) )

# Make a heavy ball
ball = ode.Body(world)
M = ode.Mass()
M.setSphere(1000,0.01)
M.mass = 1.0
ball.setMass(M)
ball.setPosition((0,1,0))
ball.addForce( (20,9,0) )

# And a rod without any weight
rod = ode.Body(world)
M = ode.Mass()
M.setCylinder(0.01, 2, 0.01,1)
M.mass = 1e-3
rod.setMass(M)
rod.setPosition((0,0.5,0))

# Connect the rod with the world through a Hinge joint
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
dt = 0.02
prev_angle = 0
while total_time<15:
    angle = world_joint.getAngle()

    # Control the pendulum by applying a torque at the world
    # pendulum joint.
    
    # Try with different PD parameters to explore stability of the Pendulum!
    Kp = 0
    Kd = 0
    angle_rate_of_change = normalize_angle(angle - prev_angle)/dt
    torque = -angle * Kp - angle_rate_of_change * Kd

    # Here is the control feedback
    world_joint.addTorque(torque)

    # Show text, the rod, and the ball
    f = Frame.Frame()
    f.add_text(pos=(-0.9,0.5),
               face='Serif 18',
               scale=0.003,
               markup=(u"Time=%.2fs\n"
                       u"φ=%.2f\n"
                       u"ω=%.2f\n"
                       u"τ=%.2f\n"
                       u"K<sub>p</sub>=%.2f\n"
                       u"K<sub>d</sub>=%.2f\n"
                       ) %
               (total_time, angle, angle_rate_of_change, torque, Kp, Kd),
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
    prev_angle = angle
    
viewer.wait()
