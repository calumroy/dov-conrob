#!/usr/bin/python

from pylab import *
import control       # Load the controls systems library

#  This script tries to reproduce in python the equations of for the pendulum
#  as can be found on:
#  
#   http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
#  
#  The goal is then to try to control a simulated pendulum with the same parameters.
#
#  Dov Grobgeld <dov.grobgeld@gmail.com>
#  2013-02-24 Sun

M = 0.5    # kg - mass of cart
m = 0.2    # kg - mass of pendulum
b = 0.1    # N/m/s - coefficient of friction for cart
l = 0.3    # Length to pendulum center of mass
I = 0.006  # kg - m^2
F = 0      # N - Force applied to the cart
x = 0      # cart position
theta = 0  # pendulum angle from vertical down
g = 9.8    # gravitational constant

p = I*(M+m)+M*m*l**2 # denominator for the A and B matrices
 
A = array([[0,      1,              0,           0],
           [0, -(I+m*l**2)*b/p,  (m**2*g*l**2)/p,   0],
           [0,      0,              0,           1],
           [0, -(m*l*b)/p,       m*g*l*(M+m)/p,  0]])
B = array([[0],
           [(I+m*l**2)/p],
           [0],
           [m*l/p]])
C = array([[1, 0, 0, 0],
           [0, 0, 1, 0]])
D = array([[0],
           [0]])

poles,vect = eig(A)
# Print vectors to verify that the system is unstable
print poles

K = control.place(A,B, [-1,-1,-1,-1])
print 'K(place)=',K

Q = C.transpose().dot(C)
Q[1,1]=5000
Q[3,3] = 100
R = 1
K,S,E = control.lqr(A,B,Q,R)
print 'K(lqr)=',K

# New control matrix
Ac = A-B.dot(K)
poles,vect = eig(Ac)
print "Ac poles=", poles
