# Sobot Rimulator - A Robot Programming Tool
# Copyright (C) 2013-2014 Nicholas S. D. McCrea
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
# 
# Email mccrea.engineering@gmail.com for questions, comments, or to report bugs.





from math import *
from utils import linalg2_util as linalg
from utils import math_util

EPS32 = 1.19209e-07

class CanudasDeWitController:

  def __init__( self, supervisor ):
    # bind the supervisor
    self.supervisor = supervisor

    # control gains
    self.gamma = 1.0
    self.kappa = 1.0

    # stored values - for computing next results
    self.prev_time = 0.0

    # key vectors and data (initialize to any non-zero vector)
    self.robot_pos_new_coord    = [ 1.0, 0.0 ]
    self.robot_theta_new_coord  = 0.0    

  def update_variables( self ):
    # generate and store new heading vector
    self.robot_pos_new_coord, self.robot_theta_new_coord = self.calculate_canudas_new_coordinate()

  def execute( self ):
    # calculate the time that has passed since the last control iteration
    current_time = self.supervisor.time()
    dt = current_time - self.prev_time

    qx      = self.robot_pos_new_coord[0]
    qy      = self.robot_pos_new_coord[1]
    qtheta  = self.robot_theta_new_coord

    # Series of check to avoid NaN errors
    if linalg.mag(self.robot_pos_new_coord) <= EPS32 :
      beta    = 0.0
      theta_d = 0.0
      # Defining the new state 
      z1      = 0.0
      z2      = 0.0
      # Defining b1 and b2 parameters
      b1      = 0.0
      b2      = 0.0
    else :
      if -EPS32 <= qx <= EPS32:
        if qx >= 0.0:
          qx = EPS32
        else: 
          qx = -EPS32
      elif -EPS32 <= qy <= EPS32:
        if qy >= 0.0:
          qy = EPS32
        else: 
          qy = -EPS32      
      # Defining the theta_d angle and beta
      beta    = qy / qx
      theta_d = 2.0 * atan( beta )
      # Defining the new state 
      z1      = ( (qx**2 + qy**2) / qy ) * atan( beta )
      z2      = qtheta - theta_d
      # Defining b1 and b2 parameters
      b1      = cos(qtheta) * ((theta_d / beta) - 1) + sin(qtheta) * (theta_d/2 * (1 - (1/beta)**2) + 1/beta)
      b2      = cos(qtheta) * (2 * beta / ((1 + beta**2)*qx)) - sin(qtheta) * (2 / ((1 + beta**2)*qx))

    # Defining the Control actions     
    v       = - self.gamma * b1 * z1
    omega   = - b2 * v - self.kappa * z2

    self.supervisor.set_outputs( v, omega )

    # return a canudas heading vector in the goal reference frame
  def calculate_canudas_new_coordinate( self ):
    # get the inverse of the robot's pose respect to the origin
    robot_inv_pos, robot_inv_theta = self.supervisor.estimated_pose().inverse().vunpack()
    
    # calculate the goal vector in the robot's reference frame
    goal = self.supervisor.goal()
    goal = linalg.rotate_and_translate_vector( goal, robot_inv_theta, robot_inv_pos )

    # calculate the robot vector in the goal reference frame (basically the inverse of the one above)
    robot_theta = - robot_inv_theta
    robot_pos   = linalg.rotate_vector( [ -goal[0], -goal[1] ], robot_theta )

    return robot_pos, robot_theta

    # === FOR DEBUGGING ===
    # self._print_vars( v, omega )

  def _print_vars( self, v, omega ):
    print "\n\n"
    print "=============="
    print "CANUDAS NEW STATE:"
    # print "z1: " + str( z1 )
    # print "z2: " + str( z2 )
    print ""
    print "CONTROL COMPONENTS:"
    # print "kP * eP = " + str( self.kP ) + " * " + str( eP )
    # print "= " + str( self.kP * eP )
    # print "kI * eI = " + str( self.kI ) + " * " + str( eI )
    # print "= " + str( self.kI * eI )
    # print "kD * eD = " + str( self.kD ) + " * " + str( eD )
    # print "= " + str( self.kD * eD )
    print ""
    print "OUTPUTS:"
    print "omega: " + str( omega )
    print "v    : " + str( v )
