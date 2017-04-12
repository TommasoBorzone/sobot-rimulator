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





from utils import linalg2_util as linalg
from utils import math_util

class Pose:

  def __init__( self, *args ):
    if len( args ) == 1: # initialize using a copy constractor ( original_pose )
      try:
        self.__deepcopy__(args[0])
      except WrongTypeExeption:
        raise TypeError( "Trying to copy a Pose from a Non-Pose object" )
    elif len( args ) == 2: # initialize using a vector ( vect, theta )
      vect = args[0]
      theta = args[1]

      self.x = vect[0]
      self.y = vect[1]
      self.theta = math_util.normalize_angle( theta )
    elif len( args ) == 3: # initialize using scalars ( x, y theta )
      x = args[0]
      y = args[1]
      theta = args[2]

      self.x = x
      self.y = y
      self.theta = math_util.normalize_angle( theta )
    else:
      raise TypeError( "Wrong number of arguments. Pose requires 2 or 3 arguments to initialize" )

  def __deepcopy__(self, original_pose):
      orig_vect, orig_theta = original_pose.vunpack()

      self.x = orig_vect[0]
      self.y = orig_vect[1]
      self.theta = orig_theta
      
  # get a new global pose given by this relative pose and the global pose of the reference PR, PrefG ----> PG
  def transform_to( self, reference_pose ):
    rel_vect, rel_theta = self.vunpack()            # elements of this pose (in the relative frame)
    ref_vect, ref_theta = reference_pose.vunpack()  # elements of the reference pose (in the global frame)

    # construct the elements of the transformed pose
    result_vect_d = linalg.rotate_vector( rel_vect, ref_theta )
    result_vect = linalg.add( ref_vect, result_vect_d ) 
    result_theta = ref_theta + rel_theta

    return Pose( result_vect, result_theta )

  # get a new relative pose given by this global pose and the global pose of the reference PG, PrefG ---->PR
  def transform_to_GG2R( self, reference_pose ):
    rel_vect, rel_theta = self.vunpack()            # elements of this pose (in the global frame)
    ref_vect, ref_theta = reference_pose.vunpack()  # elements of the reference pose (in the global frame)

    # construct the elements of the transformed pose
    result_vect_d = linalg.add( rel_vect, [-ref_vect[0], -ref_vect[1]] ) 
    result_vect = linalg.rotate_vector( result_vect_d, -ref_theta )
    result_theta = rel_theta - ref_theta

    return Pose( result_vect, result_theta )
   
  # get a new pose given by inverting this pose, e.g. return the pose of the "world" relative to this pose
  def inverse( self ):
    result_theta = -self.theta
    result_pos = linalg.rotate_vector( [ -self.x, -self.y ], result_theta )

    return Pose( result_pos, result_theta )

  # update pose using a vector
  def vupdate( self, vect, theta ):
    self.x = vect[0]
    self.y = vect[1]
    self.theta = math_util.normalize_angle( theta )

  # update pose using scalars
  def supdate( self, x, y, theta ):
    self.x = x
    self.y = y
    self.theta = math_util.normalize_angle( theta )

  # return the constituents of this pose with location as a vector
  def vunpack( self ):
    return [ self.x, self.y ], self.theta

  # return the constituents of this pose as all scalars
  def sunpack( self ):
    return self.x, self.y, self.theta

  # return the position component of this pose as a vector
  def vposition( self ):
    return [ self.x, self.y ]
