/*
Created on Wed Dec 05 10:15:12 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#ifndef ELFIN_KINEMATIC_SOLVER_H
#define ELFIN_KINEMATIC_SOLVER_H

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Eigen>

namespace elfin3_ikfast_simple_api{

// class for storing and sorting solutions
class LimitObeyingSol
{
public:
  std::vector<double> value;
  double dist_from_seed;

  bool operator<(const LimitObeyingSol& a) const
  {
    return dist_from_seed < a.dist_from_seed;
  }
};

// class for forward and inverse kinematic solutions
class Elfin3KinematicSolver{

public:

/**
 * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
 * @param trans The desired position of the end-effector
 * @param orient The desired orientation of the end-effector
 * @param vfree The vector of the free links. Since elfin is a 6dof robot, there is no free link.
 *              So \p vfree should be set as an empty vector
 * @param seed_state An initial guess solution for the inverse kinematics
 * @param solution A calculated set of joint angles that is closest to \p seed_state
 * @return Returns \c true in the case of success, \c false otherwise.
 */
bool getPositionIK(const Eigen::Vector3d& trans, const Eigen::Matrix3d& orient, const std::vector<double>& vfree,
                   const std::vector<double>& seed_state, std::vector<double>& solution);

/**
 * @brief Given a set of joint angles, compute the pose of the end-effector
 * @param joint_angles The state for which FK is being computed
 * @param trans The calculated position of the end-effector
 * @param orient The calculated orientation of the end-effector
 * @return Returns \c true in the case of success, \c false otherwise.
 */
bool getPositionFK(const std::vector<double>& joint_angles, Eigen::Vector3d& trans, Eigen::Matrix3d& orient);

};

}

namespace elfin5_ikfast_simple_api{

// class for storing and sorting solutions
class LimitObeyingSol
{
public:
  std::vector<double> value;
  double dist_from_seed;

  bool operator<(const LimitObeyingSol& a) const
  {
    return dist_from_seed < a.dist_from_seed;
  }
};

// class for forward and inverse kinematic solutions
class Elfin5KinematicSolver{

public:

/**
 * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
 * @param trans The desired position of the end-effector
 * @param orient The desired orientation of the end-effector
 * @param vfree The vector of the free links. Since elfin is a 6dof robot, there is no free link.
 *              So \p vfree should be set as an empty vector
 * @param seed_state An initial guess solution for the inverse kinematics
 * @param solution A calculated set of joint angles that is closest to \p seed_state
 * @return Returns \c true in the case of success, \c false otherwise.
 */
bool getPositionIK(const Eigen::Vector3d& trans, const Eigen::Matrix3d& orient, const std::vector<double>& vfree,
                   const std::vector<double>& seed_state, std::vector<double>& solution);

/**
 * @brief Given a set of joint angles, compute the pose of the end-effector
 * @param joint_angles The state for which FK is being computed
 * @param trans The calculated position of the end-effector
 * @param orient The calculated orientation of the end-effector
 * @return Returns \c true in the case of success, \c false otherwise.
 */
bool getPositionFK(const std::vector<double>& joint_angles, Eigen::Vector3d& trans, Eigen::Matrix3d& orient);

};

}

namespace elfin10_ikfast_simple_api{

// class for storing and sorting solutions
class LimitObeyingSol
{
public:
  std::vector<double> value;
  double dist_from_seed;

  bool operator<(const LimitObeyingSol& a) const
  {
    return dist_from_seed < a.dist_from_seed;
  }
};

// class for forward and inverse kinematic solutions
class Elfin10KinematicSolver{

public:

/**
 * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
 * @param trans The desired position of the end-effector
 * @param orient The desired orientation of the end-effector
 * @param vfree The vector of the free links. Since elfin is a 6dof robot, there is no free link.
 *              So \p vfree should be set as an empty vector
 * @param seed_state An initial guess solution for the inverse kinematics
 * @param solution A calculated set of joint angles that is closest to \p seed_state
 * @return Returns \c true in the case of success, \c false otherwise.
 */
bool getPositionIK(const Eigen::Vector3d& trans, const Eigen::Matrix3d& orient, const std::vector<double>& vfree,
                   const std::vector<double>& seed_state, std::vector<double>& solution);

/**
 * @brief Given a set of joint angles, compute the pose of the end-effector
 * @param joint_angles The state for which FK is being computed
 * @param trans The calculated position of the end-effector
 * @param orient The calculated orientation of the end-effector
 * @return Returns \c true in the case of success, \c false otherwise.
 */
bool getPositionFK(const std::vector<double>& joint_angles, Eigen::Vector3d& trans, Eigen::Matrix3d& orient);

};

}

#endif
